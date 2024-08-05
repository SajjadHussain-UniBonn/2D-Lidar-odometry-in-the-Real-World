#include "viewer.hpp"
#include <algorithm>
#include <iostream>
#include <open3d/Open3D.h>
#include <open3d/core/ShapeUtil.h>
#include <utility>
#include <vector>

void viewCloud(const std::vector<Eigen::Vector2d> &pcd) 
{
  std::vector<Eigen::Vector3d> pts(pcd.size());
  std::transform(pcd.cbegin(), pcd.cend(), pts.begin(), [](const auto &p) {
    return Eigen::Vector3d(p.x(), p.y(), 0.0);
  });
  open3d::geometry::PointCloud pointcloud{pts};
  pointcloud.PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
  open3d::visualization::DrawGeometries(
      {std::make_shared<open3d::geometry::PointCloud>(pointcloud)});
}
Eigen::Vector2d ComputeMean(const std::vector<Eigen::Vector2d> &pcd)
{
  Eigen::Vector2d zero = Eigen::Vector2d::Zero();
  Eigen::Vector2d sum = std::accumulate(pcd.begin(), pcd.end(), zero);
  Eigen::Vector2d mean = sum / pcd.size();
  return mean;
}
Eigen::Matrix2d ComputeCovariance(const std::vector<Eigen::Vector2d> &source, 
const std::vector<Eigen::Vector2d> &target,const Eigen::Vector2d source_mean, 
const Eigen::Vector2d target_mean)
{
  Eigen::Matrix2d covariance_matrix = Eigen::Matrix2d::Zero();
  for(size_t i=0;i< source.size(); i++)
  {
    Eigen::Vector2d source_deviation = source[i] - source_mean;
    Eigen::Vector2d target_deviation = target[i] - target_mean;
    covariance_matrix += source_deviation * target_deviation.transpose();
  }
  return covariance_matrix;
}
Eigen::Matrix3d ComputeTransformation(const std::vector<Eigen::Vector2d> &source, 
const std::vector<Eigen::Vector2d> &target)
{
  Eigen::Matrix3d transformation_matrix = Eigen::Matrix3d::Zero();
  Eigen::Vector2d target_mean = ComputeMean(target);
  Eigen::Vector2d source_mean = ComputeMean(source);
  Eigen::Matrix2d covariance_matrix = ComputeCovariance(source,target,source_mean,target_mean);
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance_matrix,Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();
  Eigen::Matrix2d rotation_matrix = V*(U.transpose());
  Eigen::Vector2d translation_vector = target_mean - (rotation_matrix*source_mean);
  transformation_matrix.block<2,2>(0,0) = rotation_matrix;
  transformation_matrix.block<2,1>(0,2) = translation_vector;
  transformation_matrix(2,2) = 1;
  return transformation_matrix;
}
std::vector<Eigen::Vector2d> ApplyTransformation(const std::vector<Eigen::Vector2d> &pcd,
const Eigen::Matrix3d &transformation_matrix)
{
  std::vector<Eigen::Vector2d> tranformed_pc;
  //Get Transformation
  Eigen::Matrix2d rotation_matrix = transformation_matrix.block<2,2>(0,0);
  Eigen::Vector2d translation_vector = transformation_matrix.block<2,1>(0,2);
  //Transform point cloud point by point
  for(size_t i=0; i< pcd.size();i++)
  {
    tranformed_pc.push_back(rotation_matrix*pcd[i] + translation_vector);
  }
  //Release the extra allocated  memory
  tranformed_pc.shrink_to_fit();
  return tranformed_pc;
}
double ComputeError(const std::vector<Eigen::Vector2d> &target_pc, 
const std::vector<Eigen::Vector2d> &transformed_pc)
{
  double error = 0;
  for(size_t i=0; i<target_pc.size();i++)
  {
    error += (target_pc[i] - transformed_pc[i]).squaredNorm();
  }
  return error;
}
std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> CreateGridMap(const std::vector<Eigen::Vector2d> &pcd, 
double grid_size)
{
  std::unordered_map<GridIndex, std::vector<Eigen::Vector2d>> grid;
  for (const auto &point : pcd) 
  {
    GridIndex index {static_cast<int>(point[0] / grid_size),
    static_cast<int>(point[1] / grid_size)};
    grid[index].push_back(point);
  }
  return grid;
}
std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> FindNearestNeighbors(const std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> &grid, 
const std::vector<Eigen::Vector2d> &source_pc, double grid_size)
{
  std::vector<Eigen::Vector2d> source_correspondences;
  std::vector<Eigen::Vector2d> target_correspondences;
  double minimum_distance = INFINITY;
  double distance = INFINITY;
  Eigen::Vector2d nearest_neighbor = Eigen::Vector2d::Zero();
  bool neighbor_found = false;
  for(const Eigen::Vector2d &query_point:source_pc)
  {
    neighbor_found = false;
    minimum_distance = INFINITY;
    GridIndex query_index {static_cast<int>(query_point[0] / grid_size), 
    static_cast<int>(query_point[1] / grid_size)};
    // Iterate through the current cell and adjacent cells
    for (int dx = -1; dx <= 1; ++dx) 
    {
      for (int dy = -1; dy <= 1; ++dy) 
      {
        GridIndex neighbor_index {query_index.x + dx, query_index.y + dy};
        // Check if the neighbor cell exists in the grid map
        if (grid.find(neighbor_index) != grid.end()) 
        {
          neighbor_found = true;
          std::vector<Eigen::Vector2d> list = grid.at(neighbor_index);
          for (const auto &point : list) 
          {
            distance = (point - query_point).norm();
            if (distance < minimum_distance) 
            {
              minimum_distance = distance;
              nearest_neighbor = point;
            }
          }
        }
      }
    }
    if(neighbor_found)
    {
      source_correspondences.emplace_back(query_point);
      target_correspondences.emplace_back(nearest_neighbor);
    }
  }
  return  std::make_tuple(source_correspondences,target_correspondences);
}
Eigen::Matrix3d ICP(const std::vector<Eigen::Vector2d> &source_, 
const std::vector<Eigen::Vector2d> &target,const double &grid_size)
{
  std::vector<Eigen::Vector2d> source = source_;
  int maximum_iteration = 30;
  int iteration_counter = 0;
  double old_error = INFINITY;
  double error = INFINITY;
  Eigen::Matrix3d final_transformation_matrix = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d current_tranformation_matrix = Eigen::Matrix3d::Identity();
  std::vector<Eigen::Vector2d> source_correspondences;
  std::vector<Eigen::Vector2d> target_correspondences;
  std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> grid_map = CreateGridMap(target,grid_size);
  while(true)
  {
    iteration_counter++;
    std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> nearest_neighbors = 
    FindNearestNeighbors(grid_map,source,grid_size);
    source_correspondences = std::get<0>(nearest_neighbors);
    target_correspondences = std::get<1>(nearest_neighbors);
    // viewCloud(source_correspondences);
    // viewCloud(target_correspondences);
    // std::cout<<source_correspondences.size()<<std::endl;
    // std::cout<<target_correspondences.size()<<std::endl;
    current_tranformation_matrix = ComputeTransformation(source_correspondences,target_correspondences);
    final_transformation_matrix = current_tranformation_matrix*final_transformation_matrix;
    source= ApplyTransformation(source,current_tranformation_matrix);
    error = ComputeError(target,source);
    if(error==old_error || iteration_counter==maximum_iteration)
    {
      return final_transformation_matrix;
      
    }
    old_error = error;
    // std::cout<<error<<std::endl;
    source_correspondences.clear();
    target_correspondences.clear();
  }
}
std::vector<Eigen::Vector2d> ConcatenatePointClouds(const std::vector<Eigen::Vector2d> &first, 
const std::vector<Eigen::Vector2d> &second)
{
  std::vector<Eigen::Vector2d> result = first;
  result.insert(result.end(),second.begin(),second.end());
  return result;
}
std::vector<Eigen::Vector2d> RegisterPointClouds(const std::vector<Eigen::Vector2d> &source,
const std::vector<Eigen::Vector2d> &target,const double &grid_size)
{

  std::vector<Eigen::Vector2d> transformed_PC;
  std::vector<Eigen::Vector2d> registered_PCs;
  Eigen::Matrix3d transformation_matrix = ICP(source,target,grid_size);
  transformed_PC = ApplyTransformation(source,transformation_matrix);
  registered_PCs = ConcatenatePointClouds(target,transformed_PC);
  return registered_PCs;
}
std::vector<Eigen::Vector2d> RegisterPointClouds_KnownCorrespondences(const std::vector<Eigen::Vector2d> &source,
const std::vector<Eigen::Vector2d> &target)
{
  std::vector<Eigen::Vector2d> transformed_PC;
  std::vector<Eigen::Vector2d> registered_PCs;
  Eigen::Matrix3d transformation_matrix = ComputeTransformation(source,target);
  transformed_PC = ApplyTransformation(source,transformation_matrix);
  registered_PCs = ConcatenatePointClouds(target,transformed_PC);
  return registered_PCs;
}
std::vector<Eigen::Vector2d> DownSample(const std::vector<Eigen::Vector2d> &pcd, 
const double &grid_size, const int &n_points)
{
  std::vector<Eigen::Vector2d> sampled_points;
  std::unordered_map<GridIndex,int> grid2D;
  for(const auto &query_point:pcd)
  {
    GridIndex query_index {static_cast<int>(query_point[0] / grid_size), 
    static_cast<int>(query_point[1] / grid_size)};
    if(grid2D.find(query_index)==grid2D.end())
    {
      grid2D[query_index]=1;
      sampled_points.emplace_back(query_point);
    }
    else if(grid2D[query_index]< n_points)
    {
      grid2D[query_index]++;
      sampled_points.emplace_back(query_point);
    }
  }
  return sampled_points;
}