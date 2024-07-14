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
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance_matrix,Eigen::ComputeThinU | Eigen::ComputeThinV);
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
    error += (target_pc[i] - transformed_pc[i]).norm();
  }
  return error;
}