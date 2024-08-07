#pragma once
#include <Eigen/Core>
#include <vector>
#include <unordered_map>

// Define a GridIndex structure to represent grid cell indices
struct GridIndex 
{
    int x, y;
    bool operator==(const GridIndex& other) const {
        return x == other.x && y == other.y;
    }
};
// Hash function for GridIndex to be used in unordered_map
namespace std 
{
    template <>
    struct hash<GridIndex> 
    {
        std::size_t operator()(const GridIndex& k) const 
        {
            return ((1 << 20) - 1) & (k.x * 73856093 ^ k.y * 19349663);
        }
    };
}
namespace LaserScanRegistration
{
    /*This function computes the mean of given points
    Input: 
        arg1: vector of 2d points
    Output: vector2d(mean of all points)*/
    Eigen::Vector2d ComputeMean(const std::vector<Eigen::Vector2d> &pcd);
    /*This function computes covariance matrix of two 2D point clouds
    Input:
        arg1: vector of 2d points (source point cloud)
        arg2: vector of 2d points (target point cloud)
        arg3: vector2d(mean of source point cloud)
        arg4: vector2d(mean of target point cloud)
    Output: Matrix2d */ 
    Eigen::Matrix2d ComputeCovariance(const std::vector<Eigen::Vector2d> &source,
    const std::vector<Eigen::Vector2d> &target,const Eigen::Vector2d &source_mean, 
    const Eigen::Vector2d &target_mean);
    /*This function computes transformation matrix(rotation & translation) required to align 
    two 2D point clouds
    Input:
        arg1: vector of 2d points (source point cloud)
        arg2: vector of 2d points (target point cloud)
    Output: Matrix3d*/ 
    Eigen::Matrix3d ComputeTransformation(const std::vector<Eigen::Vector2d> &source, 
    const std::vector<Eigen::Vector2d> &target);
    /*This function applies transformation matrix(rotation & translation) to 
    to-be-transformed point cloud
    Input:
        arg1: vector of 2d points (to-be-transformed point cloud)
        arg2: Matrix3d (transformation matrix)
    Output: vector of 2D points (transformed point cloud)*/
    std::vector<Eigen::Vector2d> ApplyTransformation(std::vector<Eigen::Vector2d> &pcd,
    const Eigen::Matrix3d &transformation_matrix);
    /*This function computes RMS error between 2D points of two point clouds
    Input:
        arg1: vector of 2d points (target point cloud)
        arg2: vector of 2d points (transformed point cloud)
    Output: RMS error*/
    double ComputeError(const std::vector<Eigen::Vector2d> &target_pc, 
    const std::vector<Eigen::Vector2d> &transformed_pc);
    /*This function voxalizez a point cloud
    Input:
        arg1: vector of 2d points (point cloud)
        arg2: voxel_size
    Output: unordered Grid map*/
    std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> CreateGridMap(const std::vector<Eigen::Vector2d> &pcd, 
    double pixel_size);
    /*This function finds nearest neighburs of points in a point cloud in another point cloud
    Input:
        arg1: unordered grid map
        arg2: vector of 2d points (source point cloud)
        arg3: voxel_size
    Output: tuple of vectors of corresponding nearest neighbours*/
    std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> FindNearestNeighbors(const std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> &grid, 
    const std::vector<Eigen::Vector2d> &source_pc, double &grid_size);
    /*This function applies ICP for unknown correspondences and transforms to-be-transformed point cloud
    Input:
        arg1: vector of 2d points (source point cloud)
        arg2: vector of 2d points (target point cloud)
        arg3: voxel_size
    Output: returns nothing*/
    void ICP(std::vector<Eigen::Vector2d> &source, 
    const std::vector<Eigen::Vector2d> &target,const double &grid_size);
    /*This function down samples a point cloud
    Input:
        arg1: vector of 2d points (point cloud)
        arg2: voxel_size
        arg3: repetition score (if you want to add single or multiple points in a voxel)
    Output: vector of 2d points (sampled point cloud)*/
    std::vector<Eigen::Vector2d> DownSample(const std::vector<Eigen::Vector2d> &pcd, const double &grid_size, const int &n_points);
}//end of namespace


