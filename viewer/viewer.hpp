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
void viewCloud(const std::vector<Eigen::Vector2d> &pcd);
Eigen::Vector2d ComputeMean(const std::vector<Eigen::Vector2d> &pcd);
Eigen::Matrix2d ComputeCovariance(const std::vector<Eigen::Vector2d> &source, 
const std::vector<Eigen::Vector2d> &target,const Eigen::Vector2d source_mean, 
const Eigen::Vector2d target_mean);
Eigen::Matrix3d ComputeTransformation(const std::vector<Eigen::Vector2d> &source, 
const std::vector<Eigen::Vector2d> &target);
std::vector<Eigen::Vector2d> ApplyTransformation(const std::vector<Eigen::Vector2d> &pcd,
const Eigen::Matrix3d &transformation_matrix);
double ComputeError(const std::vector<Eigen::Vector2d> &target_pc, 
const std::vector<Eigen::Vector2d> &transformed_pc);
std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> CreateGridMap(const std::vector<Eigen::Vector2d> &pcd, 
double pixel_size);
std::tuple<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> FindNearestNeighbors(const std::unordered_map<GridIndex,std::vector<Eigen::Vector2d>> &grid, 
const std::vector<Eigen::Vector2d> &source_pc, double grid_size);
