#pragma once
#include <Eigen/Core>
#include <vector>

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
