#pragma once
#include <Eigen/Core>
#include <vector>

/* This function draws given point cloud
Input: point cloud
Output: Window to show given point cloud
*/
void viewCloud(const std::vector<Eigen::Vector2d> &pcd);



