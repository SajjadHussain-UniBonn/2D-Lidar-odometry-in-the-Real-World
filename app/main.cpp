#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"
using namespace dataset;

int main()
{
    // Get data path through CMake_variable
    std::string data_root_dir = PROJECT_ROOT_DIR;
    //Read laserscan data directory in a list
    LaserScanDataset ls_dataset(data_root_dir);
    //Access one point cloud at a time from the list
    std::vector<Eigen::Vector2d> target_pc = ls_dataset[0];
    std::vector<Eigen::Vector2d> source_pc = ls_dataset[1];
    //View single Point cloud
    // viewCloud(pointcloud_points);
    //Compute mean of Point clouds
    Eigen::Vector2d target_mean = ComputeMean(target_pc);
    Eigen::Vector2d source_mean = ComputeMean(source_pc);
    std::cout << "Mean point: (" << target_mean.x() << ", " << target_mean.y() << ")" << std::endl;
    //Compute Covariance Matrix
    Eigen::Matrix2d covariance_matrix = ComputeCovariance(source_pc,target_pc,source_mean,target_mean);
   // Iterate through rows
    for (int i = 0; i < covariance_matrix.rows(); ++i) {
        // Iterate through columns
        for (int j = 0; j < covariance_matrix.cols(); ++j) {
            std::cout << covariance_matrix(i, j) << " ";
        }
        // Move to the next line after each row
        std::cout << std::endl; 
    }
    return 0;
}