#include <iostream>
#include "dataloader.hpp"
#include "scanRegistration.hpp"
#include "viewer.hpp"
#include <open3d/Open3D.h>
#include <chrono>

int main()
{  
    // Get the starting time
    auto start = std::chrono::high_resolution_clock::now();
    // Get data path through CMake_variable
    std::string data_root_dir = PROJECT_ROOT_DIR;
    //Read laserscan data directory in a list
    dataset::LaserScanDataset ls_dataset(data_root_dir);
    //declare source and target point clouds
    std::vector<Eigen::Vector2d> target; 
    std::vector<Eigen::Vector2d> source=ls_dataset[0];
    // hyperparameters for the given data
    const double grid_size = 0.1;
    const double donsample_gridsize = 0.085;
    const int repetition_score = 1;
    int total_scans = ls_dataset.size();
    //Registration Process
    std::cout<<"Progress of Registering Clouds"<<std::endl;
    for(int i=1;i<total_scans;i++)
    {
        target = ls_dataset[i];
        //run ICP for unknown correspondences
        LaserScanRegistration::ICP(source,target,grid_size);
        // append new transformed point cloud in already_registered point clouds
        source.insert(source.end(),target.begin(),target.end());
        // Down-sample registered point clouds
        source = LaserScanRegistration::DownSample(source,donsample_gridsize,repetition_score);
        target.clear();
        // number of registered scans
        std::cout<<"\r "<<i<<std::flush;
    }
    //Get the end time
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout<<std::endl;
    std::cout<<"Execution time = "<<duration.count()<<std::endl;
    std::cout<<"Size of registered Point Cloud = "<<source.size()<<std::endl;
    //View final map
    viewCloud(source);
    std::cout<<"Process Completed"<<std::endl;
    return 0;
}