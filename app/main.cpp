#include <iostream>
#include "dataloader.hpp"
#include "scanRegistration.hpp"
#include "viewer.hpp"
#include <open3d/Open3D.h>

int main()
{   
    // Get data path through CMake_variable
    std::string data_root_dir = PROJECT_ROOT_DIR;
    //Read laserscan data directory in a list
    dataset::LaserScanDataset ls_dataset(data_root_dir);
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
        LaserScanRegistration::ICP(source,target,grid_size);
        source.insert(source.end(),target.begin(),target.end());
        source = LaserScanRegistration::DownSample(source,donsample_gridsize,repetition_score);
        target.clear();
        std::cout<<"\r "<<i<<std::flush;
    }
    std::cout<<std::endl;
    std::cout<<"Size of registered Point Cloud"<<std::endl;
    std::cout<<source.size()<<std::endl;
    //View final map
    viewCloud(source);
    std::cout<<"Process Completed"<<std::endl;
    return 0;
}