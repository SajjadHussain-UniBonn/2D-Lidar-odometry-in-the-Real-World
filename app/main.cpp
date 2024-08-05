#include <iostream>
#include "dataloader.hpp"
#include "viewer.hpp"
#include <open3d/Open3D.h>
using namespace dataset;

int main()
{
    // const std::string filename = "/home/hussain/Work/CPP_Projects/Ex5/data/src.ply";
    // const std::string filename1 = "/home/hussain/Work/CPP_Projects/Ex5/data/target.ply";

    // open3d::geometry::PointCloud pcd;
    // open3d::geometry::PointCloud pcd1;

    // open3d::io::ReadPointCloud(filename, pcd);
    // open3d::io::ReadPointCloud(filename1, pcd1);

    // std::vector<Eigen::Vector3d> points;
    // std::vector<Eigen::Vector3d> points1;

    // points = pcd.points_;
    // points1 = pcd1.points_;

    // std::vector<Eigen::Vector2d> src;
    // std::vector<Eigen::Vector2d> target;
    // // convert 3D points to 2D points
    // std::transform(points.begin(), points.end(), std::back_inserter(src), [](const auto &p)
    //         { return Eigen::Vector2d(p.x(), p.y()); });
    // std::transform(points1.begin(), points1.end(), std::back_inserter(target), [](const auto &p)
    //             { return Eigen::Vector2d(p.x(), p.y()); });

    // std::cout << "Read points from both pointclouds successfully!" << std::endl;
    // // viewCloud(target);
    // // viewCloud(src);
    // // viewCloud(src);
    // Get data path through CMake_variable
    std::string data_root_dir = PROJECT_ROOT_DIR;
    //Read laserscan data directory in a list
    LaserScanDataset ls_dataset(data_root_dir);
    std::vector<Eigen::Vector2d> target = ls_dataset[0];
    std::vector<Eigen::Vector2d> source;
    std::vector<Eigen::Vector2d> transformed_PC;
    std::vector<Eigen::Vector2d> registered_PCs =target;
    Eigen::Matrix3d transformation_matrix;
    const double grid_size = 0.1;
    int total_scans = 100;
    std::cout<<"Progress of Registering Clouds"<<std::endl;
    for(int i=1;i<=total_scans;i++)
    {
        source = ls_dataset[i];
        source = DownSample(source,0.1,1);
        transformation_matrix = ICP(source,target,grid_size);
        transformed_PC = ApplyTransformation(source,transformation_matrix);
        // registered_PCs = ConcatenatePointClouds(registered_PCs,transformed_PC);
        // registered_PCs.insert(registered_PCs.end(),transformed_PC.begin(),transformed_PC.end());
        target.insert(target.end(),transformed_PC.begin(),transformed_PC.end());
        target = DownSample(target,0.1,1);
        std::cout<<"\r "<<i<<std::flush;
        source.clear();
    }
    std::cout<<std::endl;
    // std::cout<<registered_PCs.size()<<std::endl;
    std::cout<<target.size()<<std::endl;
    // viewCloud(registered_PCs);
    viewCloud(target);
    std::cout<<"Process Completed"<<std::endl;
    return 0;
}