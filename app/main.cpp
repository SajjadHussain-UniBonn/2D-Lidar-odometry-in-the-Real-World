#include "dataloader.hpp"
#include "viewer.hpp"
using namespace dataset;
int main()
{
    std::string data_root_dir = PROJECT_ROOT_DIR;
    LaserScanDataset ls_dataset(data_root_dir);
    std::vector<Eigen::Vector2d> pointcloud_points = ls_dataset[1];
    viewCloud(pointcloud_points);
    return 0;
}