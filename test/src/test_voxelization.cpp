#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include <tbb/concurrent_unordered_map.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <CLI/CLI.hpp>

#include <visnav/common_types.h>

#include <visnav/voxel_utils.h>

#include <visnav/serialization.h>

using namespace visnav;

/// lidar map
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
    new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Eigen::Vector3f> global_map_points;

/// voxel distribution
Voxels voxels;

int main() {
  /////////////////////// load data ////////////////////////

  std::string dataset_path = "data/V1_01_easy/mav0";

  const std::string lidar_data_path = dataset_path + "/pointcloud0/data.ply";

  pcl::PLYReader Reader;

  if (Reader.read(lidar_data_path, *global_map) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %f \n", lidar_data_path);
    // return (-1);
  }

  std::cerr << " first point " << global_map->at(0).x << " "
            << global_map->at(1).y << " " << global_map->at(2).z << std::endl;

  for (std::size_t i = 0; i < global_map->size(); i++) {
    global_map_points.push_back(global_map->at(i).getVector3fMap());
  }

  ////////////////////// VOXELIZATION /////////////////////////////

  std::vector<Eigen::Vector3f> local_map_points;

  for (int i = 0; i < 2500; i++) {
    Eigen::Vector3f pos = global_map_points[1000 * i];
    local_map_points.push_back(pos);
  }

  double resolution = 1;  // 1 meter

  clock_t begin = clock();

  calculate_voxel_distribution(global_map_points, resolution, voxels);

  clock_t end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "VOXELIZATION completed in " << elapsedSecs << " seconds."
            << std::endl;

  // for debug
}
