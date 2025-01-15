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

#include <visnav/icp_flann_utils.h>

#include <visnav/serialization.h>

using namespace visnav;

/// lidar map
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
    new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Eigen::Vector3f> global_map_points;

// flann match object pointer
FlannMatch* flann_match;

int main() {
  /////////////////////// load data ////////////////////////

  std::string dataset_path = "data/V1_01_easy/mav0";

  const std::string lidar_data_path = dataset_path + "/pointcloud0/data.ply";

  pcl::PLYReader Reader;

  std::cout << " loading point cloud file takes time..." << std::endl;

  if (Reader.read(lidar_data_path, *global_map) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %f \n", lidar_data_path);
    // return (-1);
  }

  // std::cerr << " first point " << global_map->at(0).x << " "
  //         << global_map->at(1).y << " " << global_map->at(2).z << std::endl;

  for (std::size_t i = 0; i < global_map->size(); i++) {
    global_map_points.push_back(global_map->at(i).getVector3fMap());
  }

  // flann match pointer
  flann_match = new FlannMatch();

  // build index
  flann_match->buildIndex(global_map_points);

  ////////////////////// FLANN SEARCH /////////////////////////////

  ICPPairs icp_pairs;

  std::cout << std::endl << " data is being created for the test" << std::endl;
  std::cout << " source is the subset of target" << std::endl;
  std::cout << " pairs have to be [i, 1000i]" << std::endl << std::endl;

  std::vector<Eigen::Vector3f> local_map_points;

  for (int i = 0; i < 2500; i++) {
    Eigen::Vector3f pos = global_map_points[1000 * i];
    local_map_points.push_back(pos);
  }

  clock_t begin = clock();

  /// start FLANN
  std::cout << " Searching kd_tree for " << local_map_points.size()
            << " source points" << std::endl;
  flann_match->findMatches(local_map_points, icp_pairs);

  /// end FLANN

  clock_t end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "FLANN search completed in " << elapsedSecs << " seconds."
            << std::endl;

  // for debug
  std::cout << " pairs up to 20" << std::endl;
  int ttt = 0;
  for (auto& pair : icp_pairs) {
    std::cout << std::endl
              << " local and global point " << pair.first << " " << pair.second
              << std::endl;
    std::cout << local_map_points[pair.first].transpose() << std::endl;
    std::cout << global_map_points[pair.second].transpose() << std::endl;

    ttt++;
    if (ttt > 20) {
      break;
    }
  }
}
