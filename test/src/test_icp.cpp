#include <algorithm>
#include <atomic>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

#include <sophus/se3.hpp>

#include <tbb/concurrent_unordered_map.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <CLI/CLI.hpp>

#include <visnav/common_types.h>

#include <visnav/icp_utils.h>
#include <visnav/opt_sim3_utils.h>

#include <visnav/serialization.h>

using namespace visnav;

/// lidar map
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
    new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Eigen::Vector3f> global_map_points;

// flann match object pointer
FlannMatch* flann_match;

int main() {
  /////////// load target data ///////////

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

  /////////// create source data ///////////

  // subset of target data
  std::vector<Eigen::Vector3f> local_map_points;

  for (int i = 0; i < 2500; i++) {
    Eigen::Vector3f pos = global_map_points[1000 * i];
    local_map_points.push_back(pos);
  }

  // transform the source data
  Eigen::Vector3d actual_t;
  actual_t << 0.0, 1.0, 1.0;  // Eigen::Vector3d::Zero()

  Eigen::Matrix3d actual_R;
  double rot = 0;  /// 180 * 3.14;
  actual_R << cos(rot), -sin(rot), 0.0, sin(rot), cos(rot), 0.0, 0.0, 0.0,
      1.0;  // Eigen::Matrix3d::Identity()

  Sophus::SE3d actual_transformation = Sophus::SE3d(actual_R, actual_t);

  // Sophus::SE3d actual_transformation =
  //    Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  transform_points(actual_transformation, local_map_points);

  /////////// FLANN for the nearest point search ///////////
  // flann match pointer
  flann_match = new FlannMatch();

  // build index
  flann_match->buildIndex(global_map_points);

  /////////// ICP options ///////////

  ICPOptions icp_options;
  // TODO: define icp_options

  ICPPairs icp_pairs;

  /////////// initial guess ///////////

  Eigen::Vector3d initial_t;
  initial_t << 0.0, 1.0, 1.0;  // Eigen::Vector3d::Zero()

  Eigen::Matrix3d initial_R;
  double rot2 = 0;  /// 180 * 3.14;
  initial_R << cos(rot2), -sin(rot2), 0.0, sin(rot2), cos(rot2), 0.0, 0.0, 0.0,
      1.0;  // Eigen::Matrix3d::Identity()

  // Sophus::SE3d guess = Sophus::SE3d(initial_R, initial_t);
  Sophus::SE3d guess =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  /////////// ICP SETUP ///////////

  clock_t begin = clock();

  find_initial_matches(global_map_points, local_map_points, guess, icp_options,
                       flann_match, icp_pairs);

  clock_t end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "ICP Completed in " << elapsedSecs << " seconds." << std::endl;
}
