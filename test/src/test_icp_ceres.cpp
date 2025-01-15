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

int main() {
  /////////////////////// load data ////////////////////////

  /////////// load target data ///////////

  std::string dataset_path = "data/V1_01_easy/mav0";

  const std::string lidar_data_path = dataset_path + "/pointcloud0/data.ply";

  pcl::PLYReader Reader;

  std::cout << " loading point cloud file takes time..." << std::endl;

  if (Reader.read(lidar_data_path, *global_map) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file %f \n", lidar_data_path);
    // return (-1);
  }

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
  double rot = 10 / 180 * 3.14;
  actual_R << cos(rot), -sin(rot), 0.0, sin(rot), cos(rot), 0.0, 0.0, 0.0,
      1.0;  // Eigen::Matrix3d::Identity()

  Sophus::SE3d actual_transformation = Sophus::SE3d(actual_R, actual_t);

  // Sophus::SE3d actual_transformation =
  //    Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  transform_points(actual_transformation.inverse(), local_map_points);

  /////////// target and source point pairs ///////////
  ICPPairs icp_pairs;

  icp_pairs.clear();
  // correct matches
  for (int i = 0; i < int(local_map_points.size()); i++) {
    icp_pairs.push_back(std::make_pair(i, 1000 * i));
  }
  std::cout << " match size " << icp_pairs.size() << std::endl;

  // for debug
  std::cout << " pairs " << std::endl;
  int ttt = 0;
  for (auto& pair : icp_pairs) {
    std::cout << pair.first << " " << pair.second << std::endl;
    std::cout << global_map_points[pair.second].transpose() << std::endl;
    std::cout
        << local_map_points[pair.first].transpose()
        << std::endl
        // << global_map_points[1000 * pair.first].transpose() << std::endl
        << std::endl;
    ttt++;
    if (ttt > 20) {
      break;
    }
  }

  /////////// initial guess ///////////

  Eigen::Vector3d initial_t;
  initial_t << 0.0, 0.0, 0.0;  // Eigen::Vector3d::Zero()

  Eigen::Matrix3d initial_R;
  double rot2 = 0;  /// 180 * 3.14;
  initial_R << cos(rot2), -sin(rot2), 0.0, sin(rot2), cos(rot2), 0.0, 0.0, 0.0,
      1.0;  // Eigen::Matrix3d::Identity()

  Sophus::SE3d initial_guess = Sophus::SE3d(initial_R, initial_t);

  /////////// ICP ///////////

  clock_t begin = clock();

  estimate_pose(global_map_points, local_map_points, icp_pairs, initial_guess);

  clock_t end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "CERES SE(3) Completed in " << elapsedSecs << " seconds."
            << std::endl;

  std::cout << "actual transformation T_g_l" << std::endl
            << actual_transformation.matrix() << std::endl;

  std::cout << " final transformation T_g_l" << std::endl
            << initial_guess.matrix() << std::endl;

  std::cout << " before transform_points " << std::endl
            << local_map_points[0] << std::endl;

  transform_points(initial_guess, local_map_points);

  std::cout << " after transform_points " << std::endl
            << local_map_points[0] << std::endl;

  /*for (auto& pair : icp_pairs) {
    std::cout << " local and global point " << std::endl
              << local_map_points[pair.first].transpose() << std::endl
              << global_map_points[pair.second].transpose() << std::endl;
  }*/

  /// end ICP
}
