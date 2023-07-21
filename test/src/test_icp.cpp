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
#include <pcl/common/transforms.h>

#include <CLI/CLI.hpp>

#include <visnav/common_types.h>

#include <visnav/icp_utils.h>
#include <visnav/opt_sim3_utils.h>

#include <visnav/serialization.h>

using namespace visnav;

/// target (lidar map)
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map(
    new pcl::PointCloud<pcl::PointXYZ>);
std::vector<Eigen::Vector3f> global_map_points;

// source (local map)
pcl::PointCloud<pcl::PointXYZ>::Ptr local_map(
    new pcl::PointCloud<pcl::PointXYZ>);

// flann match object pointer
FlannMatch* flann_match;

int main() {
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
    local_map->push_back(global_map->at(1000 * i));
  }

  // transform the source data
  Eigen::Vector3d actual_t;
  actual_t << 0.0, 3.0, 3.0;  // Eigen::Vector3d::Zero()

  Eigen::Quaterniond q_R;
  q_R.x() = 0.0;
  q_R.y() = 0.0;
  q_R.z() = 0.1736482;
  q_R.w() = 0.9848078;

  Eigen::Matrix3d actual_R = q_R.normalized().toRotationMatrix();
  /*
  Eigen::Matrix3d actual_R;
  double rot = 10 / 180 * 3.14;
  actual_R << 0.9396926, -0.3420202,  0.0000000,
   0.3420202,  0.9396926,  0.0000000,
   0.0000000,  0.0000000,  1.0000000;   // Eigen::Matrix3d::Identity()
*/
  Sophus::SE3d actual_transformation = Sophus::SE3d(actual_R, actual_t);

  // Sophus::SE3d actual_transformation =
  //    Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  transform_points(actual_transformation, local_map_points);

  Eigen::Affine3d eigen_transform = Eigen::Affine3d::Identity();

  // Define a translation of 2.5 meters on the x axis.
  eigen_transform.translation() = actual_t;

  // The same rotation matrix as before; theta radians around Z axis
  eigen_transform.rotate(actual_R);

  pcl::transformPointCloud(*local_map, *local_map, eigen_transform);

  // check data
  std::cout << " global map " << std::endl;
  std::cout << global_map->at(10).getVector3fMap().transpose() << std::endl;
  std::cout << global_map_points[10].transpose() << std::endl;

  std::cout << " local map " << std::endl;
  std::cout << local_map->at(10).getVector3fMap().transpose() << std::endl;
  std::cout << local_map_points[10].transpose() << std::endl;

  /////////// FLANN for the nearest point search ///////////
  // flann match pointer
  flann_match = new FlannMatch();

  // build index
  flann_match->buildIndex(global_map_points);

  /////////// initial guess ///////////

  Eigen::Vector3d initial_t;
  initial_t << 0.0, 0.0, 0.0;  // Eigen::Vector3d::Zero()

  Eigen::Matrix3d initial_R;
  double rot2 = 0;  /// 180 * 3.14;
  initial_R << cos(rot2), -sin(rot2), 0.0, sin(rot2), cos(rot2), 0.0, 0.0, 0.0,
      1.0;  // Eigen::Matrix3d::Identity()

  ICPOptions icp_options;
  icp_options.max_itr = 500;
  // icp_options.guess = Sophus::SE3d(initial_R, initial_t);
  icp_options.guess =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  /////////// ICP  ///////////
  ICPPairs icp_pairs;

  clock_t begin = clock();

  find_initial_matches(global_map_points, local_map_points, icp_options,
                       flann_match, icp_pairs);

  clock_t end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;

  std::cout << "ICP Completed in " << elapsedSecs << " seconds." << std::endl;

  /////////// PCL ICP  ///////////
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  std::cout << "setInputSource " << std::endl;
  icp.setInputSource(local_map);
  std::cout << "setInputTarget " << std::endl;
  icp.setInputTarget(global_map);
  std::cout << "okokokokok " << std::endl;

  pcl::PointCloud<pcl::PointXYZ> Final;
  Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
  icp.align(Final, guess);

  // pcl::PointCloud<pcl::PointXYZ> Final;
  // icp.align(Final);

  std::cout << std::endl << " RESULTS " << std::endl;

  std::cout << "actual eigen transformation " << std::endl
            << actual_transformation.inverse().matrix() << std::endl;

  std::cout << "actual pcl transformation " << std::endl
            << eigen_transform.inverse().matrix() << std::endl;

  std::cout << "custom icp guess transformation " << std::endl
            << icp_options.guess.matrix() << std::endl;

  std::cout << " PCL guess" << std::endl
            << icp.getFinalTransformation() << std::endl;
  std::cout << " PCL guess inverse" << std::endl
            << icp.getFinalTransformation().inverse() << std::endl;

  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
}
