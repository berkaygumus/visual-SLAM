
#pragma once

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

#include <visnav/voxel_utils.h>
#include <visnav/icp_utils.h>
#include <visnav/opt_sim3_utils.h>

#include <visnav/gui_helper.h>
#include <visnav/tracks.h>

#include <visnav/serialization.h>

namespace visnav {

void get_local_map_points(const Landmarks landmarks,
                          std::vector<Eigen::Vector3f>& local_map_points) {
  local_map_points.clear();
  for (auto landmark : landmarks) {
    Eigen::Vector3f pos = landmark.second.p.cast<float>();
    local_map_points.push_back(pos);
  }
}

void lidar_map_adjustment(const std::vector<Eigen::Vector3f> global_map_points,
                          Cameras& cameras, const FrameCamId current_keyframe,
                          Landmarks& landmarks, const Voxels voxels,
                          ICPOptions& icp_options, FlannMatch* flann_match) {
  // create local_map_points using landmarks
  std::vector<Eigen::Vector3f> local_map_points;
  get_local_map_points(landmarks, local_map_points);

  // finds matches and refines acc. to voxel distribution : icp_pairs
  // gets final ICP guess : icp_options.guess
  // transforms local_map_points into global map with ICP guess
  // p_globalmap = T_g_l * p_localmap
  Eigen::Vector3f p_localmap = local_map_points[0];
  ICPPairs icp_pairs;
  find_refined_matches(global_map_points, local_map_points, voxels, icp_options,
                       flann_match, icp_pairs);
  Sophus::SE3d T_g_l = icp_options.guess;

  std::cout << " p_localmap " << p_localmap << std::endl;
  std::cout << " p_globalmap " << local_map_points[0] << std::endl;
  std::cout << " T_g_l " << std::endl << T_g_l.matrix() << std::endl;

  /// camera pose (transforms from camera to local map)
  Sophus::SE3d T_l_c = cameras.at(current_keyframe).T_w_c;
  std::cout << " T_l_c " << std::endl << T_l_c.matrix() << std::endl;

  // transformation from global map to current camera frame
  Sophus::SE3d T_c_g = T_l_c.inverse() * T_g_l.inverse();

  // transform map points into camera frame
  std::vector<Eigen::Vector3f> global_map_points_wrt_keyframe;
  std::vector<Eigen::Vector3f> local_map_points_wrt_keyframe;

  transform_points(global_map_points, local_map_points,
                   global_map_points_wrt_keyframe,
                   local_map_points_wrt_keyframe, T_c_g, icp_pairs);

  // sim3 optimization
  Sophus::SE3d S_g_l =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  sim3_optimize(global_map_points_wrt_keyframe, local_map_points_wrt_keyframe,
                S_g_l);

  Sophus::SE3d T_g_l_final = (T_c_g.inverse() * S_g_l * T_c_g) * T_g_l;

  std::cout << " S_g_l " << std::endl << S_g_l.matrix() << std::endl;
  std::cout << " T_g_l_final " << std::endl << T_g_l_final.matrix() << std::endl;
  std::cout << " T_g_l_final inverse " << std::endl << T_g_l_final.inverse().matrix() << std::endl;
}

}  // namespace visnav