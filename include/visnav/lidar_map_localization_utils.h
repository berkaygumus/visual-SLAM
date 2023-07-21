
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

void update_local_map(const Sophus::SE3d T_correction, Cameras& cameras,
                      Landmarks& landmarks) {
  for (auto& camera : cameras) {
    camera.second.T_w_c = T_correction * camera.second.T_w_c;
  }

  for (auto& landmark : landmarks) {
    landmark.second.p = T_correction * landmark.second.p;
  }
}

void lidar_map_adjustment(const std::vector<Eigen::Vector3f> global_map_points,
                          Cameras& cameras, const FrameCamId current_keyframe,
                          Landmarks& landmarks, const Voxels voxels,
                          ICPOptions& icp_options, FlannMatch* flann_match) {
  // time
  clock_t begin, end;

  // create local_map_points using landmarks

  std::vector<Eigen::Vector3f> local_map_points;
  begin = clock();
  get_local_map_points(landmarks, local_map_points);
  end = clock();
  double elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "get_local_map_points completed in " << elapsedSecs
            << " seconds." << std::endl;

  // finds matches and refines acc. to voxel distribution : icp_pairs
  // gets final ICP guess : icp_options.guess
  // transforms local_map_points into global map with ICP guess
  // p_globalmap = T_g_l * p_localmap

  ICPPairs icp_pairs;
  Sophus::SE3d initial_guess = icp_options.guess;
  begin = clock();
  find_refined_matches(global_map_points, local_map_points, voxels, icp_options,
                       flann_match, icp_pairs);
  end = clock();
  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "find_refined_matches completed in " << elapsedSecs
            << " seconds." << std::endl;
  Sophus::SE3d T_g_l = icp_options.guess;

  /// camera pose (transforms from camera to local map)
  Sophus::SE3d T_l_c = cameras.at(current_keyframe).T_w_c;
  std::cout << " keyframe " << current_keyframe.frame_id;
  std::cout << " T_l_c " << std::endl << T_l_c.matrix() << std::endl;

  // transformation from global map to current camera frame
  Sophus::SE3d T_c_g = T_l_c.inverse() * T_g_l.inverse();

  // transform map points into camera frame
  std::vector<Eigen::Vector3f> global_map_points_wrt_keyframe;
  std::vector<Eigen::Vector3f> local_map_points_wrt_keyframe;

  begin = clock();
  transform_points(global_map_points, local_map_points,
                   global_map_points_wrt_keyframe,
                   local_map_points_wrt_keyframe, T_c_g, icp_pairs);
  end = clock();
  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "transform_points completed in " << elapsedSecs << " seconds."
            << std::endl;

  // sim3 optimization
  Sophus::SE3d S_g_l =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  begin = clock();
  sim3_optimize(global_map_points_wrt_keyframe, local_map_points_wrt_keyframe,
                S_g_l);
  end = clock();
  elapsedSecs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "sim3_optimize completed in " << elapsedSecs << " seconds."
            << std::endl;

  Sophus::SE3d T_g_l_final = (T_c_g.inverse() * S_g_l * T_c_g) * T_g_l;
  Sophus::SE3d correction =
      T_g_l.inverse() * (T_c_g.inverse() * S_g_l * T_c_g) * T_g_l;

  update_local_map(correction, cameras, landmarks);

  Sophus::SE3d T_g_l_ground = get_initial_guess();

  std::cout << " initial_guess " << std::endl
            << initial_guess.matrix() << std::endl;

  std::cout << " T_g_l_ground ???" << std::endl
            << T_g_l_ground.matrix() << std::endl;

  std::cout << " T_g_l_ground inverse ???" << std::endl
            << T_g_l_ground.inverse().matrix() << std::endl;

  std::cout << " T_g_l from ICP" << std::endl << T_g_l.matrix() << std::endl;

  std::cout << " S_g_l " << std::endl << S_g_l.matrix() << std::endl;
  std::cout << " T_g_l_final " << std::endl
            << T_g_l_final.matrix() << std::endl;

  std::cout << " correction after ICP " << std::endl
            << correction.matrix() << std::endl;

  // TODO: update all cameras and landmarks with T_g_l_final
}

}  // namespace visnav