
/*
@param keypoints from local map
@param keyframes from local map
@param Tc current keyframe
@param matches (of keypoints and 3D points)
@param opt_parameters
@return local_map (keypoints and keyframe poses) TODO:type? pair? two vectors?
*/

// takes matches between keypoints from local and 3d points from known map

// Transform correspondence ck’ -> ck’’
// Tc: from map to current keyframe

// Optimization sim(3)  with ceres
// Create loss function, Huber cost function
// Create problem with data
// Optimization options

// Update local map (keypoints and keyframe poses) with optimum sim(3)

#pragma once

#include <Eigen/Dense>

#include <pcl/point_types.h>

#include <visnav/common_types.h>
#include <visnav/icp_utils.h>

namespace visnav {

void transform_points(
    const std::vector<Eigen::Vector3f> global_map_points,
    const std::vector<Eigen::Vector3f> local_map_points,
    std::vector<Eigen::Vector3f>& global_map_points_wrt_keyframe,
    std::vector<Eigen::Vector3f>& local_map_points_wrt_keyframe,
    const Sophus::SE3d T_c_g, const ICPPairs icp_pairs) {
  global_map_points_wrt_keyframe.clear();
  local_map_points_wrt_keyframe.clear();
  for (auto& pair : icp_pairs) {
    local_map_points_wrt_keyframe.push_back(local_map_points[pair.first]);
    global_map_points_wrt_keyframe.push_back(global_map_points[pair.second]);
  }

  transform_points(T_c_g, local_map_points_wrt_keyframe);
  transform_points(T_c_g, global_map_points_wrt_keyframe);
}

void sim3_optimize(
    const std::vector<Eigen::Vector3f> global_map_points_wrt_keyframe,
    std::vector<Eigen::Vector3f>& local_map_points_wrt_keyframe,
    Sophus::SE3d& S_g_l) {
  // TODO: add opt parameters
  ICPPairs icp_pairs;
  for (int i = 0; i < int(local_map_points_wrt_keyframe.size()); i++) {
    icp_pairs.push_back(std::make_pair(i, i));
  }
  estimate_pose(global_map_points_wrt_keyframe, local_map_points_wrt_keyframe,
                icp_pairs, S_g_l);
}

}  // namespace visnav