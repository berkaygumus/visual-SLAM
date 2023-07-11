
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

namespace visnav {

void sim3_optimize(const pcl::Indices map_indices,
                   const pcl::Indices keypoint_indices, Cameras& cameras_opt,
                   Landmarks& landmarks_opt) {
  // TODO: add opt parameters
  // add Tc
}

}  // namespace visnav