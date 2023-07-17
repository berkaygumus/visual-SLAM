
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

void lidar_map_adjustment(const std::vector<Eigen::Vector3f> global_map_points,
                          Cameras& cameras, Landmarks& landmarks,
                          const Voxels voxels, ICPOptions& icp_options,
                          FlannMatch* flann_match) {
  // create local_map_points using landmarks
  std::vector<Eigen::Vector3f> local_map_points;
  get_local_map_points(landmarks, local_map_points);

  // finds matches and refines acc. to voxel distribution : icp_pairs
  // gets final ICP guess : icp_options.guess
  // transforms local_map_points into global map with ICP guess
  ICPPairs icp_pairs;
  find_refined_matches(global_map_points, local_map_points, voxels, icp_options,
                       flann_match, icp_pairs);

  UNUSED(cameras);
}

}  // namespace visnav