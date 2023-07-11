// takes keypoints from local and 3d points from known map

// icp function to get matches (custom or builtin library)

// Correspondence refinement ck -> ck’ using distribution
// The objective is to filter out correspondences associating keypoints that are
// located in areas not covered by or on the boundaries of the map

/*
@param keypoints
@param 3Dpoints
@return matches TODO:type? pair? two vectors?
*/

// Firstly, use anp ICP library

// TODO : KD TREE of known map?
// For each reconstructed point, we find its nearest neighbor in the map,
// which is stored in a kd-tree to allow for fast look-up. If the
// nearest neighbor is close enough, the pair is added to the
// correspondence set

// TODO : dynamic threshold
// reduce the distance threshold τk over the iterations k.
// τmax and τmin are parameters of our method and define the
// linear threshold function

#pragma once

#include <Eigen/Dense>

#include <ceres/ceres.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <flann/flann.hpp>

#include <visnav/common_types.h>

using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

namespace visnav {

struct ICPOptions {
  double min_dist;
  double max_dist;
  int max_itr;
  // initial guess
  // threshol error
};

void find_closest_points_brute_force(const std::vector<Eigen::Vector3f> map,
                                     const Landmarks landmarks,
                                     ICPPairs& icp_pairs) {
  // TODO: find closest point on the global map using  brute force
  icp_pairs.clear();
  for (auto landmark : landmarks) {
    Eigen::Vector3d pos = landmark.second.p;
    int closest_index = -1;
    double closest_dist = 2;
    double dist;
    for (int i = 0; i < map.size(); i++) {
      dist = (pos - map[i].cast<double>()).norm();
      // Vector3f{ vertex.position.x(), vertex.position.y(), vertex.position.z()
      // }
      if (dist < closest_dist) {
        closest_index = i;
        closest_dist = dist;
      }
    }

    if (closest_index != -1) {
      icp_pairs.push_back(std::make_pair(closest_index, landmark.first));
    }
  }
}

void find_initial_matches(const std::vector<Eigen::Vector3f> map,
                          const Landmarks landmarks,
                          const ICPOptions icp_options, ICPPairs& icp_pairs) {
  // TODO: find matches using icp ( flann + ceres)
  find_closest_points_brute_force(map, landmarks, icp_pairs);
}

void refine_matches(const std::vector<Eigen::Vector3f> map,
                    const Landmarks landmarks, const Voxels voxels,
                    ICPPairs& icp_pairs) {
  // TODO: refine matches acc. to voxels
}

void find_refined_matches(const std::vector<Eigen::Vector3f> map,
                          const Landmarks landmarks, const Voxels voxels,
                          const ICPOptions icp_options, ICPPairs& icp_pairs) {
  // there are around 2500 landmarks for max_num_kfs = 20 frames
  // bundle adjustment takes 1.0-1.5 seconds
  // one brute force mathing takes 30-35 seconds
  find_initial_matches(map, landmarks, icp_options, icp_pairs);

  refine_matches(map, landmarks, voxels, icp_pairs);
}

}  // namespace visnav