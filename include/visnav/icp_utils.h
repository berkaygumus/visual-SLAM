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

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <visnav/common_types.h>

namespace visnav {

struct ICPOptions {
  /// 0: silent, 1: ceres brief report (one line), 2: ceres full report
  int verbosity_level = 1;

  /// update intrinsics or keep fixed
  bool optimize_intrinsics = false;

  /// use huber robust norm or squared norm
  bool use_huber = true;

  /// parameter for huber loss (in pixel)
  double huber_parameter = 1.0;

  /// maximum number of solver iterations
  int max_num_iterations = 20;
};


void find_initial_matches(const pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                          Eigen::Matrix4f initial_guess, const double dist_max,
                          const double dist_min, pcl::Indices& map_indices,
                          pcl::Indices& keypoint_indices) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // Set the input source and target
  icp.setInputSource(keypoints);
  icp.setInputTarget(map);

  // Set the max correspondence distance to dist_max (e.g., correspondences with
  // higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(dist_max);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  // icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  // icp.setEuclideanFitnessEpsilon(1);

  // Perform the alignment
  pcl::PointCloud<pcl::PointXYZ> transformed_keypoints;
  icp.align(transformed_keypoints, initial_guess);

  // TODO: change source code and get matches (indices)
  /*
  pcl::Indices map_indices(3);
  pcl::Indices keypoint_indices(3);
  icp.align(pc_matched, initial_guess, sample_indices, corresponding_indices);
  */

  // Obtain the transformation that aligned cloud_source to
  // cloud_source_registered
  Eigen::Matrix4f transformation = icp.getFinalTransformation();

  // return matches or matches as parameter
}

void refine_matches(const pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                    const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                    const std::pair<Eigen::Vector3d, voxel_dist> voxels,
                    pcl::Indices& map_indices, pcl::Indices& keypoint_indices) {
}

void find_refined_matches(const pcl::PointCloud<pcl::PointXYZ>::Ptr map,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints,
                          const Eigen::Matrix4f initial_guess,
                          const double dist_max, const double dist_min,
                          const std::pair<Eigen::Vector3d, voxel_dist> voxels,
                          pcl::Indices& map_indices,
                          pcl::Indices& keypoint_indices) {
  // TODO: change the types of map amd keypoints:

  // from Landmarks = std::unordered_map<TrackId, Landmark> to
  // pcl::PointCloud<pcl::PointXYZ>::Ptr

  // from Cameras = std::map<FrameCamId, Camera> to
  // pcl::PointCloud<pcl::PointXYZ>::Ptr

  find_initial_matches(map, keypoints, initial_guess, dist_max, dist_min,
                       map_indices, keypoint_indices);

  refine_matches(map, keypoints, voxels, map_indices, keypoint_indices);
}

}  // namespace visnav