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

#include <visnav/icp_ceres_utils.h>
#include <visnav/icp_flann_utils.h>
#include <visnav/voxel_utils.h>

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
  Sophus::SE3d guess;
  // initial guess
  // threshol error
};

Sophus::SE3d get_initial_guess() {
  //// BODY POSE1 WRT WORLD FROM GROUND TRUTH FILE
  // Sensor extrinsics wrt. the body-frame. This is the transformation of the
  // tracking prima to the body frame.
  // p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [],
  // q_RS_z [] 0.878612,2.142470,0.947262,0.060514,-0.828459,-0.058956,-0.553641

  Eigen::Quaterniond q_body1_w;
  q_body1_w.x() = -0.828459;
  q_body1_w.y() = -0.058956;
  q_body1_w.z() = -0.553641;
  q_body1_w.w() = 0.060514;

  Eigen::Vector3d t_body1_w;
  t_body1_w << 0.878612, 2.142470, 0.947262;

  Eigen::Matrix3d R_body1_w = q_body1_w.normalized().toRotationMatrix();

  Sophus::SE3d T_body1_w = Sophus::SE3d(R_body1_w, t_body1_w);

  ///// CAM POSE WRT BODY1 FROM GROUND TRUTH FILE
  // cam0 pose wrt body
  // Sensor extrinsics wrt. the body-frame.
  /*
  [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
         0.0, 0.0, 0.0, 1.0]
  */

  ///
  Eigen::Vector3d t_body_cam;
  t_body_cam << -0.0216401454975, -0.064676986768, 0.00981073058949;

  Eigen::Matrix3d R_body_cam;
  R_body_cam << 0.0148655429818, -0.999880929698, 0.00414029679422,
      0.999557249008, 0.0149672133247, 0.025715529948, -0.0257744366974,
      0.00375618835797, 0.999660727178;

  Sophus::SE3d T_body_cam = Sophus::SE3d(R_body_cam, t_body_cam);

  ///// CAM1 POSE WRT WORLD
  // cam1 frame is local map frame

  // Sophus::SE3d T_w_cam1 = T_body1_w.inverse() * T_body_cam;
  Sophus::SE3d T_w_cam1 = T_body1_w * T_body_cam;

  std::cout << " ground truth transformation Sophus::SE3d" << std::endl
            << T_w_cam1.matrix() << std::endl;

  Eigen::Vector3d t_w_cam1 =
      R_body1_w.inverse() * t_body_cam - R_body1_w.inverse() * t_body1_w;
  Eigen::Matrix3d R_w_cam1 = R_body1_w.inverse() * R_body_cam;

  std::cout << " ground truth transformation Eigen" << std::endl
            << R_w_cam1 << std::endl
            << t_w_cam1 << std::endl;

  /// initial guess
  // Sophus::SE3d final_result = T_body1_w.inverse() * T_body_cam;
  return T_w_cam1;
}

void find_initial_matches(const std::vector<Eigen::Vector3f> global_map_points,
                          std::vector<Eigen::Vector3f>& local_map_points,
                          ICPOptions& icp_options, FlannMatch* flann_match,
                          ICPPairs& icp_pairs) {
  // TODO: find matches using icp ( flann + ceres)
  // find_closest_points_brute_force(map, landmarks, icp_pairs);

  // icp_options
  Sophus::SE3d& guess = icp_options.guess;

  /////// START ICP ////////

  Sophus::SE3d incremental_result =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  Sophus::SE3d final_result =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  // for debug
  // std::cout << " before transform_points " << std::endl
  //          << local_map_points[0] << std::endl;

  transform_points(guess, local_map_points);

  // std::cout << "initial_guess " << std::endl << guess.matrix() << std::endl;

  // std::cout << " after transform_points " << std::endl
  //          << local_map_points[0] << std::endl;

  int itr_num = icp_options.max_itr;
  for (int i = 0; i < itr_num; i++) {
    std::cout << " ///////////// itreation " << i << " ////////////////////"
              << std::endl;
    // for debug
    // std::cout << "first transformation " << std::endl
    //          << final_result.matrix() << std::endl;

    flann_match->findMatches(local_map_points, icp_pairs);
    // icp_pairs.clear();
    // for (int i = 0; i < 100; i++) {
    //  icp_pairs.push_back(std::make_pair(i, i));
    //}
    std::cout << " match size " << icp_pairs.size() << std::endl;

    // for debug

    std::cout << " pairs local-global" << std::endl;
    int ttt = 0;
    for (auto& pair : icp_pairs) {
      std::cout << pair.first << " " << pair.second << std::endl;
      std::cout << local_map_points[pair.first].transpose() << std::endl;
      std::cout << global_map_points[pair.second].transpose() << std::endl
                << std::endl;
      ttt++;
      if (ttt > 5) {
        break;
      }
    }

    estimate_pose(global_map_points, local_map_points, icp_pairs,
                  incremental_result);
    // std::cout << " incremental transformation " << std::endl
    //          << incremental_result.matrix() << std::endl;

    transform_points(incremental_result, local_map_points);

    // std::cout << " before point " << std::endl << local_map_points[0] <<
    // std::endl;
    final_result = incremental_result * final_result;
    // for debug
    // std::cout << " transformation " << std::endl
    //          << final_result.matrix() << std::endl;
    std::cout << " transformation " << std::endl
              << final_result.translation().transpose() << std::endl;

    // std::cout << " after point " << std::endl << local_map_points[0] <<
    // std::endl;
  }

  std::cout << "initial_guess " << std::endl << guess.matrix() << std::endl;

  std::cout << " transformation after initial guess " << std::endl
            << final_result.matrix() << std::endl;

  guess = final_result * guess;

  std::cout << " final transformation " << std::endl
            << guess.matrix() << std::endl;

  /*for (auto& pair : icp_pairs) {
    std::cout << " local and global point " << std::endl
              << local_map_points[pair.first].transpose() << std::endl
              << global_map_points[pair.second].transpose() << std::endl;
  }*/
}

/*
void find_initial_matches(const std::vector<Eigen::Vector3f> global_map_points,
                          const Landmarks landmarks, Sophus::SE3d& guess,
                          const ICPOptions icp_options,
                          const FlannMatch* flann_match, ICPPairs& icp_pairs) {
  // create local_map_points using landmarks
  std::vector<Eigen::Vector3f> local_map_points;
  get_local_map_points(landmarks, local_map_points);

  // call find_initial_matches for local_map_points
  find_initial_matches(global_map_points, local_map_points, guess, icp_options,
                       flann_match, icp_pairs);
}
*/

// TODO: const Voxel gives error why?
bool check_pair(const Eigen::Vector3f local_map_point, Voxel voxel) {
  int Nmin = 10;
  int Nsigma = 3;
  if (voxel.N < Nmin) {
    std::cerr << " not enough point " << std::endl;
    return false;
  }

  Eigen::Vector3f local_map_point_transformed = voxel.T * local_map_point;
  std::cout << " local_map_point_transformed " << std::endl
            << local_map_point_transformed.transpose() << std::endl;
  std::cout << " 3 *sigma " << std::endl
            << Nsigma * voxel.sigma.transpose() << std::endl;

  if (abs(local_map_point_transformed[0]) > Nsigma * voxel.sigma[0]) {
    std::cerr << " first axis " << std::endl;
    return false;
  }
  if (abs(local_map_point_transformed[1]) > Nsigma * voxel.sigma[1]) {
    std::cerr << " second axis " << std::endl;
    return false;
  }
  if (abs(local_map_point_transformed[2]) > Nsigma * voxel.sigma[2]) {
    std::cerr << " third axis " << std::endl;
    return false;
  }

  std::cout << " OK " << std::endl << std::endl;
  return true;
}

// TODO: const Voxels gives error why?
void refine_matches(const std::vector<Eigen::Vector3f> global_map_points,
                    const std::vector<Eigen::Vector3f> local_map_points,
                    Voxels voxels, ICPPairs& icp_pairs) {
  // TODO: refine matches acc. to voxels
  Eigen::Vector3f voxel_center;
  double resolution = 1.0;  // TODO: ADD parameter
  int pair_size = icp_pairs.size();
  std::cout << " pair size " << pair_size << std::endl;

  ICPPairs refined_icp_pairs;
  for (auto& pair : icp_pairs) {
    find_voxel_center(local_map_points[pair.first], resolution, voxel_center);
    std::cout << " local point " << std::endl
              << local_map_points[pair.first].transpose() << std::endl;
    std::cout << " global point " << std::endl
              << global_map_points[pair.second].transpose() << std::endl;
    std::cout << " voxel center " << std::endl
              << voxel_center.transpose() << std::endl;

    if (!check_pair(local_map_points[pair.first], voxels[voxel_center])) {
      pair.second = -1;
      pair_size--;
    } else {
      refined_icp_pairs.push_back(pair);
    }
  }

  icp_pairs = refined_icp_pairs;

  std::cout << "refined pair size " << pair_size << " " << icp_pairs.size()
            << std::endl;
}

void find_refined_matches(const std::vector<Eigen::Vector3f> global_map_points,
                          std::vector<Eigen::Vector3f>& local_map_points,
                          const Voxels voxels, ICPOptions& icp_options,
                          FlannMatch* flann_match, ICPPairs& icp_pairs) {
  // there are around 2500 landmarks for max_num_kfs = 20 frames
  // bundle adjustment takes 1.0-1.5 seconds
  // one brute force mathing takes 30-35 seconds
  // one flann matching takes 0.25-0.45 seconds

  // finds matches
  // gets final ICP guess
  // updates local_map_points
  find_initial_matches(global_map_points, local_map_points, icp_options,
                       flann_match, icp_pairs);

  refine_matches(global_map_points, local_map_points, voxels, icp_pairs);
}

}  // namespace visnav