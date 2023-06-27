/**
BSD 3-Clause License

Copyright (c) 2018, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <set>

#include <visnav/common_types.h>

#include <visnav/calibration.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

namespace visnav {

void project_landmarks(
    const Sophus::SE3d& current_pose,
    const std::shared_ptr<AbstractCamera<double>>& cam,
    const Landmarks& landmarks, const double cam_z_threshold,
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    std::vector<TrackId>& projected_track_ids) {
  projected_points.clear();
  projected_track_ids.clear();

  // TODO SHEET 5: project landmarks to the image plane using the current
  // locations of the cameras. Put 2d coordinates of the projected points into
  // projected_points and the corresponding id of the landmark into
  // projected_track_ids.

  for (auto& landmark : landmarks) {
    Eigen::Vector3d p_3d_cam = current_pose.inverse() * landmark.second.p;
    // std::cout << p_3d_cam[2] << " z threshold " << cam_z_threshold <<
    // std::endl;

    // behind the cam
    if (p_3d_cam[2] < cam_z_threshold) {
      // std::cout << " continue " << std::endl;
      continue;
    }

    Eigen::Vector2d p_2d_cam = cam->project(p_3d_cam);

    // outside the image
    if (p_2d_cam[0] < 0 || p_2d_cam[0] >= cam->width()) {
      // std::cout << p_2d_cam[0] << " width " << cam->width() << std::endl;
      continue;
    }

    // outside the image
    if (p_2d_cam[1] < 0 || p_2d_cam[1] >= cam->height()) {
      // std::cout << p_2d_cam[1] << " height " << cam->height() << std::endl;
      continue;
    }

    projected_points.push_back(p_2d_cam);
    projected_track_ids.push_back(landmark.first);
  }
}

void find_matches_landmarks(
    const KeypointsData& kdl, const Landmarks& landmarks,
    const Corners& feature_corners,
    const std::vector<Eigen::Vector2d,
                      Eigen::aligned_allocator<Eigen::Vector2d>>&
        projected_points,
    const std::vector<TrackId>& projected_track_ids,
    const double match_max_dist_2d, const int feature_match_threshold,
    const double feature_match_dist_2_best, LandmarkMatchData& md) {
  md.matches.clear();

  // TODO SHEET 5: Find the matches between projected landmarks and detected
  // keypoints in the current frame. For every detected keypoint search for
  // matches inside a circle with radius match_max_dist_2d around the point
  // location. For every landmark the distance is the minimal distance between
  // the descriptor of the current point and descriptors of all observations of
  // the landmarks. The feature_match_threshold and feature_match_dist_2_best
  // should be used to filter outliers the same way as in exercise 3. You should
  // fill md.matches with <featureId,trackId> pairs for the successful matches
  // that pass all tests.

  for (int key_point_index = 0; key_point_index < int(kdl.corners.size());
       key_point_index++) {
    // key_point_index the index of keypoint;
    // kdl.corner_descriptors[key_point_index]
    // kdl.corners[key_point_index]
    std::bitset<256> keypoint_descriptor =
        kdl.corner_descriptors[key_point_index];

    size_t min_dist1 = 256;
    size_t min_dist2 = 256;
    int best_match = -1;

    for (int projected_point_index = 0;
         projected_point_index < int(projected_points.size());
         projected_point_index++) {
      // projected_point_index
      // projected_points[projected_point_index] : 2d point
      // projected_track_ids[projected_point_index] : track id

      // check each keypoint with each projected landmark
      if ((kdl.corners[key_point_index] -
           projected_points[projected_point_index])
              .norm() > match_max_dist_2d) {
        // std::cout << " projected_point_index " << projected_point_index << "
        // norm big " << std::endl;
        continue;
      }

      // candidate match
      // find min dist for this keypoint and this landmark
      // check all descriptors of the keypoints of this landmark with keypoint
      // descriptor
      // similar to keypoints.h file
      size_t min_dist_for_descriptors = 256;
      for (auto& frame_feature_pair :
           landmarks.at(projected_track_ids[projected_point_index]).obs) {
        std::bitset<256> frame_descriptor =
            feature_corners.at(frame_feature_pair.first)
                .corner_descriptors[frame_feature_pair.second];

        std::bitset<256> diff_vector = frame_descriptor ^ keypoint_descriptor;
        size_t dist = diff_vector.count();
        if (dist < min_dist_for_descriptors) {
          min_dist_for_descriptors = dist;
        }
      }
      // std::cout << " min desc dist " << min_dist_for_descriptors << " for
      // landmark index" << projected_point_index << std::endl;

      if (min_dist_for_descriptors < min_dist1) {
        min_dist2 = min_dist1;
        min_dist1 = min_dist_for_descriptors;
        best_match = projected_point_index;
        // std::cout << " min1 dist " << min_dist1 << " min2 dist" << min_dist2
        // << std::endl;

      } else if (min_dist_for_descriptors < min_dist2) {
        min_dist2 = min_dist_for_descriptors;
        // std::cout << " min2 dist" << min_dist2 << std::endl;
      }
    }

    if (int(min_dist1) < feature_match_threshold &&
        feature_match_dist_2_best * int(min_dist1) <= int(min_dist2)) {
      // <featureId,trackId>
      // std::cout << "OKOKOKOK min1 dist " << min_dist1 << " min2 dist" <<
      // min_dist2 << std::endl;
      md.matches.push_back(std::pair<int, int>(
          int(key_point_index), projected_track_ids[best_match]));
    }
  }
}

void localize_camera(const Sophus::SE3d& current_pose,
                     const std::shared_ptr<AbstractCamera<double>>& cam,
                     const KeypointsData& kdl, const Landmarks& landmarks,
                     const double reprojection_error_pnp_inlier_threshold_pixel,
                     LandmarkMatchData& md) {
  md.inliers.clear();

  // default to previous pose if not enough inliers
  md.T_w_c = current_pose;

  if (md.matches.size() < 4) {
    return;
  }

  // TODO SHEET 5: Find the pose (md.T_w_c) and the inliers (md.inliers) using
  // the landmark to keypoints matches and PnP. This should be similar to the
  // localize_camera in exercise 4 but in this exercise we don't explicitly have
  // tracks.

  // double fx = cam->getParam()[0];
  // double fy = cam->getParam()[1];
  // std::cout << " focal " << fx << " " << fy << std::endl;
  // 350 - 351
  double f = 500;  // fx

  double threshold_ransac =
      1.0 - cos(atan(reprojection_error_pnp_inlier_threshold_pixel / f));

  opengv::bearingVectors_t bearingVectors;
  opengv::points_t points;

  for (auto& match : md.matches) {
    // match.first feature id
    // match.second track id

    // beam from cam to landmark
    opengv::bearingVector_t beam_3d = cam->unproject(kdl.corners[match.first]);
    bearingVectors.push_back(beam_3d);

    // point wrt world
    opengv::point_t point = landmarks.at(match.second).p;
    points.push_back(point);
  }

  // the rest is same as localize_camera in map_utils.h
  // https://github.com/laurentkneip/opengv/blob/master/test/test_absolute_pose_sac.cpp#L94
  // create the central adapter
  opengv::absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors, points);
  // create a Ransac object
  opengv::sac::Ransac<
      opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      ransac;
  // create an AbsolutePoseSacProblem
  // (algorithm is selectable: KNEIP, GAO, or EPNP)
  std::shared_ptr<opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem>
      absposeproblem_ptr(
          new opengv::sac_problems::absolute_pose::AbsolutePoseSacProblem(
              adapter, opengv::sac_problems::absolute_pose::
                           AbsolutePoseSacProblem::KNEIP));
  // run ransac
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = threshold_ransac;
  // ransac.max_iterations_ = 1000;  // maxIterations;
  ransac.computeModel();

  // ransac result
  // opengv::transformation_t ransac_transformation =
  // ransac.model_coefficients_;

  // refined result
  opengv::transformation_t refined_transformation;

  ransac.sac_model_->optimizeModelCoefficients(
      ransac.inliers_, ransac.model_coefficients_, refined_transformation);

  Eigen::Vector3d t12 = refined_transformation.block(0, 3, 3, 1);
  Eigen::Matrix3d R12 = refined_transformation.block(0, 0, 3, 3);
  md.T_w_c = Sophus::SE3d(R12, t12);

  // inlier_track_ids <- ransac.inliers_;
  for (auto inlier_index : ransac.inliers_) {
    md.inliers.push_back(md.matches[inlier_index]);
  }
}

void add_new_landmarks(const FrameCamId fcidl, const FrameCamId fcidr,
                       const KeypointsData& kdl, const KeypointsData& kdr,
                       const Calibration& calib_cam, const MatchData& md_stereo,
                       const LandmarkMatchData& md, Landmarks& landmarks,
                       TrackId& next_landmark_id) {
  // input should be stereo pair
  assert(fcidl.cam_id == 0);
  assert(fcidr.cam_id == 1);

  const Sophus::SE3d T_0_1 = calib_cam.T_i_c[0].inverse() * calib_cam.T_i_c[1];
  const Eigen::Vector3d t_0_1 = T_0_1.translation();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  // TODO SHEET 5: Add new landmarks and observations. Here md_stereo contains
  // stereo matches for the current frame and md contains feature to landmark
  // matches for the left camera (camera 0). For all inlier feature to landmark
  // matches add the observations to the existing landmarks. If the left
  // camera's feature appears also in md_stereo.inliers, then add both
  // observations. For all inlier stereo observations that were not added to the
  // existing landmarks, triangulate and add new landmarks. Here
  // next_landmark_id is a running index of the landmarks, so after adding a new
  // landmark you should always increase next_landmark_id by 1.

  for (auto& match : md.inliers) {
    // match.first feature id for left cam fcidl
    // match.second track id

    // For all inlier feature to landmark matches
    // add the observations to the existing landmarks
    // std::pair<const visnav::FrameCamId, visnav::FeatureId> id
    landmarks.at(match.second).obs.insert({fcidl, match.first});

    // If the left camera's feature appears also in md_stereo.inliers,
    // then add both observations.
    // TODO: better way instead of for? maybe contain()?
    for (auto& stereo_match : md_stereo.inliers) {
      // stereo_match.first feature id for left cam fcidl
      // stereo_match.second feature id for right cam fcidr

      if (stereo_match.first == match.first) {
        landmarks.at(match.second).obs.insert({fcidr, stereo_match.second});
      }
    }
  }

  // For all inlier stereo observations that were not added to the
  // existing landmarks, triangulate and add new landmarks.
  for (auto& stereo_match : md_stereo.inliers) {
    // stereo_match.first feature id for left cam fcidl
    // stereo_match.second feature id for right cam fcidr

    bool exist = false;
    // std::cout << " search for stereo feature " << stereo_match.first <<
    // std::endl;

    for (auto& match : md.inliers) {
      // match.first feature id for left cam fcidl
      // match.second track id

      if (match.first == stereo_match.first) {
        // landmark exists
        // std::cout << " found stereo feature " << stereo_match.first << "
        // track id " << match.second << std::endl;
        exist = true;
        break;
      }
    }

    if (exist) {
      continue;
    }

    // std::cout << " not found stereo feature " << stereo_match.first <<
    // std::endl;

    // add new landmark
    opengv::bearingVector_t p0_3d =
        calib_cam.intrinsics[0]->unproject(kdl.corners[stereo_match.first]);

    opengv::bearingVector_t p1_3d =
        calib_cam.intrinsics[1]->unproject(kdr.corners[stereo_match.second]);

    opengv::bearingVectors_t p0_3d_vector, p1_3d_vector;
    p0_3d_vector.push_back(p0_3d);
    p1_3d_vector.push_back(p1_3d);

    opengv::relative_pose::CentralRelativeAdapter adapter(
        p0_3d_vector, p1_3d_vector, t_0_1, R_0_1);

    size_t index = 0;

    // run method 1
    Eigen::Vector3d point = opengv::triangulation::triangulate(adapter, index);

    Landmark l;

    l.p = md.T_w_c * point;
    l.obs.insert({fcidl, stereo_match.first});
    l.obs.insert({fcidr, stereo_match.second});

    landmarks[next_landmark_id++] = l;
  }
}

void remove_old_keyframes(const FrameCamId fcidl, const int max_num_kfs,
                          Cameras& cameras, Landmarks& landmarks,
                          Landmarks& old_landmarks,
                          std::set<FrameId>& kf_frames) {
  kf_frames.emplace(fcidl.frame_id);

  // TODO SHEET 5: Remove old cameras and observations if the number of keyframe
  // pairs (left and right image is a pair) is larger than max_num_kfs. The ids
  // of all the keyframes that are currently in the optimization should be
  // stored in kf_frames. Removed keyframes should be removed from cameras and
  // landmarks with no left observations should be moved to old_landmarks.

  for (int size = int(kf_frames.size()); size > max_num_kfs; size--) {
    std::cout << "remove frame pair" << std::endl;
    // The frame IDs can be seen as timestamps
    // for the images, i.e. they induce a temporal order on keyframes.

    FrameId old_frame = *kf_frames.begin();
    FrameCamId old_frame_cam_left(old_frame, 0);
    FrameCamId old_frame_cam_right(old_frame, 1);

    cameras.erase(old_frame_cam_left);
    cameras.erase(old_frame_cam_right);

    for (auto& landmark : landmarks) {
      landmark.second.obs.erase(old_frame_cam_left);
      landmark.second.obs.erase(old_frame_cam_right);

      if (landmark.second.obs.size() == 0) {
        old_landmarks.insert(landmark);
        // landmarks.erase(landmark.first); // TODO: not working?
        // iterate the map but delete the element, is it possible?
      }
    }

    for (auto& landmark : old_landmarks) {
      landmarks.erase(landmark.first);
    }

    kf_frames.erase(old_frame);
  }
}
}  // namespace visnav
