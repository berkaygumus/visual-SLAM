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
  UNUSED(cam);
  UNUSED(kdl);
  UNUSED(landmarks);
  UNUSED(reprojection_error_pnp_inlier_threshold_pixel);
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
  UNUSED(fcidl);
  UNUSED(fcidr);
  UNUSED(kdl);
  UNUSED(kdr);
  UNUSED(calib_cam);
  UNUSED(md_stereo);
  UNUSED(md);
  UNUSED(landmarks);
  UNUSED(next_landmark_id);
  UNUSED(t_0_1);
  UNUSED(R_0_1);
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
  UNUSED(max_num_kfs);
  UNUSED(cameras);
  UNUSED(landmarks);
  UNUSED(old_landmarks);
}
}  // namespace visnav
