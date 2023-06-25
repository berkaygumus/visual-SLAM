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

#include <fstream>
#include <thread>

#include <ceres/ceres.h>

#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <opengv/triangulation/methods.hpp>

#include <visnav/common_types.h>
#include <visnav/serialization.h>

#include <visnav/reprojection.h>
#include <visnav/local_parameterization_se3.hpp>

#include <visnav/tracks.h>

namespace visnav {

// save map with all features and matches
void save_map_file(const std::string& map_path, const Corners& feature_corners,
                   const Matches& feature_matches,
                   const FeatureTracks& feature_tracks,
                   const FeatureTracks& outlier_tracks, const Cameras& cameras,
                   const Landmarks& landmarks) {
  {
    std::ofstream os(map_path, std::ios::binary);

    if (os.is_open()) {
      cereal::BinaryOutputArchive archive(os);
      archive(feature_corners);
      archive(feature_matches);
      archive(feature_tracks);
      archive(outlier_tracks);
      archive(cameras);
      archive(landmarks);

      size_t num_obs = 0;
      for (const auto& kv : landmarks) {
        num_obs += kv.second.obs.size();
      }
      std::cout << "Saved map as " << map_path << " (" << cameras.size()
                << " cameras, " << landmarks.size() << " landmarks, " << num_obs
                << " observations)" << std::endl;
    } else {
      std::cout << "Failed to save map as " << map_path << std::endl;
    }
  }
}

// load map with all features and matches
void load_map_file(const std::string& map_path, Corners& feature_corners,
                   Matches& feature_matches, FeatureTracks& feature_tracks,
                   FeatureTracks& outlier_tracks, Cameras& cameras,
                   Landmarks& landmarks) {
  {
    std::ifstream is(map_path, std::ios::binary);

    if (is.is_open()) {
      cereal::BinaryInputArchive archive(is);
      archive(feature_corners);
      archive(feature_matches);
      archive(feature_tracks);
      archive(outlier_tracks);
      archive(cameras);
      archive(landmarks);

      size_t num_obs = 0;
      for (const auto& kv : landmarks) {
        num_obs += kv.second.obs.size();
      }
      std::cout << "Loaded map from " << map_path << " (" << cameras.size()
                << " cameras, " << landmarks.size() << " landmarks, " << num_obs
                << " observations)" << std::endl;
    } else {
      std::cout << "Failed to load map from " << map_path << std::endl;
    }
  }
}

// Create new landmarks from shared feature tracks if they don't already exist.
// The two cameras must be in the map already.
// Returns the number of newly created landmarks.
int add_new_landmarks_between_cams(const FrameCamId& fcid0,
                                   const FrameCamId& fcid1,
                                   const Calibration& calib_cam,
                                   const Corners& feature_corners,
                                   const FeatureTracks& feature_tracks,
                                   const Cameras& cameras,
                                   Landmarks& landmarks) {
  // shared_track_ids will contain all track ids shared between the two images,
  // including existing landmarks
  std::vector<TrackId> shared_track_ids;

  // find shared feature tracks
  const std::set<FrameCamId> fcids = {fcid0, fcid1};
  if (!GetTracksInImages(fcids, feature_tracks, shared_track_ids)) {
    return 0;
  }

  // at the end of the function this will contain all newly added track ids
  std::vector<TrackId> new_track_ids;

  // TODO SHEET 4: Triangulate all new features and add to the map
  // std::cout << " shared size " << shared_track_ids.size() << std::endl;

  for (auto shared_track_id : shared_track_ids) {
    // std::pair<const visnav::TrackId, visnav::FeatureTrack> f_track
    // f_track.second[fcid0] featureID of the landmark in fcid0 and f_track
    // feature_corners[fcid0].corners[f_track.second[fcid0]] is 2d point in the
    // image

    // if the landmark (track id) exists, skip
    if (landmarks.find(shared_track_id) != landmarks.end()) {
      continue;
    }

    new_track_ids.push_back(shared_track_id);

    Landmark l;
    for (auto id : feature_tracks.at(shared_track_id)) {
      if (cameras.find(id.first) != cameras.end()) {
        l.obs.insert(id);
      }
    }

    opengv::bearingVector_t p0_3d =
        calib_cam.intrinsics[fcid0.cam_id]->unproject(
            feature_corners.at(fcid0)
                .corners[feature_tracks.at(shared_track_id).at(fcid0)]);

    opengv::bearingVector_t p1_3d =
        calib_cam.intrinsics[fcid1.cam_id]->unproject(
            feature_corners.at(fcid1)
                .corners[feature_tracks.at(shared_track_id).at(fcid1)]);

    opengv::bearingVectors_t p0_3d_vector, p1_3d_vector;
    p0_3d_vector.push_back(p0_3d);
    p1_3d_vector.push_back(p1_3d);

    Sophus::SE3d T_c0_c1 =
        cameras.at(fcid0).T_w_c.inverse() * cameras.at(fcid1).T_w_c;
    Sophus::Matrix3d rotation = T_c0_c1.rotationMatrix();
    Sophus::Vector3d translation = T_c0_c1.translation();

    /*
    std::cout << std::endl
              << " fcid0 " << fcid0 << " fcid1 " << fcid1 << " rot "
              << std::endl
              << rotation << std::endl
              << " t " << std::endl
              << translation << std::endl
              << " beam0 " << std::endl
              << p0_3d_vector[0] << std::endl
              << " beam1 " << std::endl
              << p1_3d_vector[0] << std::endl;
    */
    opengv::relative_pose::CentralRelativeAdapter adapter(
        p0_3d_vector, p1_3d_vector, translation, rotation);

    size_t index = 0;

    // run method 1
    Eigen::Vector3d point = opengv::triangulation::triangulate(adapter, index);

    // std::cout << " point0 " << std::endl << point << std::endl;

    // std::cout << " rot " << std::endl <<
    // cameras.at(fcid0).T_w_c.rotationMatrix() << " t " << std::endl <<
    // cameras.at(fcid0).T_w_c.translation() << std::endl;

    l.p = cameras.at(fcid0).T_w_c * point;
    // opengv::triangulation::triangulate(adapter, index);

    // std::cout << " tri " << shared_track_id << std::endl << point <<
    // std::endl;

    // std::cout << " point1 " << std::endl<< l.p << std::endl;

    // std::cout << " TrackId " << shared_track_id << "  " << std::endl;
    // for (auto f : feature_tracks.at(shared_track_id)) {
    //  std::cout << "FeatureTrack  " << f.first << " " << f.second <<
    //  std::endl;
    //}

    // f_track.second feature track
    landmarks[shared_track_id] = l;
  }

  return new_track_ids.size();
}

// Initialize the scene from a stereo pair, using the known transformation from
// camera calibration. This adds the inital two cameras and triangulates shared
// landmarks.
// Note: in principle we could also initialize a map from another images pair
// using the transformation from the pairwise matching with the 5-point
// algorithm. However, using a stereo pair has the advantage that the map is
// initialized with metric scale.
bool initialize_scene_from_stereo_pair(const FrameCamId& fcid0,
                                       const FrameCamId& fcid1,
                                       const Calibration& calib_cam,
                                       const Corners& feature_corners,
                                       const FeatureTracks& feature_tracks,
                                       Cameras& cameras, Landmarks& landmarks) {
  // check that the two image ids refer to a stereo pair
  if (!(fcid0.frame_id == fcid1.frame_id && fcid0.cam_id != fcid1.cam_id)) {
    std::cerr << "Images " << fcid0 << " and " << fcid1
              << " don't form a stereo pair. Cannot initialize." << std::endl;
    return false;
  }

  // TODO SHEET 4: Initialize scene (add initial cameras and landmarks)
  Camera cam0, cam1;
  cam0.T_w_c = Sophus::SE3d(
      Eigen::Matrix4d::Identity());  // calib_cam.T_i_c[fcid0.cam_id];
                                     // // identity
  cam1.T_w_c = calib_cam.T_i_c[fcid0.cam_id].inverse() *
               calib_cam.T_i_c[fcid1.cam_id];  // T from stereo calibration
  cameras[fcid0] = cam0;
  cameras[fcid1] = cam1;

  // std::cout << " feature_tracks " << feature_tracks.size() << std::endl;
  // std::cout << " feature_corners " << feature_corners.size() << std::endl;

  /// Feature tracks are collections of {ImageId => FeatureId}.
  /// I.e. a collection of all images that observed this feature and the
  /// corresponding feature index in that image.
  // using FeatureTrack = std::map<FrameCamId, FeatureId>;
  /// FeatureTracks is a collection {TrackId => FeatureTrack}
  // using FeatureTracks = std::unordered_map<TrackId, FeatureTrack>;

  add_new_landmarks_between_cams(fcid0, fcid1, calib_cam, feature_corners,
                                 feature_tracks, cameras, landmarks);

  return true;
}

// Localize a new camera in the map given a set of observed landmarks. We use
// pnp and ransac to localize the camera in the presence of outlier tracks.
// After finding an inlier set with pnp, we do non-linear refinement using all
// inliers and also update the set of inliers using the refined pose.
//
// shared_track_ids already contains those tracks which the new image shares
// with the landmarks (but some might be outliers).
//
// We return the refined pose and the set of track ids for all inliers.
//
// The inlier threshold is given in pixels. See also the opengv documentation on
// how to convert this to a ransac threshold:
// http://laurentkneip.github.io/opengv/page_how_to_use.html#sec_threshold
void localize_camera(
    const FrameCamId& fcid, const std::vector<TrackId>& shared_track_ids,
    const Calibration& calib_cam, const Corners& feature_corners,
    const FeatureTracks& feature_tracks, const Landmarks& landmarks,
    const double reprojection_error_pnp_inlier_threshold_pixel,
    Sophus::SE3d& T_w_c, std::vector<TrackId>& inlier_track_ids) {
  inlier_track_ids.clear();

  // TODO SHEET 4: Localize a new image in a given map
  // double fx = calib_cam.intrinsics[fcid.cam_id]->getParam()[0];
  // double fy = calib_cam.intrinsics[fcid.cam_id]->getParam()[1];
  // std::cout << " focal " << fx << " " << fy << std::endl;
  // 350 - 351
  double f = 500;  // fx

  double threshold_ransac =
      1.0 - cos(atan(reprojection_error_pnp_inlier_threshold_pixel / f));

  opengv::bearingVectors_t bearingVectors;
  opengv::points_t points;

  for (auto shared_track_id : shared_track_ids) {
    // beam from cam to landmark
    opengv::bearingVector_t beam_3d =
        calib_cam.intrinsics[fcid.cam_id]->unproject(
            feature_corners.at(fcid)
                .corners[feature_tracks.at(shared_track_id).at(fcid)]);

    // point wrt world
    bearingVectors.push_back(beam_3d);
    opengv::point_t point = landmarks.at(shared_track_id).p;
    points.push_back(point);
  }

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
  T_w_c = Sophus::SE3d(R12, t12);

  // inlier_track_ids <- ransac.inliers_;
  for (auto inlier_index : ransac.inliers_) {
    inlier_track_ids.push_back(shared_track_ids[inlier_index]);
  }
}

struct BundleAdjustmentOptions {
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

// Run bundle adjustment to optimize cameras, points, and optionally intrinsics
void bundle_adjustment(const Corners& feature_corners,
                       const BundleAdjustmentOptions& options,
                       const std::set<FrameCamId>& fixed_cameras,
                       Calibration& calib_cam, Cameras& cameras,
                       Landmarks& landmarks) {
  ceres::Problem problem;

  // TODO SHEET 4: Setup optimization problem
  UNUSED(feature_corners);
  UNUSED(options);
  UNUSED(fixed_cameras);
  UNUSED(calib_cam);
  UNUSED(cameras);
  UNUSED(landmarks);

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = options.max_num_iterations;
  ceres_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_options.num_threads = std::thread::hardware_concurrency();
  ceres::Solver::Summary summary;
  Solve(ceres_options, &problem, &summary);
  switch (options.verbosity_level) {
    // 0: silent
    case 1:
      std::cout << summary.BriefReport() << std::endl;
      break;
    case 2:
      std::cout << summary.FullReport() << std::endl;
      break;
  }
}

}  // namespace visnav
