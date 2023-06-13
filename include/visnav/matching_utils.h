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

#include <bitset>
#include <set>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include <visnav/camera_models.h>
#include <visnav/common_types.h>

namespace visnav {

void computeEssential(const Sophus::SE3d& T_0_1, Eigen::Matrix3d& E) {
  const Eigen::Vector3d t_0_1 = T_0_1.translation().normalized();
  const Eigen::Matrix3d R_0_1 = T_0_1.rotationMatrix();

  // TODO SHEET 3: compute essential matrix
  Eigen::Matrix3d T_skew;
  T_skew << 0, -t_0_1[2], t_0_1[1], t_0_1[2], 0, -t_0_1[0], -t_0_1[1], t_0_1[0],
      0;

  E = T_skew * R_0_1;
}

void findInliersEssential(const KeypointsData& kd1, const KeypointsData& kd2,
                          const std::shared_ptr<AbstractCamera<double>>& cam1,
                          const std::shared_ptr<AbstractCamera<double>>& cam2,
                          const Eigen::Matrix3d& E,
                          double epipolar_error_threshold, MatchData& md) {
  md.inliers.clear();

  for (size_t j = 0; j < md.matches.size(); j++) {
    const Eigen::Vector2d p0_2d = kd1.corners[md.matches[j].first];
    const Eigen::Vector2d p1_2d = kd2.corners[md.matches[j].second];

    // TODO SHEET 3: determine inliers and store in md.inliers
    const Eigen::Vector3d p0_3d = cam1->unproject(p0_2d);
    const Eigen::Vector3d p1_3d = cam2->unproject(p1_2d);

    if (abs(p0_3d.transpose() * E * p1_3d) <= epipolar_error_threshold) {
      // std::cout << " epipolar " << abs(p0_3d.transpose() * E * p1_3d) << " "
      // << epipolar_error_threshold << std::endl;
      md.inliers.emplace_back(
          std::pair<int, int>(md.matches[j].first, md.matches[j].second));
    }
  }
}

void findInliersRansac(const KeypointsData& kd1, const KeypointsData& kd2,
                       const std::shared_ptr<AbstractCamera<double>>& cam1,
                       const std::shared_ptr<AbstractCamera<double>>& cam2,
                       const double ransac_thresh, const int ransac_min_inliers,
                       MatchData& md) {
  md.inliers.clear();
  md.T_i_j = Sophus::SE3d();

  // TODO SHEET 3: Run RANSAC with using opengv's CentralRelativePose and store
  // the final inlier indices in md.inliers and the final relative pose in
  // md.T_i_j (normalize translation). If the number of inliers is smaller than
  // ransac_min_inliers, leave md.inliers empty. Note that if the initial RANSAC
  // was successful, you should do non-linear refinement of the model parameters
  // using all inliers, and then re-estimate the inlier set with the refined
  // model parameters.

  opengv::bearingVectors_t bearingVectors1, bearingVectors2;

  for (size_t j = 0; j < md.matches.size(); j++) {
    const Eigen::Vector2d p0_2d = kd1.corners[md.matches[j].first];
    const Eigen::Vector2d p1_2d = kd2.corners[md.matches[j].second];

    const opengv::bearingVector_t p0_3d = cam1->unproject(p0_2d);
    const opengv::bearingVector_t p1_3d = cam2->unproject(p1_2d);

    bearingVectors1.emplace_back(p0_3d);
    bearingVectors2.emplace_back(p1_3d);
  }

  // from
  // https://laurentkneip.github.io/opengv/page_how_to_use.html#sec_conventions

  // 1) ------>firstly, find inliers with RANSAC

  // create the central relative adapter
  opengv::relative_pose::CentralRelativeAdapter adapter(bearingVectors1,
                                                        bearingVectors2);

  // create a RANSAC object
  opengv::sac::Ransac<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
      ransac;
  // create a CentralRelativePoseSacProblem
  // (set algorithm to NISTER 5 point algorithm)
  std::shared_ptr<
      opengv::sac_problems::relative_pose::CentralRelativePoseSacProblem>
      relposeproblem_ptr(
          new opengv::sac_problems::relative_pose::
              CentralRelativePoseSacProblem(
                  adapter, opengv::sac_problems::relative_pose::
                               CentralRelativePoseSacProblem::NISTER));
  // run ransac
  ransac.sac_model_ = relposeproblem_ptr;
  ransac.threshold_ = ransac_thresh;
  ransac.max_iterations_ = 1000;  // default 1000
  if (!ransac.computeModel()) {
    return;
  }
  // get the result
  opengv::transformation_t best_transformation = ransac.model_coefficients_;
  std::vector<int> inliers_ = ransac.inliers_;

  if (int(inliers_.size()) < ransac_min_inliers) {
    // If the number of inliers is smaller than
    // ransac_min_inliers, leave md.inliers empty
    return;
  }

  // 2) ------>secondly, optimize with all inliers
  // non-linear optimization (using all inlier correspondences)

  opengv::bearingVectors_t inlier_bearingVectors1, inlier_bearingVectors2;

  for (int inlier_index : inliers_) {
    inlier_bearingVectors1.emplace_back(bearingVectors1[inlier_index]);
    inlier_bearingVectors2.emplace_back(bearingVectors2[inlier_index]);

    md.inliers.emplace_back(std::pair<int, int>(
        md.matches[inlier_index].first, md.matches[inlier_index].second));
  }

  opengv::relative_pose::CentralRelativeAdapter nonlinear_adapter(
      inlier_bearingVectors1, inlier_bearingVectors2);
  // best_transformation is Eigen::Matrix<double,3,4
  nonlinear_adapter.sett12(best_transformation.block(0, 3, 3, 1));
  nonlinear_adapter.setR12(best_transformation.block(0, 0, 3, 3));
  opengv::transformation_t nonlinear_transformation =
      opengv::relative_pose::optimize_nonlinear(nonlinear_adapter);

  // md.T_i_j.block(0, 0, 3, 4) = nonlinear_transformation;
  // md.T_i_j.setRotationMatrix(nonlinear_transformation.block(0, 0, 3, 3));
  Eigen::Vector3d t12 = nonlinear_transformation.block(0, 3, 3, 1);
  Eigen::Matrix3d R12 = nonlinear_transformation.block(0, 0, 3, 3);
  md.T_i_j = Sophus::SE3d(R12, t12);
}
}  // namespace visnav
