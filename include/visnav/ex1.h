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

#include <sophus/se3.hpp>

#include <visnav/common_types.h>

namespace visnav {

// Implement exp for SO(3)
template <class T>
Eigen::Matrix<T, 3, 3> user_implemented_expmap(
    const Eigen::Matrix<T, 3, 1>& xi) {
  // TODO SHEET 1: implement
  double theta = xi.norm();
  if (theta == 0) {
    return Eigen::MatrixXd::Identity(3, 3);
  }
  Eigen::Matrix<T, 3, 3> skew_symmetric_xi, mat;
  skew_symmetric_xi << 0, -xi(2, 0), xi(1, 0), xi(2, 0), 0, -xi(0, 0),
      -xi(1, 0), xi(0, 0), 0;

  mat =
      Eigen::MatrixXd::Identity(3, 3) + sin(theta) / theta * skew_symmetric_xi +
      (1 - cos(theta)) / theta / theta * skew_symmetric_xi * skew_symmetric_xi;

  return mat;
}

// Implement log for SO(3)
template <class T>
Eigen::Matrix<T, 3, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 3, 3>& mat) {
  // TODO SHEET 1: implement
  double theta = acos((mat.trace() - 1) / 2);
  if (theta == 0) {
    return Eigen::MatrixXd::Zero(3, 1);
  }

  Eigen::Matrix<T, 3, 1> temp_vec, xi;
  temp_vec << mat(2, 1) - mat(1, 2), mat(0, 2) - mat(2, 0),
      mat(1, 0) - mat(0, 1);
  xi = theta / 2 / sin(theta) * temp_vec;

  return xi;
}

// Implement exp for SE(3)
template <class T>
Eigen::Matrix<T, 4, 4> user_implemented_expmap(
    const Eigen::Matrix<T, 6, 1>& xi) {
  // TODO SHEET 1: implement
  Eigen::Matrix<T, 3, 1> w, v;
  w = xi.tail(3);
  v = xi.head(3);

  double theta = w.norm();
  Eigen::Matrix<T, 4, 4> mat;
  mat = Eigen::MatrixXd::Identity(4, 4);
  if (theta == 0) {
    mat.block(0, 3, 3, 1) = v;
    return mat;
  }

  Eigen::Matrix<T, 3, 3> skew_symmetric_w, matR, J;

  skew_symmetric_w << 0, -w(2, 0), w(1, 0), w(2, 0), 0, -w(0, 0), -w(1, 0),
      w(0, 0), 0;

  matR = Eigen::MatrixXd::Identity(3, 3) +
         sin(theta) / theta * skew_symmetric_w +
         (1 - cos(theta)) / theta / theta * skew_symmetric_w * skew_symmetric_w;

  J = Eigen::MatrixXd::Identity(3, 3) +
      (1 - cos(theta)) / theta / theta * skew_symmetric_w +
      (theta - sin(theta)) / theta / theta / theta * skew_symmetric_w *
          skew_symmetric_w;

  mat.block(0, 0, 3, 3) = matR;
  mat.block(0, 3, 3, 1) = J * v;

  return mat;
}

// Implement log for SE(3)
template <class T>
Eigen::Matrix<T, 6, 1> user_implemented_logmap(
    const Eigen::Matrix<T, 4, 4>& mat) {
  // TODO SHEET 1: implement

  Eigen::Matrix<T, 3, 1> matT, temp_vec, w, v;
  Eigen::Matrix<T, 3, 3> matR, J_inverse, skew_symmetric_w;
  Eigen::Matrix<T, 6, 1> xi;
  // std::cout << "mat: " << std::endl << mat << std::endl;

  matR = mat.block(0, 0, 3, 3);
  matT = mat.block(0, 3, 3, 1);
  double theta = acos((matR.trace() - 1) / 2);
  // std::cout << "theta: " << theta << std::endl;
  if (theta == 0) {
    xi.block(0, 0, 3, 1) = matT;
    xi.block(3, 0, 3, 1) = Eigen::MatrixXd::Zero(3, 1);
    // std::cout << "xi1: " << std::endl << xi << std::endl;
    return xi;
  }
  temp_vec << matR(2, 1) - matR(1, 2), matR(0, 2) - matR(2, 0),
      matR(1, 0) - matR(0, 1);
  w = theta / 2 / sin(theta) * temp_vec;

  skew_symmetric_w << 0, -w(2, 0), w(1, 0), w(2, 0), 0, -w(0, 0), -w(1, 0),
      w(0, 0), 0;

  J_inverse = Eigen::MatrixXd::Identity(3, 3) - skew_symmetric_w / 2 +
              (1 / theta / theta - (1 + cos(theta)) / 2 / theta / sin(theta)) *
                  skew_symmetric_w * skew_symmetric_w;

  v = J_inverse * matT;

  xi.block(0, 0, 3, 1) = v;
  xi.block(3, 0, 3, 1) = w;

  // std::cout << "xi2: " << std::endl << xi << std::endl;

  return xi;
}

}  // namespace visnav
