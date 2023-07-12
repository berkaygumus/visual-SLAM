
#pragma once

#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <visnav/common_types.h>

#include <ceres/ceres.h>
#include <visnav/local_parameterization_se3.hpp>

#include <thread>

namespace visnav {

// template <class T>

struct TransformationCostFunctor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  TransformationCostFunctor(const Eigen::Vector3d& global_map_point,
                            const Eigen::Vector3d& local_map_point)
      : global_map_point(global_map_point), local_map_point(local_map_point) {}

  template <typename T>
  bool operator()(T const* const sT_w_l, T* sResiduals) const {
    Eigen::Map<Sophus::SE3<T> const> const T_w_l(sT_w_l);

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(sResiduals);
    residuals = global_map_point - T_w_l * local_map_point;

    return true;
  }

  const Eigen::Vector3d local_map_point;
  const Eigen::Vector3d global_map_point;
};

// Run to optimize transformation
void estimate_pose(const std::vector<Eigen::Vector3f>& local_map_points,
                   const std::vector<Eigen::Vector3f>& global_map_points,
                   const ICPPairs& icp_pairs, Sophus::SE3d& T_w_l) {
  ceres::Problem problem;

  problem.AddParameterBlock(T_w_l.data(), Sophus::SE3d::num_parameters,
                            new Sophus::test::LocalParameterizationSE3);

  bool use_huber = false;
  double huber_parameter = 1;
  ceres::HuberLoss* lost_function;
  if (use_huber) {
    lost_function = new ceres::HuberLoss(huber_parameter);
  } else {
    lost_function = NULL;
  }

  for (auto& pair : icp_pairs) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<visnav::TransformationCostFunctor, 3,
                                        7>(
            new visnav::TransformationCostFunctor(
                local_map_points[pair.first].cast<double>(),
                global_map_points[pair.second].cast<double>()));
    // std::cout << "cost_function added " << std::endl;
    problem.AddResidualBlock(cost_function, lost_function, T_w_l.data());
  }

  // Solve
  ceres::Solver::Options ceres_options;
  ceres_options.max_num_iterations = 10;  // options.max_num_iterations;
  ceres_options.linear_solver_type = ceres::SPARSE_SCHUR;
  ceres_options.num_threads = std::thread::hardware_concurrency();
  ceres::Solver::Summary summary;
  Solve(ceres_options, &problem, &summary);
  int verbosity_level = 2;
  switch (verbosity_level) {
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
