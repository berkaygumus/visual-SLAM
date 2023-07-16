// Implement voxelization & PCA
// Save/load voxels, it is offline
// 1 m res

/*
@param 3Dpoints
@param resolution
@return voxels

TODO:type for the efficient search in icp?
for voxel_dist, there is a struct type
for voxels, type? pair<3dpos,voxel> ? or  3d array of voxel?
*/

#pragma once

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <unordered_map>

#include <visnav/common_types.h>

#include <cmath>

namespace visnav {

void find_voxel_center(const Eigen::Vector3f point, const double resolution,
                       Eigen::Vector3f& voxel_center) {
  voxel_center = point / resolution;
  voxel_center[0] = roundf(voxel_center[0]);
  voxel_center[1] = roundf(voxel_center[1]);
  voxel_center[2] = roundf(voxel_center[2]);
}

void set_voxel_points(const std::vector<Eigen::Vector3f> global_map_points,
                      const double resolution, Voxels& voxels) {
  Eigen::Vector3f voxel_center;
  for (auto point : global_map_points) {
    find_voxel_center(point, resolution, voxel_center);

    // std::cout << " voxel size " << voxels.size() << std::endl;
    // for(auto v : voxels){
    //  std::cout << "v " << v.first.transpose() << " " <<
    //  v.second.voxel_center.transpose() << std::endl;
    //}

    if (voxels.find(voxel_center) == voxels.end()) {
      // std::cout << " point " << std::endl << point.transpose() << std::endl;
      // std::cout << " new voxel_center " << std::endl
      //          << voxel_center.transpose() << std::endl;
      // new voxel
      Voxel voxel;
      voxel.voxel_center = voxel_center;
      // insert point into voxel
      voxel.points.push_back(point);
      voxels.insert({voxel_center, voxel});
      // std::cout << " voxel_center " << std::endl
      //          << voxel.voxel_center.transpose() << std::endl
      //          << "point size " << voxel.points.size() << std::endl;
    }

    else {
      // std::cout << " point " << std::endl << point.transpose() << std::endl;
      // std::cout << " existing voxel_center " << std::endl
      //          << voxel_center.transpose() << std::endl;
      // existing voxel
      Voxel& voxel = voxels[voxel_center];
      // std::cout << " voxel_center " << std::endl
      //          << voxel.voxel_center.transpose() << std::endl;
      // std::cout << "point size before insert"
      //          << voxels[voxel_center].points.size() << std::endl;
      // insert point into voxel
      voxel.points.push_back(point);
      // std::cout << "point size after insert"
      //          << voxels[voxel_center].points.size() << std::endl;
    }
  }
}

void perform_pca(const std::vector<Eigen::Vector3f> global_map_points,
                 Voxels& voxels) {
  std::cout << " voxels size " << voxels.size() << std::endl;
  for (auto& v : voxels) {
    int n = v.second.points.size();
    // point matrix
    Eigen::MatrixXf points;
    points.conservativeResize(n, 3);

    std::cout << std::endl
              << "pca for " << v.first.transpose() << " size " << n
              << std::endl;
    for (int i = 0; i < n; i++) {
      points.row(i) = v.second.points[i];
    }

    // distribution

    Eigen::VectorXf mean = points.colwise().mean();
    std::cout << "mean " << mean.transpose() << std::endl;

    // Subtract the mean from each data point
    points.rowwise() -= mean.transpose();

    // Compute the covariance matrix
    Eigen::MatrixXf covariance =
        (points.adjoint() * points) / static_cast<float>(points.rows());
    //(points.adjoint() * points) / static_cast<float>(points.rows() - 1);
    std::cout << "covariance " << std::endl << covariance << std::endl;

    // Perform eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success) {
      std::cerr << "Eigenvalue decomposition failed!" << std::endl;
    }

    // Retrieve the eigenvalues and eigenvectors
    Eigen::VectorXf eigenvalues = eigensolver.eigenvalues();
    Eigen::MatrixXf eigenvectors = eigensolver.eigenvectors();

    // Normalize eigenvectors
    eigenvectors.colwise().normalize();

    // sort eigenvalues acc. to eigenvalues
    // TODO: is it already sorted?
    std::vector<std::pair<float, int>> eigen_value_index;

    // Inserting element in pair vector
    // to keep track of previous indexes
    for (int i = 0; i < 3; ++i) {
      eigen_value_index.push_back(std::make_pair(abs(eigenvalues[i]), i));
    }

    // Sorting pair vector
    std::sort(eigen_value_index.begin(), eigen_value_index.end());

    // Displaying sorted element
    // with previous indexes
    // corresponding to each element
    std::cout << "eigenvalues and index" << std::endl;
    for (int i = 0; i < eigen_value_index.size(); i++) {
      std::cout << eigen_value_index[i].first << "\t"
                << eigen_value_index[i].second << std::endl;
    }

    // Print the results
    std::cout << "Eigenvalues:\n" << eigenvalues.transpose() << std::endl;
    std::cout << "\nEigenvectors:\n" << eigenvectors << std::endl;

    Eigen::Matrix3f R;
    R.col(0) = eigenvectors.col(eigen_value_index[2].second);
    R.col(1) = eigenvectors.col(eigen_value_index[1].second);
    R.col(2) = R.col(0).cross(R.col(1));
    // eigenvectors.col(eigen_value_index[0].second);

    std::cout << "\r Rotation:\n" << R << std::endl;

    Sophus::SE3f T_inv = Sophus::SE3f(R, mean);
    std::cout << "T_inv:\n" << T_inv.matrix() << std::endl;
    Sophus::SE3f T = T_inv.inverse();
    // Sophus::SE3f A = T * Sophus::SE3f(covariance, Eigen::Vector3f::Zero())  *
    // T_inv;

    Eigen::Matrix4f covariance4f =
        Eigen::Matrix4f::Identity();  // covariance  * T_inv;
    covariance4f.block(0, 0, 3, 3) = covariance;
    Eigen::Matrix4f A = T.matrix() * covariance4f * T_inv.matrix();
    Eigen::Vector3f sigma_vec;
    sigma_vec << sqrt(A(0, 0)), sqrt(A(1, 1)), sqrt(A(2, 2));
    std::cout << "sigma_vec " << sigma_vec << std::endl;

    v.second.N = n;
    v.second.T = T;
    v.second.sigma = sigma_vec;
  }
}

void calculate_voxel_distribution(
    const std::vector<Eigen::Vector3f> global_map_points,
    const double resolution, Voxels& voxels) {
  // TODO: calculate voxel distribution
  std::cout << " res: " << resolution << std::endl;
  set_voxel_points(global_map_points, resolution, voxels);
  perform_pca(global_map_points, voxels);
}

}  // namespace visnav