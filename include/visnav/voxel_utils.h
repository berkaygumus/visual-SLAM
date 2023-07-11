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

#include <visnav/common_types.h>

namespace visnav {

void calculate_voxel_distribution(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double resolution,
    std::pair<Eigen::Vector3d, voxel_dist>& voxels) {
  // TODO: calculate voxel distribution
  std::cout << " res: " << resolution << std::endl;
}

}  // namespace visnav