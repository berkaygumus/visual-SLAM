#pragma once

#include <bitset>
#include <set>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <opengv/relative_pose/CentralRelativeAdapter.hpp>
#include <opengv/relative_pose/methods.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/relative_pose/CentralRelativePoseSacProblem.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <visnav/camera_models.h>
#include <visnav/common_types.h>

namespace visnav {

void calculate_voxel_distribution(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double resolution) {
  std::cout << " res: " << resolution << std::endl;
}

}  // namespace visnav