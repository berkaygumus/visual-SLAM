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
#include <visnav/voxel_utils.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <flann/flann.hpp>

#include <visnav/common_types.h>

namespace visnav {

void find_closest_points_brute_force(const std::vector<Eigen::Vector3f> map,
                                     const Landmarks landmarks,
                                     ICPPairs& icp_pairs) {
  // TODO: find closest point on the global map using  brute force
  icp_pairs.clear();
  for (auto landmark : landmarks) {
    Eigen::Vector3d pos = landmark.second.p;
    int closest_index = -1;
    double closest_dist = 2;
    double dist;
    for (int i = 0; i < int(map.size()); i++) {
      dist = (pos - map[i].cast<double>()).norm();
      // Vector3f{ vertex.position.x(), vertex.position.y(), vertex.position.z()
      // }
      if (dist < closest_dist) {
        closest_index = i;
        closest_dist = dist;
      }
    }

    if (closest_index != -1) {
      icp_pairs.push_back(std::make_pair(closest_index, landmark.first));
    }
  }
}

class FlannMatch {
 public:
  FlannMatch() : m_nTrees{1}, m_index{nullptr}, m_flatPoints{nullptr} {}

  // TODO: best m_nTrees?

  // https://github.com/flann-lib/flann/blob/master/examples/flann_example.cpp

  ~FlannMatch() {
    if (m_index) {
      delete m_flatPoints;
      delete m_index;
      m_flatPoints = nullptr;
      m_index = nullptr;
    }
  }

  void buildIndex(std::vector<Eigen::Vector3f> global_map_points) {
    // build query for global map

    std::cout << "Building kd_tree for target points with "
              << global_map_points.size() << " points." << std::endl;

    // FLANN requires that all the points be flat. Therefore we copy the points
    // to a separate flat array.
    m_flatPoints = new float[global_map_points.size() * 3];
    for (size_t pointIndex = 0; pointIndex < global_map_points.size();
         pointIndex++) {
      for (size_t dim = 0; dim < 3; dim++) {
        m_flatPoints[pointIndex * 3 + dim] = global_map_points[pointIndex][dim];
      }
    }

    flann::Matrix<float> dataset(m_flatPoints, global_map_points.size(), 3);

    // Building the index takes some time.

    m_index = new flann::Index<flann::L2<float>>(
        dataset, flann::KDTreeIndexParams(m_nTrees));
    m_index->buildIndex();

    std::cout << "FLANN index created." << std::endl;

    // end of build query for global map
  }

  void findMatches(std::vector<Eigen::Vector3f> local_map_points,
                   ICPPairs& icp_pairs) {
    // TODO: find closest point on the global map using flann

    if (!m_index) {
      std::cout << "FLANN index needs to be build before querying any matches."
                << std::endl;
    }

    icp_pairs.clear();

    // match

    // FLANN requires that all the points be flat. Therefore we copy the points
    // to a separate flat array.
    float* queryPoints = new float[local_map_points.size() * 3];
    for (size_t pointIndex = 0; pointIndex < local_map_points.size();
         pointIndex++) {
      for (size_t dim = 0; dim < 3; dim++) {
        queryPoints[pointIndex * 3 + dim] = local_map_points[pointIndex][dim];
      }
    }

    flann::Matrix<float> query(queryPoints, local_map_points.size(), 3);
    flann::Matrix<int> indices(new int[query.rows * 1], query.rows, 1);
    flann::Matrix<float> distances(new float[query.rows * 1], query.rows, 1);

    // Do a knn search, searching for 1 nearest point and using 16 checks.
    // TODO: best check number ?
    flann::SearchParams searchParams{16};
    searchParams.cores = 0;
    m_index->knnSearch(query, indices, distances, 1, searchParams);

    // Filter the matches.
    const unsigned nMatches = local_map_points.size();
    // std::vector<Match> matches;
    // matches.reserve(nMatches);

    float m_maxDistance = 2;

    for (int i = 0; i < int(nMatches); i++) {
      if (*distances[i] <= m_maxDistance) {
        // matches.push_back(Match{*indices[i], 1.f});
        icp_pairs.push_back(std::make_pair(i, *indices[i]));
      }
    }

    // Release the memory.
    delete[] query.ptr();
    delete[] indices.ptr();
    delete[] distances.ptr();

    // for (int i = 0; i < nMatches; i++)
    //{
    //    std::cout << "Matches: " << matches[i].idx << std::endl;
    //}
  }

 private:
  int m_nTrees;
  // lookup table for global map points
  flann::Index<flann::L2<float>>* m_index;
  float* m_flatPoints;
};

}  // namespace visnav