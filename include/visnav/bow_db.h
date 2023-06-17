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

#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>

#include <visnav/common_types.h>
#include <visnav/serialization.h>

namespace visnav {

class BowDatabase {
 public:
  BowDatabase() {}

  inline void insert(const FrameCamId& fcid, const BowVector& bow_vector) {
    // TODO SHEET 3: add a bow_vector that corresponds to frame fcid to the
    // inverted index. You can assume the image hasn't been added before.
    // using BowDBInverseIndexConcurrent = tbb::concurrent_unordered_map<
    // WordId, tbb::concurrent_vector<std::pair<FrameCamId, WordValue>>>;

    for (auto bow : bow_vector) {
      inverted_index.insert(std::make_pair(
          bow.first,
          tbb::concurrent_vector<std::pair<FrameCamId, WordValue>>()));
      inverted_index[bow.first].push_back(std::make_pair(fcid, bow.second));
    }
  }

  inline void query(const BowVector& bow_vector, size_t num_results,
                    BowQueryResult& results) const {
    // TODO SHEET 3: find num_results closest matches to the bow_vector in the
    // inverted index. Hint: for good query performance use std::unordered_map
    // to accumulate scores and std::partial_sort for getting the closest
    // results. You should use L1 difference as the distance measure. You can
    // assume that BoW descripors are L1 normalized.

    std::unordered_map<FrameCamId, double>
        images_dist_map;  // distance map for each FrameCamId

    // BowDBInverseIndexConcurrent = tbb::concurrent_unordered_map<
    // WordId, tbb::concurrent_vector<std::pair<FrameCamId, WordValue>>>;

    // for each pair<visnav::WordId, visnav::WordValue>
    for (auto bow : bow_vector) {
      // for each FrameCamId in the word
      for (auto frameid_weight : inverted_index.at(bow.first)) {
        if (images_dist_map.find(frameid_weight.first) ==
            images_dist_map.end()) {
          images_dist_map[frameid_weight.first] =
              2 + std::abs(bow.second - frameid_weight.second) -
              std::abs(bow.second) - std::abs(frameid_weight.second);
          std::cout << " new key" << std::endl;
        } else {
          images_dist_map[frameid_weight.first] +=
              std::abs(bow.second - frameid_weight.second) -
              std::abs(bow.second) - std::abs(frameid_weight.second);
          std::cout << " existing key" << std::endl;
        }
      }
    }

    std::cout << " map size " << images_dist_map.size() << std::endl;
    int i = 0;
    for (auto image_dist : images_dist_map) {
      std::cout << "image_dist " << i++ << " " << image_dist.first << " "
                << image_dist.second << std::endl;
    }

    std::vector<std::pair<visnav::FrameCamId, double>> images_dist_vec;
    for (auto image_dist : images_dist_map) {
      images_dist_vec.push_back(std::pair<visnav::FrameCamId, double>(
          image_dist.first, image_dist.second));
    }

    std::sort(images_dist_vec.begin(), images_dist_vec.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    size_t j = 0;
    for (auto image_dist : images_dist_vec) {
      std::cout << "image_dist " << j << " " << image_dist.first << " "
                << image_dist.second << std::endl;
      if (j < num_results) {
        results.push_back(image_dist);
      }
      j++;
    }

    // std::partial_sort(images_dist_map.begin(), images_dist_map.begin() + 1,
    // images_dist_map.end(), comparison_function);
  }

  void clear() { inverted_index.clear(); }

  void save(const std::string& out_path) {
    BowDBInverseIndex state;
    for (const auto& kv : inverted_index) {
      for (const auto& a : kv.second) {
        state[kv.first].emplace_back(a);
      }
    }
    std::ofstream os;
    os.open(out_path, std::ios::binary);
    cereal::JSONOutputArchive archive(os);
    archive(state);
  }

  void load(const std::string& in_path) {
    BowDBInverseIndex inverseIndexLoaded;
    {
      std::ifstream os(in_path, std::ios::binary);
      cereal::JSONInputArchive archive(os);
      archive(inverseIndexLoaded);
    }
    for (const auto& kv : inverseIndexLoaded) {
      for (const auto& a : kv.second) {
        inverted_index[kv.first].emplace_back(a);
      }
    }
  }

  const BowDBInverseIndexConcurrent& getInvertedIndex() {
    return inverted_index;
  }

 protected:
  BowDBInverseIndexConcurrent inverted_index;
};

}  // namespace visnav
