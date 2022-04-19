// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#ifndef __DBSCAN_H__
#define __DBSCAN_H__

#include <assert.h>
#include <stdio.h>

#include <functional>
#include <memory>
#include <queue>
#include <set>
#include <vector>

#include "kdtree.h"

/**
 * @brief Implementation of the KD-DBSCAN
 *
 * @tparam T an array-like container which represents a n-dimensional data.
 * It defines the operator[] for iteration.
 */
template <typename T>
class DBSCAN {
 public:
  /**
   * @brief Construct a new DBSCAN object
   * @param eps epsilon or radian of a cluster
   * @param min_pts minimal number of points in epsilon radian
   */
  DBSCAN(const float eps, const uint32_t min_pts) : kdtree_(nullptr), epsilon_(eps), min_pts_(min_pts), datadim_(0) {
    assert(min_pts_ > 1 && epsilon_ > 0.f);
  }

  /**
   * @brief Destroy the DBSCAN object
   *
   */
  ~DBSCAN() {}

  /**
   * @brief Execute the DBSCAN clustering
   *
   * @param dataset dataset input
   * @param dim the dimension of a single data
   * @return true successful run
   * @return false unsuccessful run
   */
  bool Execute(const std::vector<T> &dataset, const uint32_t dim) {
    assert(dataset.size() > 0 || dim > 0);

    visited_ = std::vector<bool>(dataset.size(), false);
    assigned_ = std::vector<bool>(dataset.size(), false);
    clusters.clear();
    outliers.clear();
    datadim_ = dim;

    kdtree_ = new KdTree<T>(datadim_);
    kdtree_->AddDataset(dataset);

    // Checking the cluster on each index
    for (size_t idx = 0; idx < dataset.size(); ++idx) {
      borderset_.clear();
      if (!visited_[idx]) {
        visited_[idx] = true;
        const std::vector<uint32_t> neighbors = FindNeigbors(dataset, idx);
        if (neighbors.size() < min_pts_) {
          continue;
        } else {
          uint32_t cluster_idx = (uint32_t)clusters.size();
          clusters.push_back(std::vector<uint32_t>());
          AddDataIndex(idx);
          AddToCluster(idx, cluster_idx);
          ExpandCluster(dataset, cluster_idx, neighbors);
          // recheck the cluster items
          if (clusters.at(cluster_idx).size() < min_pts_) {
            clusters.pop_back();
          }
        }
      }
    }
    // Collect the outliers
    for (size_t idx = 0; idx < dataset.size(); ++idx) {
      if (!assigned_[idx]) {
        outliers.push_back(idx);
      }
    }

    delete kdtree_;
    return true;
  }

  /**
   * @brief cluster result
   *
   */
  std::vector<std::vector<uint32_t>> clusters;

  /**
   * @brief outliers result
   *
   */
  std::vector<uint32_t> outliers;

 private:
  /**
   * @brief find the neigbors indices
   *
   * @param dataset input
   * @param idx data index of the center
   * @return std::vector<uint32_t> indices of the neigbours
   */
  std::vector<uint32_t> FindNeigbors(const std::vector<T> &dataset, const uint32_t idx) const {
    std::vector<uint32_t> neighbors;
    typename KdTree<T>::NearestTree presults = kdtree_->FindNearestNodes(dataset[idx], epsilon_);

    while (!kdtree_->IsEnd(presults)) {
      int32_t data_pos = kdtree_->GetDataPos(presults);

      if (data_pos > 0 && idx != static_cast<uint32_t>(data_pos)) {
        neighbors.push_back(data_pos);
      }

      /* go to the next entry */
      kdtree_->NextTraversal(&presults);
    }

    return neighbors;
  }
  /**
   * @brief Add a data index to a cluster
   *
   * @param idx input index
   * @param cluster_idx cluster index
   */
  void AddToCluster(const uint32_t idx, const uint32_t cluster_idx) {
    clusters[cluster_idx].push_back(idx);
    assigned_[idx] = true;
  }
  /**
   * @brief Expand a cluster index
   *
   * @param dataset input data
   * @param cluster_idx cluster index
   * @param neighbors vector of neighbours
   */
  void ExpandCluster(const std::vector<T> &dataset, const uint32_t cluster_idx,
                     const std::vector<uint32_t> &neighbors) {
    std::queue<uint32_t> q_border;
    for (auto idx : neighbors) q_border.push(idx);
    AddDataIndices(neighbors);

    while (q_border.size() > 0) {
      const uint32_t idx = q_border.front();
      q_border.pop();

      if (!visited_[idx]) {
        // Mark if not been visited yet
        visited_[idx] = true;
        const std::vector<uint32_t> nb_indices = FindNeigbors(dataset, idx);

        // Expanding the neighbors
        if (nb_indices.size() >= min_pts_) {
          AddToCluster(idx, cluster_idx);
          for (uint32_t pidnid : nb_indices) {
            if (!IsInBorderSet(pidnid)) {
              q_border.push(pidnid);
              AddDataIndex(pidnid);
            }
          }
        }
      }
    }
  }

  /**
   * @brief Add data index to the borderset
   *
   * @param idx input of a data index
   */
  void AddDataIndex(const uint32_t idx) { borderset_.insert(idx); }

  /**
   * @brief Add a vector of indices
   *
   * @param indices input indices
   */
  void AddDataIndices(const std::vector<uint32_t> &indices) {
    for (uint32_t idx : indices) this->borderset_.insert(idx);
  }

  /**
   * @brief check if a data index in the borderset
   *
   * @param idx
   * @return true
   * @return false
   */
  bool IsInBorderSet(const uint32_t idx) const { return this->borderset_.end() != this->borderset_.find(idx); }
  // kd-tree
  KdTree<T> *kdtree_;
  // radius to the neigbour noded in a cluster
  float epsilon_;
  // minimum data or node in a cluste
  uint32_t min_pts_;
  // dimension of the dataset
  uint32_t datadim_;
  // list of flags maps to the dataset
  std::vector<bool> visited_;
  std::vector<bool> assigned_;
  std::set<uint32_t> borderset_;
};

#endif  // __DBSCAN_H__
