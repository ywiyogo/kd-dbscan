// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//
#include "dbscan.h"

#include "gtest/gtest.h"

/**
 * @brief Helper function
 *
 * @param container container class
 * @param element element to be search in the container
 * @return true if the element in the contaienr exists
 * @return false if the element in the contaienr doesn't exist
 */
template <class C, typename T>
bool contains(C&& container, T element) {
  return find(begin(container), end(container), element) != end(container);
}

/**
 * @brief Test the DBSCAN clustering with an std::array container
 *
 */
TEST(DBSCAN, TestClusteringArray) {
  const size_t kDim = 2;
  std::vector<std::array<int32_t, kDim>> data{
      // clang-format off
      {427, 1046},
      {436, 1076},
      {435, 1074},
      {435, 1075},
      {434, 1075},
      {434, 1077},
      {933, 4075},
      {933, 4077},
      {932, 4075},
      {932, 4073},
      {934, 4075},
      {103, 1100},
      {104, 1101},
      {105, 1101},
      {107, 1103},
      {104, 1104},
      {105, 1105},
      {100, 1110},
      // clang-format on
  };

  float cluster_eps = 3.;
  uint32_t min_data = 2;
  DBSCAN<std::array<int32_t, kDim>> dbscan(cluster_eps, min_data);
  dbscan.Execute(data, kDim);
  size_t exp_cluster_size = 3;
  size_t exp_max_outliers = 2;
  EXPECT_EQ(dbscan.clusters.size(), exp_cluster_size);
  EXPECT_LE(dbscan.outliers.size(), exp_max_outliers);

  // check outliers at index 0 and 17
  EXPECT_EQ(contains(dbscan.outliers, 0), true);
  EXPECT_EQ(contains(dbscan.outliers, 17), true);
}

struct Data2D {
  uint x;
  uint y;
  constexpr Data2D() : x(0), y(0) {}
  constexpr Data2D(int x, int y) : x(x), y(y) {}
  bool inline operator<(const struct Data2D& other) const { return ((x < other.x) && (y < other.y)); }
  int inline operator[](const size_t i) const {
    assert(i < 2);
    if (i == 0) {
      return x;
    } else if (i == 1) {
      return y;
    } else {
      /* the above assertion must throw an exception */
      return -1;
    }
  }
  constexpr size_t size() const { return 2; }
};

/**
 * @brief Test the DBSCAN clustering with a custom Data2D container
 *
 */
TEST(DBSCAN, TestClusteringCell2D) {
  std::vector<Data2D> data{
      // clang-format off
      Data2D{1037, 1046},
      Data2D(1036, 1076),
      Data2D(1035, 1074),
      Data2D(1035, 1075),
      Data2D(1034, 1075),
      Data2D(1034, 1077),
      Data2D(1033, 1075),
      Data2D(1033, 1077),
      Data2D(1032, 1075),
      Data2D(1032, 1073),
      Data2D(1093, 1099),
      Data2D(2103, 1100),
      Data2D(2104, 1101),
      Data2D(2105, 1101),
      Data2D(2107, 1103),
      Data2D(2102, 1104),
      Data2D(2105, 1105),
      Data2D(2102, 1105),
      // clang-format on
  };
  float cluster_eps = 4.;
  uint32_t min_cells = 3;
  const size_t kDim = 2;
  DBSCAN<Data2D> dbscan(cluster_eps, min_cells);

  dbscan.Execute(data, kDim);
  size_t exp_cluster_size = 2;
  size_t exp_max_outliers = 4;
  EXPECT_EQ(dbscan.clusters.size(), exp_cluster_size);
  EXPECT_LE(dbscan.outliers.size(), exp_max_outliers);
}
