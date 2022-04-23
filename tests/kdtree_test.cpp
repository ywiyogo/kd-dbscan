// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//
#include "kdtree.h"

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
 * @brief Test the function SortDataByDistance in the KdTree with range of 0
 *
 */
TEST(KdTree, TestSortDataByDistance0) {
  const size_t kDim = 2;
  std::array<uint32_t, kDim> center{0, 0};
  float range = 0.;
  KdTree<std::array<uint32_t, kDim>> kdtree(kDim);
  std::vector<std::array<uint32_t, kDim>> dataset{{1, 0}, {4, 0}, {7, 0}, {9, 0}, {10, 0}};

  kdtree.AddDataset(dataset);
  std::vector<std::array<uint32_t, kDim>> sorted_data = kdtree.SortDataByDistance(center, range);

  size_t expected_size = 0;
  EXPECT_EQ(sorted_data.size(), expected_size);
}
/**
 * @brief Test the function SortDataByDistance in the KdTree with range of 9
 *
 */
TEST(KdTree, TestSortDataByDistance9) {
  const size_t kDim = 2;
  std::array<uint32_t, kDim> center{0, 0};
  float range = 9.;
  KdTree<std::array<uint32_t, kDim>> kdtree(kDim);
  std::vector<std::array<uint32_t, kDim>> dataset{{1, 0}, {4, 0}, {7, 0}, {9, 0}, {10, 0}};

  kdtree.AddDataset(dataset);

  std::vector<std::array<uint32_t, kDim>> sorted_data = kdtree.SortDataByDistance(center, range);

  size_t expected_size = 4;
  EXPECT_EQ(sorted_data.size(), expected_size);
  // check for the ordered result
  for (size_t i = 0; i < sorted_data.size(); i++) {
    EXPECT_EQ(sorted_data[i], dataset[i]);
  }
  for (size_t i = 0; i < sorted_data.size() - 1; i++) {
    EXPECT_EQ(contains(sorted_data, dataset[i]), true);
  }
  EXPECT_EQ(contains(sorted_data, dataset[4]), false);

  range = 10.;
  sorted_data = kdtree.SortDataByDistance(center, range);
  EXPECT_EQ(contains(sorted_data, dataset[4]), true);
}

/**
 * @brief Test the function SortDataByDistance in the KdTree with range of 11
 *
 */
TEST(KdTree, TestSortDataByDistance10) {
  const size_t kDim = 2;
  std::array<uint32_t, kDim> center{0, 0};
  float range = 11.;
  KdTree<std::array<uint32_t, kDim>> kdtree(kDim);
  std::vector<std::array<uint32_t, kDim>> dataset{{1, 0}, {4, 0}, {7, 0}, {9, 0}, {10, 0}};

  kdtree.AddDataset(dataset);

  std::vector<std::array<uint32_t, kDim>> sorted_data = kdtree.SortDataByDistance(center, range);
  // check for the ordered result
  for (size_t i = 0; i < sorted_data.size(); i++) {
    EXPECT_EQ(sorted_data[i], dataset[i]);
  }
  for (size_t i = 0; i < sorted_data.size(); i++) {
    EXPECT_EQ(contains(sorted_data, dataset[i]), true);
  }
}
