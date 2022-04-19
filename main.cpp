// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#include <stdio.h>

#include <array>
#include <vector>

#include "include/dbscan.h"

struct Data2D {
  float data[2];
  float operator[](int idx) const { return data[idx]; }
};

struct Data3D {
  float data[3];
  float operator[](int idx) const { return data[idx]; }
};

int main() {
  std::vector<std::array<int32_t, 2>> data{
      // clang-format off
      {1037, 1046},
      {1036, 1076},
      {1035, 1074},
      {1035, 1075},
      {1034, 1075},
      {1034, 1077},
      {1033, 1075},
      {1033, 1077},
      {1032, 1075},
      {1032, 1073},
      {1093, 1099},
      {103, 1100},
      {104, 1101},
      {105, 1101},
      {107, 1103},
      {102, 1104},
      {105, 1105},
      {102, 1105},
      // clang-format on
  };
  float cluster_eps = 4.;
  uint32_t min_cells = 3;
  const size_t kDim = 2;
  auto dbscan = DBSCAN<std::array<int32_t, kDim>>(cluster_eps, min_cells);

  dbscan.Execute(data, kDim);

  printf("Number of cluster %ld, noise: %ld\n", dbscan.clusters.size(), dbscan.outliers.size());
  for (int i = 0; i < dbscan.clusters.size(); i++) {
    printf("Cluster %d\n", i);
    for (auto j : dbscan.clusters[i]) {
      printf("    Element of data[%d] \n", j);
    }
  }
  for (int i = 0; i < dbscan.outliers.size(); i++) {
    printf("Outliers: data[%d]\n", dbscan.outliers[i]);
  }

  printf("\n-----------------------------------------\n");
  std::array<uint32_t, kDim> center{0, 0};
  float range = 9;
  printf("Get sorted data by distance from (%d, %d) with range %.2f\n", (&center)[0][0], (&center)[0][1], range);
  KdTree<std::array<uint32_t, kDim>> kdtree(2);
  std::vector<std::array<uint32_t, kDim>> dataset{{1, 0}, {4, 0}, {7, 0}, {9, 0}, {10, 10}};

  kdtree.AddDataset(dataset);

  std::vector<std::array<uint32_t, kDim>> sorted_data = kdtree.SortDataByDistance(center, range);
  printf("Size of result :%ld\n", sorted_data.size());
  for (auto data : sorted_data) {
    printf("Sorted data: (%d, %d)\n", data[0], data[1]);
  }
  return 0;
}