// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#include <stdio.h>

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
  std::vector<Data2D> data{
      // clang-format off
      Data2D{1037, 1046},
      Data2D{1036, 1076},
      Data2D{1035, 1074},
      Data2D{1035, 1075},
      Data2D{1034, 1075},
      Data2D{1034, 1077},
      Data2D{1033, 1075},
      Data2D{1033, 1077},
      Data2D{1032, 1075},
      Data2D{1032, 1073},
      Data2D{1093, 1099},
      Data2D{103, 1100},
      Data2D{104, 1101},
      Data2D{105, 1101},
      Data2D{107, 1103},
      Data2D{102, 1104},
      Data2D{105, 1105},
      Data2D{102, 1105},
      // clang-format on
  };
  float cluster_range = 4.;
  uint32_t min_cells = 3;
  uint32_t kDim = 2;
  auto dbscan = DBSCAN<Data2D>(cluster_range, min_cells);

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
  Data2D center{0., 0.};
  printf("Get sorted data by distance from (%f, %f)\n", (&center)[0][0], (&center)[0][1]);
  KdTree<Data2D> kdtree(2);
  std::vector<Data2D> dataset{Data2D{7.0, 0.0}, Data2D{5.0, 0.0}, Data2D{3.0, 0.0}, Data2D{1.0, 0.0}};
  float range = 6.9;
  kdtree.AddDataset(dataset);
  std::vector<const Data2D *> sorted_data = kdtree.SortDataByDistance(center, range);

  for (auto data : sorted_data) {
    printf("Data: %f, %f\n", data[0][0], data[0][1]);
  }
  return 0;
}