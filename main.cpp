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
      Data2D{1080, 940},
      Data2D{1086, 939},
      Data2D{1060, 900},
      Data2D{1104, 943},
      Data2D{1105, 942},
      Data2D{1105, 940},
      Data2D{1149, 1046},
      Data2D{1149, 1051},
      Data2D{1149, 1059},
      Data2D{1154, 1054},
      Data2D{1155, 1049},
      Data2D{1155, 1055},
      Data2D{1156, 1050},
      Data2D{1157, 1051}
      // clang-format on
  };

  auto dbscan = DBSCAN<Data2D>(30.f, 3);

  dbscan.Execute(data, 2);

  printf("Number of cluster %ld, noise: %ld\n", dbscan.clusters.size(),
         dbscan.outliers.size());
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
  printf("Get sorted data by distance from (%f, %f)\n", (&center)[0][0],
         (&center)[0][1]);
  KdTree<Data2D> kdtree(2);
  std::vector<Data2D> dataset{Data2D{7.0, 0.0}, Data2D{5.0, 0.0},
                              Data2D{3.0, 0.0}, Data2D{1.0, 0.0}};
  float range = 6.9;
  kdtree.AddDataset(dataset);
  std::vector<Data2D *> sorted_data = kdtree.SortDataByDistance(center, range);

  for (auto data : sorted_data) {
    printf("Data: %f, %f\n", data[0][0], data[0][1]);
  }
  return 0;
}