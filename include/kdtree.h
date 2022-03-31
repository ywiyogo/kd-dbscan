// Copyright (c) 2022 Yongkie Wiyogo
//
// Licensed under the MIT license: https://opensource.org/licenses/MIT
// Permission is granted to use, copy, modify, and redistribute the work.
// Full license information available in the project LICENSE file.
//

#ifndef _KDTREE_H_
#define _KDTREE_H_

#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
/**
 * @brief A KD-Tree is a binary tree which contains nodes. This struc defines a
 * node of the kd-tree.
 *
 */
class KdNode {
 public:
  KdNode()
      : ndata(nullptr),
        data(nullptr),
        direction(0),
        left(nullptr),
        right(nullptr) {}
  KdNode(uint32_t d)
      : ndata(nullptr),
        data(nullptr),
        direction(d),
        left(nullptr),
        right(nullptr) {}
  ~KdNode() {
    if (left != nullptr) delete left;
    if (right != nullptr) delete right;
    delete[] ndata;
    // do not delete node->data
  }
  void print(KdNode **n, std::string indent) {
    if (*n != nullptr) {
      printf("%sNode:\n", indent.c_str());
      if (&(*n)->ndata) {
        printf("  %sData: (%f, %f)\n", indent.c_str(), ndata[0], ndata[1]);
        printf("  %sdirection: %d\n", indent.c_str(), direction);
      }
      indent = indent + "  ";
      if (&(*n)->left) {
        std::printf("%sLeft child: \n", indent.c_str());
        print(&(*n)->left, indent);
      }
      if (&(*n)->right) {
        std::printf("%sRight child: \n", indent.c_str());
        print(&(*n)->right, indent);
      }
      return;
    }
  }
  // A n-dim data value e.g. float[3]
  float *ndata;
  // Data pointer in a vector or list
  void *data;
  // direction of the node in the kd-tree
  uint32_t direction;
  // Left child node
  KdNode *left;
  // Right child node
  KdNode *right;
};

/**
 * @brief Implementation of the implicit kd-tree class
 *        See https://en.wikipedia.org/wiki/Implicit_k-d_tree
 *
 * @tparam T an array-like container which represents a n-dimensional data.
 * It defines the operator[] for iteration.
 */
template <typename T>
class KdTree {
 public:
  /**
   * @brief Construct a new Kd Tree <T> object
   *
   * @param dim dimension of the input data
   */
  KdTree<T>(int dim) : root_(nullptr), dim_(dim), hyp_rectangle_(nullptr) {}

  // destructor
  ~KdTree<T>() {
    if (hyp_rectangle_ != nullptr) {
      FreeRectangle(hyp_rectangle_);
      hyp_rectangle_ = nullptr;
    }
  }

  void print() {
    root_->print(&root_, "");
    if (hyp_rectangle_) {
      if (hyp_rectangle_->lower)
        printf("Rect lower: %f, upper: %f\n", *hyp_rectangle_->lower,
               *hyp_rectangle_->upper);
    }
  }
  /**
   * @brief  A HyperRectangle is defined as two vectors: upper and lower,
   * with each dimension of lower having the minimum value seen in each
   * dimension, and each dimension of higher having the maximum value seen in
   * each dimension.
   */
  struct HyperRectangle {
    int dim;
    float *lower;
    float *upper;
  };

  struct SearchNode {
    KdNode *node;
    float sq_dist;
    struct SearchNode *next;
  };

  // Tree of a nearest nodes
  struct NearestTree {
    struct KdTree<T> *tree;
    struct SearchNode *search_list, *search_iter;
    int size;
  };

  /**
   * @brief Insert a data
   *
   * @param pos
   * @param data
   * @return true
   * @return false
   */
  void InsertData(const float *data_ptr, void *data) {
    InsertNode(&root_, data_ptr, data, 0, dim_);
    if (hyp_rectangle_ == 0) {
      hyp_rectangle_ = CreateRectangle(dim_, data_ptr, data_ptr);
    } else {
      AddDataHypRectangle(hyp_rectangle_, data_ptr);
    }
  }

  /**
   * @brief Add a dataset
   *
   * @param dataset input dataset as vector
   */
  void AddDataset(const std::vector<T> &dataset) {
    std::unique_ptr<float[]> data(new float[dim_]);

    for (size_t idx = 0; idx < dataset.size(); ++idx) {
      for (uint32_t ndim = 0; ndim < dim_; ++ndim) {
        data[ndim] = static_cast<float>(dataset.at(idx)[ndim]);
      }
      InsertData(data.get(), (void *)&dataset.at(idx));
    }
  }

  /**
   * @brief Find nearest nodes from a given data. The result must be deallocated
   * with the Free() if it doesn't need anymore.
   *
   * @param data n-dimentional data
   * @param range search radius
   * @return NearestTree*
   */
  NearestTree *FindNearestNodes(const float *data, float range) {
    int ret;
    NearestTree *nearest_tree = new NearestTree();

    if (!(nearest_tree->search_list = new SearchNode)) {
      delete nearest_tree;
      return 0;
    }
    nearest_tree->search_list->next = 0;
    nearest_tree->tree = this;

    if ((ret = FindNearestAsTree(root_, data, range, nearest_tree->search_list,
                                 0, dim_)) == -1) {
      Free(nearest_tree);
      return 0;
    }
    nearest_tree->size = ret;
    RewindTree(nearest_tree);
    return nearest_tree;
  }

  /**
   * @brief Free the nearest tree after using FindNearestNodes
   *
   * @param set
   */
  void Free(NearestTree *nearest_tree) {
    ClearNearestTree(nearest_tree);
    delete nearest_tree->search_list;
    delete nearest_tree;
  }

  std::vector<T *> SortDataByDistance(const T &from, float range) {
    std::vector<T *> result;
    std::unique_ptr<float[]> temp_data(new float[dim_]);
    // calc the nearest data from the 0-element
    for (uint32_t dim = 0; dim < dim_; ++dim) {
      temp_data[dim] = static_cast<float>(from[dim]);
    }
    KdTree::NearestTree *presults = FindNearestNodes(temp_data.get(), range);

    while (!IsEnd(presults)) {
      T *data = reinterpret_cast<T *>(GetDataPtr(presults, temp_data.get()));
      result.push_back(data);
      /* go to the next entry */
      NextTraversal(presults);
    }
    return result;
  }

  /**
   * @brief Treversing the next iterator in the nearest tree
   *
   * @param nearest_tree input tree
   * @return true if an element found
   * @return false if no more element
   */
  bool NextTraversal(NearestTree *nearest_tree) {
    nearest_tree->search_iter = nearest_tree->search_iter->next;
    return nearest_tree->search_iter != 0;
  }

  /**
   * @brief Get the Data Ptr object
   *
   * @param nearest_tree tree structure of the nearest nodes
   * @param ndata n-dim data result
   * @return void*
   */
  void *GetDataPtr(NearestTree *nearest_tree, float *ndata) {
    if (nearest_tree->search_iter) {
      if (ndata) {
        std::copy(nearest_tree->search_iter->node->ndata,
                  nearest_tree->search_iter->node->ndata + dim_, ndata);
      }
      return nearest_tree->search_iter->node->data;
    }
    return 0;
  }

  /**
   * @brief Check if end node is reached
   *
   * @param nearest_tree
   * @return true if the set iterator reaches the end after the last element
   * @return false if the set iterator doesn't reach the end of the last element
   */
  bool IsEnd(NearestTree *nearest_tree) {
    return nearest_tree->search_iter == 0;
  }

 private:
  /**
   * @brief Insert a new node with n-dimensional data
   *
   * @param node_ptr Pointer of the node pointer
   * @param ndata
   * @param data
   * @param direction
   * @param dim
   */
  void InsertNode(KdNode **node_ptr, const float *ndata, void *data,
                  uint32_t direction, int dim) {
    if (*node_ptr == nullptr) {
      *node_ptr = new KdNode(direction);
      (*node_ptr)->ndata = new float[dim];
      std::copy(ndata, ndata + dim, (*node_ptr)->ndata);
      (*node_ptr)->data = data;
      return;
    }
    uint32_t dir = (*node_ptr)->direction;
    uint32_t next_direction = (dir + 1) % dim;
    if (ndata[dir] < (*node_ptr)->ndata[dir]) {
      return InsertNode(&(*node_ptr)->left, ndata, data, next_direction, dim);
    }
    return InsertNode(&(*node_ptr)->right, ndata, data, next_direction, dim);
  }

  /**
   * @brief find the nearest node
   *
   * @param node input node
   * @param ndata
   * @param range
   * @param list
   * @param ordered
   * @param dim
   * @return int number of neigbors
   */
  int FindNearestAsTree(KdNode *node, const float *ndata, float range,
                        KdTree::SearchNode *snode, int ordered, int dim) {
    float dx = 0.;
    int ret, added_res = 0;

    if (node == nullptr) return 0;

    float sq_dist = 0.;
    for (int i = 0; i < dim; i++) {
      sq_dist += Square(node->ndata[i] - ndata[i]);
    }
    if (sq_dist <= Square(range)) {
      if (InsertSearchNode(snode, node, ordered ? sq_dist : -1.0)) {
        added_res = 1;
      } else
        return -1;
    }

    dx = ndata[node->direction] - node->ndata[node->direction];

    ret = FindNearestAsTree(dx <= 0.0 ? node->left : node->right, ndata, range,
                            snode, ordered, dim);
    if (ret >= 0 && fabs(dx) < range) {
      added_res += ret;
      ret = FindNearestAsTree(dx <= 0.0 ? node->right : node->left, ndata,
                              range, snode, ordered, dim);
    }
    if (ret == -1) {
      return -1;
    }
    added_res += ret;

    return added_res;
  }

  /**
   * @brief Add data to the hyper rectangle
   *
   * @param hrectangle hyper rectangle
   * @param ndata n dimensional data
   */
  void AddDataHypRectangle(HyperRectangle *rect, const float *ndata) {
    for (auto i = 0; i < rect->dim; i++) {
      if (ndata[i] < rect->lower[i]) {
        rect->lower[i] = ndata[i];
      }
      if (ndata[i] > rect->upper[i]) {
        rect->upper[i] = ndata[i];
      }
    }
  }

  /**
   * @brief Calculate square distance
   *
   * @param rect hyper rectangle
   * @param ndata
   * @return float the result of the square distance
   */
  T CalcSquareDistance(HyperRectangle *rect, const float *ndata) {
    T result = 0;

    for (auto i = 0; i < rect->dim; i++) {
      if (ndata[i] < rect->lower[i]) {
        result += Square(rect->lower[i] - ndata[i]);
      } else if (ndata[i] > rect->upper[i]) {
        result += Square(rect->upper[i] - ndata[i]);
      }
    }

    return result;
  }

  /**
   * @brief Create a Hyper Rectangle object
   *
   * @param dim_
   * @param min_data
   * @param max_data
   * @return HyperRectangle*
   */
  HyperRectangle *CreateRectangle(int dim_, const float *min_data,
                                  const float *max_data) {
    size_t size = dim_ * sizeof(float);
    KdTree::HyperRectangle *rect = 0;

    if (!(rect = new KdTree::HyperRectangle)) {
      return 0;
    }

    rect->dim = dim_;
    if (!(rect->lower = new float[size])) {
      delete rect;
      return 0;
    }
    if (!(rect->upper = new float[size])) {
      delete[] rect->lower;
      delete rect;
      return 0;
    }
    std::copy(min_data, min_data + size, rect->lower);
    std::copy(max_data, max_data + size, rect->upper);

    return rect;
  }

  /**
   * @brief Free the hyper rectangle
   *
   * @param hrectangle input
   */
  void FreeRectangle(HyperRectangle *hrectangle) {
    delete[] hrectangle->lower;
    delete[] hrectangle->upper;
    delete hrectangle;
  }

  /**
   * @brief Insert a search node
   *
   * @param search_node
   * @param node
   * @param sq_dist
   * @return bool true if a node found
   */
  bool InsertSearchNode(SearchNode *search_node, KdNode *node, float sq_dist) {
    KdTree::SearchNode *s_node;

    if (!(s_node = new KdTree::SearchNode)) {
      return false;
    }
    s_node->node = node;
    s_node->sq_dist = sq_dist;

    if (sq_dist >= 0.0) {
      while (search_node->next && search_node->next->sq_dist < sq_dist) {
        search_node = search_node->next;
      }
    }
    s_node->next = search_node->next;
    search_node->next = s_node;
    return true;
  }

  /**
   * @brief Rewind a given nearest tree
   *
   * @param tree
   */
  void RewindTree(NearestTree *tree) {
    tree->search_iter = tree->search_list->next;
  }

  /**
   * @brief Clear nearest tree
   *
   * @param tree
   */
  void ClearNearestTree(NearestTree *tree) {
    SearchNode *tmp, *node = tree->search_list->next;

    while (node) {
      tmp = node;
      node = node->next;
      delete tmp;
    }
    tree->search_list->next = 0;
  }

  float Square(float x) { return static_cast<float>(x * x); }

  // Root node
  KdNode *root_;

  // Dimension of data
  uint32_t dim_;

  HyperRectangle *hyp_rectangle_;
};

#endif /* _KDTREE_H_ */
