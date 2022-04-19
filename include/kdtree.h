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
    if (root_)
      delete root_;
    if (hyp_rectangle_ != nullptr) {
      delete hyp_rectangle_;
      hyp_rectangle_ = nullptr;
    }
  }

  void print() {
    root_->print(&root_, "");
    if (hyp_rectangle_) {
      if (hyp_rectangle_->lower)
        printf("Rect lower: %f, upper: %f\n", *hyp_rectangle_->lower, *hyp_rectangle_->upper);
    }
  }

  /**
   * @brief A KD-Tree is a binary tree which contains nodes. This class defines a
   * node of the kd-tree.
   *
   */
  class KdNode {
   public:
    KdNode(const T &pdata, const uint32_t pos)
        : data(pdata), data_pos(pos), direction(0), left(nullptr), right(nullptr) {}
    explicit KdNode(const T &pdata, const uint32_t pos, uint32_t d)
        : data(pdata), data_pos(pos), direction(d), left(nullptr), right(nullptr) {}
    ~KdNode() {
      if (left != nullptr)
        delete left;
      if (right != nullptr)
        delete right;
      // do not delete node->data
    }
    void print(const KdNode &n, std::string indent) {
      printf("%sNode:\n  %sData: ", indent.c_str());
      for (size_t i = 0; i < n.data.size(); i++) {
        printf("(%f, %f)\n", indent.c_str(), n.data[i]);
      }
      printf("  %sdirection: %d\n", indent.c_str(), direction);

      indent = indent + "  ";
      if (n.left) {
        printf("%sLeft child: \n", indent.c_str());
        print(n.left, indent);
      }
      if (n.right) {
        printf("%sRight child: \n", indent.c_str());
        print(n.right, indent);
      }
      return;
    }
    // Reference of a single data structure
    const T &data;

    uint32_t data_pos;
    // direction of the node in the kd-tree
    uint32_t direction;
    // Left child node
    KdNode *left;
    // Right child node
    KdNode *right;
  };

  /**
   * @brief  A HyperRectangle is defined as two vectors: upper and lower,
   * with each dimension of lower having the minimum value seen in each
   * dimension, and each dimension of higher having the maximum value seen in
   * each dimension.
   */
  struct HyperRectangle {
    int dim;
    T lower;
    T upper;
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
   * @param data_ptr pointer of the data value
   * @param data pointer of the data structure in the vector
   */
  void InsertData(const size_t pos, const T &data) {
    InsertNode(&root_, pos, data, 0, dim_);
    if (hyp_rectangle_ == 0) {
      // initialize lower and uper with the first incoming data
      hyp_rectangle_ = CreateRectangle(dim_, data, data);
    } else {
      AddDataHypRectangle(hyp_rectangle_, data);
    }
  }

  /**
   * @brief Add a dataset
   *
   * @param dataset input dataset as vector
   */
  void AddDataset(const std::vector<T> &dataset) {
    for (size_t idx = 0; idx < dataset.size(); ++idx) {
      InsertData(idx, dataset.at(idx));
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
  NearestTree FindNearestNodes(const T &ndata, float range) {
    int ret;
    NearestTree nearest_tree;

    nearest_tree.search_list = new SearchNode();

    nearest_tree.search_list->next = 0;
    nearest_tree.tree = this;

    if ((ret = FindNearestAsTree(root_, ndata, range, nearest_tree.search_list, 0, dim_)) == -1) {
      return nearest_tree;
    }
    nearest_tree.size = ret;
    RewindTree(&nearest_tree);
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

  std::vector<T> SortDataByDistance(const T &from, float range) {
    std::vector<T> result;
    KdTree::NearestTree presults = FindNearestNodes(from, range);

    while (!IsEnd(presults)) {
      T data;
      GetData(presults, &data);
      if (from != data) {
        result.push_back(data);
      }

      /* go to the next entry */
      NextTraversal(&presults);
    }
    sort(result.begin(), result.end());
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
  void GetData(const NearestTree &nearest_tree, T *result) const {
    if (nearest_tree.search_iter)
      *result = nearest_tree.search_iter->node->data;
  }

  int32_t GetDataPos(const NearestTree &nearest_tree) const {
    if (nearest_tree.search_iter)
      return nearest_tree.search_iter->node->data_pos;
    else
      return -1;
  }
  /**
   * @brief Check if end node is reached
   *
   * @param nearest_tree
   * @return true if the set iterator reaches the end after the last element
   * @return false if the set iterator doesn't reach the end of the last element
   */
  bool IsEnd(const NearestTree &nearest_tree) const { return nearest_tree.search_iter == 0; }

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
  void InsertNode(KdNode **node_ptr, const size_t pos, const T &ndata, uint32_t direction, int dim) {
    if (*node_ptr == nullptr) {
      *node_ptr = new KdNode(ndata, pos, direction);
      return;
    }
    uint32_t dir = (*node_ptr)->direction;
    uint32_t next_direction = (dir + 1) % dim;
    if (ndata[dir] < (*node_ptr)->data[dir]) {
      return InsertNode(&(*node_ptr)->left, pos, ndata, next_direction, dim);
    }
    return InsertNode(&(*node_ptr)->right, pos, ndata, next_direction, dim);
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
  int FindNearestAsTree(KdNode *node, const T &ndata, float range, KdTree::SearchNode *snode, int ordered, int dim) {
    float dx = 0.;
    int ret, added_res = 0;

    if (node == nullptr)
      return 0;

    float sq_dist = 0.;
    for (size_t i = 0; i < ndata.size(); i++) {
      float diff = static_cast<float>(node->data[i]) - static_cast<float>(ndata[i]);
      sq_dist += pow(diff, 2);
    }

    if (sqrt(sq_dist) <= sqrt(pow(range, 2))) {
      if (InsertSearchNode(snode, node, ordered ? sq_dist : -1.0)) {
        added_res = 1;
      } else {
        return -1;
      }
    }

    dx = ndata[node->direction] - node->data[node->direction];

    ret = FindNearestAsTree(dx <= 0.0 ? node->left : node->right, ndata, range, snode, ordered, dim);
    if (ret >= 0 && fabs(dx) < range) {
      added_res += ret;
      ret = FindNearestAsTree(dx <= 0.0 ? node->right : node->left, ndata, range, snode, ordered, dim);
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
  void AddDataHypRectangle(HyperRectangle *rect, const T &ndata) {
    if (ndata < rect->lower) {
      rect->lower = ndata;
    } else {
      rect->upper = ndata;
    }
  }

  /**
   * @brief Calculate square distance
   *
   * @param rect hyper rectangle
   * @param ndata
   * @return float the result of the square distance
   */
  T CalcSquareDistance(HyperRectangle *rect, const T &ndata) {
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
  HyperRectangle *CreateRectangle(int dim, const T &low_data, const T &up_data) {
    KdTree::HyperRectangle *rect = 0;

    if (!(rect = new KdTree::HyperRectangle)) {
      return 0;
    }

    rect->dim = dim;
    rect->lower = low_data;

    rect->upper = up_data;

    return rect;
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
  void RewindTree(NearestTree *tree) { tree->search_iter = tree->search_list->next; }

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

  float Square(float x) { return x * x; }

  // Root node
  KdNode *root_;

  // Dimension of data
  uint32_t dim_;

  HyperRectangle *hyp_rectangle_;
};

#endif  // _KDTREE_H_
