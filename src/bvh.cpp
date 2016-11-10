#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 { namespace StaticScene {

  void BVHAccel::constructHelper(BVHNode* node, std::vector<Primitive *> &_primitives, size_t max_leaf_size) {
    if(node->range <= max_leaf_size) {
      // Node size is less than max_leaf_size, so it is valid
      return;
    }

    double min, max;
    Vector3D min_bb = node->bb.min;
    Vector3D max_bb = node->bb.max;

    BBox split_A;
    BBox split_B;
    size_t xyz_split = -1; // axis to split along
    size_t min_split = 0; // index to split on
    double split_val = 0.0; // coordinate value of split point
    double min_cost = node->range; // default minimum cost of not splitting = (Sn / Sn) * N = N
    size_t min_left = 0; // number of primitives in split left side
    size_t min_right = 0; // number of primitives in split right side

    // Iterate through axes, x = 0, y = 1, z = 2
    for(int i = 0; i < 3; i++) {
      min = min_bb[i];
      max = max_bb[i];

      // Initialize 8 empty buckets per axis, and individual primitive vectors to reorder primitive list
      double eighth = (max - min) / 8.0;
      std::vector<BVHNode> buckets (8, BVHNode(BBox(), -1, 0));

      // Fill buckets with primitives
      for(size_t p_idx = node->start; p_idx < node->start + node->range; p_idx++) {
        // Get primitive and its bbox
        Primitive* p = _primitives[p_idx];
        BBox box = p->get_bbox();

        // Determine primitive's bucket for that axis
        Vector3D centroid = box.centroid();
        double cent = centroid[i];
        int b_idx = (int) round((cent - min) / eighth);
        if(b_idx < 0) b_idx = 0;
        if(b_idx > 7) b_idx = 7;

        // Add primitive to bucket
        buckets[b_idx].bb.expand(box);
        buckets[b_idx].range += 1;
      }


      // Evaluate SAH for all split point
      double Sn = node->bb.surface_area();
      for(size_t split = 1; split < 8; split++) {
        // Find surface area and primitive count of left side
        BBox l_box = BBox();
        size_t Na = 0;
        for(size_t l = 0; l < split; l++) {
          l_box.expand(buckets[l].bb);
          Na += buckets[l].range;
        }
        double Sa = l_box.surface_area();

        // Find surface area and primitive count of right side
        BBox r_box = BBox();
        size_t Nb = 0;
        for(size_t r = split; r < 8; r++) {
          r_box.expand(buckets[r].bb);
          Nb += buckets[r].range;
        }
        double Sb = r_box.surface_area();
        if(Sb != 0) {
        }

        // Calculate SAH and update min_cost and other fields if needed
        double sah = (Sa / Sn) * Na + (Sb / Sn) * Nb;

        if(sah < min_cost) {
          split_A = l_box;
          split_B = r_box;
          xyz_split = i;
          split_val = min + split * eighth;
          min_cost = sah;
          min_split = split;
          min_left = Na;
          min_right = Nb;
        }
      }
    }

    // Sort primitives list from start to range by the split axis

    std::sort(_primitives.begin() + node->start, _primitives.begin() + node->start + node->range,
              [xyz_split](Primitive* a, Primitive* b) {
                return a->get_bbox().centroid()[xyz_split] < b->get_bbox().centroid()[xyz_split];
              });
    this->primitives = _primitives;

    // Create new nodes and recurse

    size_t l_start = node->start;
    size_t l_range = min_left;
    size_t r_start = l_start + l_range;
    size_t r_range = min_right;

    BVHNode* left = new BVHNode(split_A, l_start, l_range);
    BVHNode* right = new BVHNode(split_B, r_start, r_range);
    node->l = left;
    node->r = right;

    constructHelper(left, _primitives, max_leaf_size);
    constructHelper(right, _primitives, max_leaf_size);
  }

  BVHAccel::BVHAccel(std::vector<Primitive *> &_primitives,
      size_t max_leaf_size) {

    this->primitives = _primitives;

    // TODO:
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code build a BVH aggregate with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    BBox bb;
    for (size_t i = 0; i < primitives.size(); ++i) {
      bb.expand(primitives[i]->get_bbox());
    }

    // cout << "surface area = " << bb.surface_area() << "\n";
    // cout << "min = " << bb.min << ", max = " << bb.max << "\n";

    root = new BVHNode(bb, 0, primitives.size());

    // cout << "\nmax leaf size = " << max_leaf_size << "\n";
    cout << "\n";
    constructHelper(root, _primitives, max_leaf_size);

  }

  void BVHAccel::del_tree(BVHNode* node) {
    if(node->l != NULL) {
      del_tree(node->l);
    }
    if(node->r != NULL) {
      del_tree(node->r);
    }

    delete node;
  }

  BVHAccel::~BVHAccel() {

    // TODO:
    // Implement a proper destructor for your BVH accelerator aggregate
    del_tree(root);
  }

  BBox BVHAccel::get_bbox() const {
    return root->bb;
  }

  bool BVHAccel::intersect(const Ray &ray) const {

    // TODO:
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    std::stack<BVHNode *> bvh_stack;
    bvh_stack.push(root);

    BVHNode* node;
    BVHNode* left;
    BVHNode* right;

    double t0;
    double t1;
    if(!root->bb.intersect(ray, t0, t1)) {
      return false;
    }

    bool hit = false;
    while(!bvh_stack.empty()) {
      node = bvh_stack.top();
      bvh_stack.pop();
      left = node->l;
      right = node->r;

      if(node->isLeaf()) {
        for (size_t p = node->start; p < node->start + node->range; ++p) {
          if(primitives[p]->intersect(ray)) {
            return true;
          }
        }
      }

      double l_t0 = ray.min_t;
      double l_t1 = ray.max_t;
      double r_t0 = ray.min_t;
      double r_t1 = ray.max_t;
      if(left != NULL && left->bb.intersect(ray, l_t0, l_t1)) {
        bvh_stack.push(left);
      }
      if(right != NULL && right->bb.intersect(ray, r_t0, r_t1)) {
        bvh_stack.push(right);
      }
    }

    return false;
  }

  bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

    // TODO:
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate. When an intersection does happen.
    // You should store the non-aggregate primitive in the intersection data
    // and not the BVH aggregate itself.

    std::stack<BVHNode *> bvh_stack;
    bvh_stack.push(root);

    BVHNode* node;
    BVHNode* left;
    BVHNode* right;

    double t0 = ray.min_t;
    double t1 = ray.max_t;
    if(!root->bb.intersect(ray, t0, t1)) {
      return false;
    }

    bool hit = false;
    while(!bvh_stack.empty()) {
      node = bvh_stack.top();
      bvh_stack.pop();
      left = node->l;
      right = node->r;

      if(node->isLeaf()) {
        // cout << "start = " << node->start << ", range = " << node->range << "\n";
        for (size_t p = node->start; p < node->start + node->range; ++p) {
          if(primitives[p]->intersect(ray, i)) {
            hit = true;
          }
        }
      }

      double l_t0 = ray.min_t;
      double l_t1 = ray.max_t;
      double r_t0 = ray.min_t;
      double r_t1 = ray.max_t;
      if(left != NULL && left->bb.intersect(ray, l_t0, l_t1)) {
        bvh_stack.push(left);
      }
      if(right != NULL && right->bb.intersect(ray, r_t0, r_t1)) {
        bvh_stack.push(right);
      }
    }

    return hit;
  }

}  // namespace StaticScene
}  // namespace CMU462
