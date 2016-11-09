#include "bvh.h"

#include "CMU462/CMU462.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CMU462 { namespace StaticScene {

  void BVHAccel::constructHelper(BVHNode* node, const std::vector<Primitive *> &_primitives, size_t max_leaf_size, int d) {
    if(node->range <= max_leaf_size || d == 0) {
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

    cout << "Node min_cost = " << min_cost << "\n";
    // Iterate through axes, x = 0, y = 1, z = 2
    for(int i = 0; i < 3; i++) {
      min = min_bb[i];
      max = max_bb[i];

      //cout << "\naxis = " << i << "--------------\n";

      // Initialize 8 empty buckets per axis, and individual primitive vectors to reorder primitive list
      double eighth = (max - min) / 8.0;
      //cout << "min = " << min << ", max = " << max << ", eighth = " << eighth << "\n";
      std::vector<BVHNode> buckets (8, BVHNode(BBox(), -1, 0));
      // std::vector<std::vector<Primitive*>> b_prims (8);

      // cout << "min = " << min << ", max = " << max << ", eighth = " << eighth << "\n";

      // Fill buckets with primitives
      for(size_t p_idx = node->start; p_idx < node->start + node->range; p_idx++) {
        // Get primitive and its bbox
        Primitive* p = _primitives[p_idx];
        BBox box = p->get_bbox();

        // Determine primitive's bucket for that axis
        Vector3D centroid = box.centroid();
        double cent = centroid[i];
        //cout << "cent = " << cent << "\n";
        int b_idx = (int) round((cent - min) / eighth);
        // cout << "b_idx = " << b_idx << "\n";
        if(b_idx < 0) b_idx = 0;
        if(b_idx > 7) b_idx = 7;

        //cout << "b_idx = " << b_idx << "\n";

        // Add primitive to bucket
        buckets[b_idx].bb.expand(box);
        buckets[b_idx].range += 1;
        //b_prims[b_idx].push_back(p);
      }

      /*
      for(int blah = 0; blah < buckets.size(); blah++){
        cout << "bucket " << blah << " = " << buckets[blah].bb.surface_area() << "\n";
      }
      */


      // Evaluate SAH for all split point
      double Sn = node->bb.surface_area();
      if(Sn != 0) {
        //cout << "Sn = " << Sn << "\n";
      }
      for(size_t split = 1; split < 8; split++) {
        // Find surface area and primitive count of left side
        BBox l_box = BBox();
        size_t Na = 0;
        for(size_t l = 0; l < split; l++) {
          l_box.expand(buckets[l].bb);
          Na += buckets[l].range;
        }
        double Sa = l_box.surface_area();
        if(Sa != 0) {
          //cout << "Sa = " << Sa << "\n";
        }

        // Find surface area and primitive count of right side
        BBox r_box = BBox();
        size_t Nb = 0;
        for(size_t r = split; r < 8; r++) {
          r_box.expand(buckets[r].bb);
          Nb += buckets[r].range;
        }
        double Sb = r_box.surface_area();
        if(Sb != 0) {
          //cout << "Sb = " << Sb << "\n";
        }

        // Calculate SAH and update min_cost and other fields if needed
        // cout << "Sn = " << Sn << "\n";
        double sah = (Sa / Sn) * Na + (Sb / Sn) * Nb;

        cout << "checking (" << i << ", " << split << ")" << ", cost = " << sah << "\n";
        if(sah < min_cost) {
          split_A = l_box;
          split_B = r_box;
          xyz_split = i;
          split_val = min + split * eighth;
          min_cost = sah;
          min_split = split;
          cout << "----new split: (" << xyz_split << ", " << min_split << ")\n";
          cout << "----split_val = " << split_val << "\n";
          cout << "----min cost = " << min_cost << "\n";
        }
      }
    }
    cout << "--------------\n";
    cout << "split: (" << xyz_split << ", " << min_split << ")\n";
    cout << "split_val = " << split_val << "\n";


    // Ask about partitioning primitive vector in OH
    // Reorder primitives vector by splitting along split value and recombining
    std::vector<Primitive *> left_p;
    std::vector<Primitive *> right_p;
    for(int v = 0; v < _primitives.size(); v++) {
      Primitive* p = _primitives[v];
      if(p->get_bbox().centroid()[xyz_split] < split_val) {
        left_p.push_back(p);
      } else {
        right_p.push_back(p);
      }
    }

    std::vector<Primitive *> new_prim;
    new_prim.insert(new_prim.end(), left_p.begin(), left_p.end());
    new_prim.insert(new_prim.end(), right_p.begin(), right_p.end());

    // _primitives = new_prim;

    // Create new nodes and recurse

    size_t l_start = 0;
    size_t l_range = left_p.size();
    size_t r_start = l_start + l_range;
    size_t r_range = right_p.size();


    BVHNode* left = new BVHNode(split_A, l_start, l_range);
    BVHNode* right = new BVHNode(split_B, r_start, r_range);
    node->l = left;
    node->r = right;

    constructHelper(left, _primitives, max_leaf_size, d - 1);
    constructHelper(right, _primitives, max_leaf_size, d - 1);
  }

  BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
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
    // constructHelper(root, _primitives, max_leaf_size, 1);

  }

  BVHAccel::~BVHAccel() {

    // TODO:
    // Implement a proper destructor for your BVH accelerator aggregate

  }

  BBox BVHAccel::get_bbox() const {
    return root->bb;
  }

  bool BVHAccel::intersect(const Ray &ray) const {

    // TODO:
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.
    bool hit = false;
    for (size_t p = 0; p < primitives.size(); ++p) {
      if(primitives[p]->intersect(ray)) hit = true;
    }

    return hit;

  }

  bool BVHAccel::intersect(const Ray &ray, Intersection *i) const {

    // TODO:
    // Implement ray - bvh aggregate intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate. When an intersection does happen.
    // You should store the non-aggregate primitive in the intersection data
    // and not the BVH aggregate itself.

    bool hit = false;
    for (size_t p = 0; p < primitives.size(); ++p) {
      if(primitives[p]->intersect(ray, i)) hit = true;
    }

    return hit;

  }

}  // namespace StaticScene
}  // namespace CMU462
