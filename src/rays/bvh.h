
#pragma once

#include "../lib/mathlib.h"
#include "../platform/gl.h"

#include "trace.h"

namespace PT {

template<typename Primitive> class BVH {
public:
    BVH() = default;
    BVH(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);
    void build(std::vector<Primitive>&& primitives, size_t max_leaf_size = 1);

    BVH(BVH&& src) = default;
    BVH& operator=(BVH&& src) = default;

    BVH(const BVH& src) = delete;
    BVH& operator=(const BVH& src) = delete;

    BBox bbox() const;
    Trace hit(const Ray& ray, bool shadowRay = false) const;

    BVH copy() const;
    size_t visualize(GL::Lines& lines, GL::Lines& active, size_t level, const Mat4& trans) const;

    std::vector<Primitive> destructure();
    void clear();

private:
    class Node {
        
        BBox bbox;
        size_t start, size, l, r;

        // A node is a leaf if l == r, since all interior nodes must have distinct children
        bool is_leaf() const;
        friend class BVH<Primitive>;
    };

    // recursive bvh build
    void buildChild(size_t parentId, std::vector<Primitive>& prims, size_t max_leaf_size, int level);
    
    // recursive find cloest hit
    void find_closest_hit(const Ray& ray, size_t nodeId, Trace& closestHit, bool shadowRay = false) const;
    
    // start: start index of primitives in primitives array
    // size: range of index in primitives list, # of primitives in subtrees
    // l: index of left child node
    // r: index of right child node
    // returns id of the created node
    size_t new_node(BBox box = {}, size_t start = 0, size_t size = 0, size_t l = 0, size_t r = 0);

    std::vector<Node> nodes;
    std::vector<Primitive> primitives;
    size_t root_idx = 0;
};

} // namespace PT

#ifdef SCOTTY3D_BUILD_REF
#include "../reference/bvh.inl"
#else
#include "../student/bvh.inl"
#endif
