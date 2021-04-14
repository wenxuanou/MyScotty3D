    
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>

namespace PT {

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {

    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    new_node(box, 0, primitives.size(), 0, 0);
    root_idx = 0;
    
    // number of buckets
    const int bNum = 32;
    
    const float delSize = EPS_F / 2; // small increase in size for zero volumn box, EPS_F * 10
    
    // iterate through nodes while adding new nodes
    // using index
    size_t nodeId = 0;
    while(nodeId < nodes.size()){
        
        // no need to split if smaller than max size
        if(nodes[nodeId].size <= max_leaf_size){
            nodeId++;
            continue;
        }
        
        // split if node size too big, assume all nodes are leaves
        //Node n = nodes[nodeId]; // current node
        
        // bbox range of this node
        float XBmin = nodes[nodeId].bbox.min.x,
              XBmax = nodes[nodeId].bbox.max.x,
              YBmin = nodes[nodeId].bbox.min.y,
              YBmax = nodes[nodeId].bbox.max.y,
              ZBmin = nodes[nodeId].bbox.min.z,
              ZBmax = nodes[nodeId].bbox.max.z;
        // initialize bucket, size to number of bucket, initialize to 0
        // row 0: number of primitives; row 1: area
        float BX[2][bNum] = {}, BY[2][bNum] = {}, BZ[2][bNum] = {};
        
        // partition step
        float XStep = (XBmax - XBmin) / (bNum * 1.0f);
        float YStep = (YBmax - YBmin) / (bNum * 1.0f);
        float ZStep = (ZBmax - ZBmin) / (bNum * 1.0f);
        
        // start and end iterator in vector
        auto start = primitives.begin() + nodes[nodeId].start;
        auto end = start + nodes[nodeId].size;
        // find primitives location in each axis bucket
        // only iterate primitives included in current node
        for(auto it = start; it != end; it++){
            
            int buckIdX = floor( ((it -> bbox().center().x) - XBmin) / XStep );
            int buckIdY = floor( ((it -> bbox().center().y) - YBmin) / YStep );
            int buckIdZ = floor( ((it -> bbox().center().z) - ZBmin) / ZStep );
            // avoid out of bound
            buckIdX = std::min(buckIdX, (bNum - 1));
            buckIdY = std::min(buckIdY, (bNum - 1));
            buckIdZ = std::min(buckIdZ, (bNum - 1));
            
            BX[0][buckIdX] += 1.0;                            // number of primitives
            BY[0][buckIdY] += 1.0;
            BZ[0][buckIdZ] += 1.0;
            
            BX[1][buckIdX] += it -> bbox().surface_area();   // surface area
            BY[1][buckIdY] += it -> bbox().surface_area();
            BZ[1][buckIdZ] += it -> bbox().surface_area();
        }
        
        // find lowest cost partition
        for(int count = 1; count < bNum; count++){
            
            BX[0][count] += BX[0][count - 1];               // number of primitives
            BY[0][count] += BY[0][count - 1];
            BZ[0][count] += BZ[0][count - 1];
            
            BX[1][count] += BX[1][count - 1];               // surface area
            BY[1][count] += BY[1][count - 1];
            BZ[1][count] += BZ[1][count - 1];
        }
        // total surface area
        float sN = nodes[nodeId].bbox.surface_area();;
        // total number of primitives
        size_t N = nodes[nodeId].size;
        
        // initialize
        float XminCost = (BX[1][0] / sN * BX[0][0]) + ((sN - BX[1][0])/ sN * (N - BX[0][0]));
        float YminCost = (BY[1][0] / sN * BY[0][0]) + ((sN - BY[1][0])/ sN * (N - BY[0][0]));
        float ZminCost = (BZ[1][0] / sN * BZ[0][0]) + ((sN - BZ[1][0])/ sN * (N - BZ[0][0]));
        int XminId(0), YminId(0), ZminId(0);

        // find the minimal cost and id
        for(int count = 1; count < bNum ; count++){
            float XCost = (BX[1][count] - BX[1][count - 1]) / sN * (BX[0][count] - BX[0][count - 1])
                        + (sN - (BX[1][count] - BX[1][count - 1])) / sN * (sN - (BX[0][count] - BX[0][count - 1]));
            
            float YCost = (BY[1][count] - BY[1][count - 1]) / sN * (BY[0][count] - BY[0][count - 1])
                        + (sN - (BY[1][count] - BY[1][count - 1])) / sN * (sN - (BY[0][count] - BY[0][count - 1]));
            
            float ZCost = (BZ[1][count] - BZ[1][count - 1]) / sN * (BZ[0][count] - BZ[0][count - 1])
                        + (sN - (BZ[1][count] - BZ[1][count - 1])) / sN * (sN - (BZ[0][count] - BZ[0][count - 1]));
            
            if(XCost < XminCost){
                XminCost = XCost;
                XminId = count;
            }
            
            if(YCost < YminCost){
                YminCost = YCost;
                YminId = count;
            }
            
            if(ZCost < ZminCost){
                ZminCost = ZCost;
                ZminId = count;
            }
        }
        
        
        // create left and right node based on cost
        
        float XBound = XBmin + XStep * XminId;
        float YBound = YBmin + YStep * YminId;
        float ZBound = ZBmin + ZStep * ZminId;
        // partition base on bound
        auto boundIt = std::partition(start, end,
                                      [=](const Primitive& prim){ return (prim.bbox().center().x < XBound)
                                                                && (prim.bbox().center().y < YBound)
                                                                && (prim.bbox().center().z < ZBound);
                                                        }
                                     );
                
        // if cannot not good partition, simply slit from middle
        if((start == boundIt) || (end == boundIt)){
            boundIt = start;
            std::advance (boundIt, floor(std::distance(start, end) / 2));
        }
        size_t boundId = size_t(std::distance(primitives.begin(), boundIt));    // get index of bound

        
        // build up bounding box
        BBox bboxLeft, bboxRight;
        for(auto it = start; it != boundIt; it++){
            // handle axis-aline triangle with 0 volumn bbox
            Vec3 minVertex = it -> bbox().min;
            Vec3 maxVertex = it -> bbox().max;
            Vec3 diff = maxVertex - minVertex;
            // add a small value
            if(diff.x == 0){
                maxVertex += Vec3(delSize, 0.0f, 0.0f);
                minVertex -= Vec3(delSize, 0.0f, 0.0f);
            }
            if(diff.y == 0){
                maxVertex += Vec3(0.0f, delSize, 0.0f);
                minVertex -= Vec3(0.0f, delSize, 0.0f);
            }
            if(diff.z == 0){
                maxVertex += Vec3(0.0f, 0.0f, delSize);
                minVertex -= Vec3(0.0f, 0.0f, delSize);
            }
            
            // expand bbox
            bboxLeft.enclose(BBox(minVertex, maxVertex));
        }
        for(auto it = boundIt; it != end; it++){
            // handle axis-aline triangle with 0 volumn bbox
            Vec3 minVertex = it -> bbox().min;
            Vec3 maxVertex = it -> bbox().max;
            Vec3 diff = maxVertex - minVertex;
            // add a small value
            if(diff.x == 0){
                maxVertex += Vec3(delSize, 0.0f, 0.0f);
                minVertex -= Vec3(delSize, 0.0f, 0.0f);
            }
            if(diff.y == 0){
                maxVertex += Vec3(0.0f, delSize, 0.0f);
                minVertex -= Vec3(0.0f, delSize, 0.0f);
            }
            if(diff.z == 0){
                maxVertex += Vec3(0.0f, 0.0f, delSize);
                minVertex -= Vec3(0.0f, 0.0f, delSize);
            }
            
            // expand bbox
            bboxRight.enclose(BBox(minVertex, maxVertex));
        }
        nodes[nodeId].l = new_node(bboxLeft, nodes[nodeId].start, (boundId - nodes[nodeId].start), 0, 0);
        nodes[nodeId].r = new_node(bboxRight, boundId, (nodes[nodeId].start + nodes[nodeId].size - boundId), 0, 0);
        
        // move to next node
        nodeId++;
    }
    
    
    /*
    // print entire bvh
    for(size_t nodeId = 0; nodeId < nodes.size(); nodeId++){
        printf("nodeID: %zu ", nodeId);
        printf("bbox min: (%f, %f, %f) ",
               nodes[nodeId].bbox.min.x,
               nodes[nodeId].bbox.min.y,
               nodes[nodeId].bbox.min.z);
        printf("bbox max: (%f, %f, %f) ",
               nodes[nodeId].bbox.max.x,
               nodes[nodeId].bbox.max.y,
               nodes[nodeId].bbox.max.z);
        printf("\n");
    }
    */
    
    return;
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray, bool shadowRay) const {

    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    size_t nodeId = root_idx;
    Vec2 hitTime;
    
    if(nodes[nodeId].bbox.hit(ray, hitTime)){
        // recursively find cloest hit
        if(shadowRay){ printf("recursive search in bvh: \n"); }
        find_closest_hit(ray, nodeId, ret, shadowRay);
    }
        
    return ret;
}


// recursive find cloest hit
template<typename Primitive>
void BVH<Primitive>::find_closest_hit(const Ray& ray, size_t nodeId, Trace& closestHit, bool shadowRay) const{
    
    
    // TODO: rewrite this, fix intersection problem
    
    // debugging shadow ray
    //if(shadowRay){ printf("nodeID: %zu \n", nodeId); }
    

    // check if leaf node
    if(nodes[nodeId].is_leaf()){
        // if leaf, check if hit primitive
        size_t primId = nodes[nodeId].start;
        
        Trace tempHit;

        // for every primitve in node
        for(size_t countPrim = primId; countPrim < primId + nodes[nodeId].size; countPrim++) {
            tempHit = primitives[countPrim].hit(ray);
            
            closestHit = Trace::min(closestHit, tempHit);   // return closest hit
            
            /*
            if(!shadowRay){
                // for forward tracing
                closestHit = Trace::min(closestHit, tempHit);   // return closest hit
                
            }else{
                if(tempHit.hit){
                    // for detecting shadow ray
                    closestHit = tempHit;   // only return it hit primitive
                }
                
                closestHit = tempHit;
                
                //printf("node id: %zu, hit prim#: %zu, is hit: %d \n", nodeId, countPrim, tempHit.hit);
            }
            */
        }

    }else{
        // if not leaf
        
        // get left and right node info
        size_t nodeId_Left = nodes[nodeId].l;
        size_t nodeId_Right = nodes[nodeId].r;
        BBox nodeBoxLeft = nodes[nodeId_Left].bbox;
        BBox nodeBoxRight = nodes[nodeId_Right].bbox;
        
        Vec2 hitTimeLeft, hitTimeRight;
        bool hitleft = nodeBoxLeft.hit(ray, hitTimeLeft);
        bool hitRight = nodeBoxRight.hit(ray, hitTimeRight);
        
        // find first hit
        if(!hitleft){
            // if not hit left, find cloest in right node
            find_closest_hit(ray, nodeId_Right, closestHit);
        }else if(!hitRight){
            // if not hit right, find cloest in left node
            find_closest_hit(ray, nodeId_Left, closestHit);
        }else if(hitleft && hitRight){
            // if both hit, go in both branch
            
            // handle case when ray origin from inside of box
            float leftTime, rightTime;
            if(hitTimeLeft.x < 0){
                leftTime = hitTimeLeft.y;
            }else{
                leftTime = hitTimeLeft.x;
            }
            
            if(hitTimeRight.x < 0){
                rightTime = hitTimeRight.y;
            }else{
                rightTime = hitTimeRight.x;
            }
            
            float minHitTime = fmin(leftTime, rightTime);
                
            // compare min hit time
            size_t firstHitId = (leftTime <= rightTime) ? nodeId_Left : nodeId_Right;
            size_t secondHitId = (nodeId_Left == firstHitId) ? nodeId_Right : nodeId_Left;
                        // update cloestHit
            find_closest_hit(ray, firstHitId, closestHit);
            
            if(!closestHit.hit || closestHit.distance > minHitTime){
                // if not hit in first branch or actual hit time larger than expected
                // go to second branch
                find_closest_hit(ray, secondHitId, closestHit);
            }

            
        }
        
    }
    
    return;
}


template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(node.l && node.r) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
