
#include "../lib/mathlib.h"
#include "debug.h"

bool BBox::hit(const Ray& ray, Vec2& times) const {

    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.

    // get ray info
    Vec3 o = ray.point;
    Vec3 d = ray.dir;   // unit direction
    
    // buffer for min and max point
    Vec3 P0, P1;
    
    Vec3 inv_d = 1 / d;
    // handle d.x = 0
    if(inv_d.x >= 0.0f){
        P0.x = min.x;
        P1.x = max.x;
    }else{
        P0.x = max.x;
        P1.x = min.x;
    }
    // d.y = 0
    if(inv_d.y >= 0.0f){
        P0.y = min.y;
        P1.y = max.y;
    }else{
        P0.y = max.y;
        P1.y = min.y;
    }
    // d.z = 0
    if(inv_d.z >= 0.0f){
        P0.z = min.z;
        P1.z = max.z;
    }else{
        P0.z = max.z;
        P1.z = min.z;
    }
    
    // finding tmin and tmax on x
    float tmin = (P0.x - o.x) * inv_d.x;
    float tmax = (P1.x - o.x) * inv_d.x;
    
    // swap if tmin > tmax
    if(tmin > tmax){ std::swap(tmin, tmax); }
    
    
    // finding tmin and tmax on y
    float tymin = (P0.y - o.y) * inv_d.y;
    float tymax = (P1.y - o.y) * inv_d.y;
    
    // swap if tymin > tymax
    if (tymin > tymax){ std::swap(tymin, tymax); }
    
    // check if intersect box in xy plane
    if ((tmin > tymax) || (tymin > tmax)){ return false; }
    
    // update tmin and tmax
    if (tymin > tmin){ tmin = tymin; }
    if (tymax < tmax){ tmax = tymax; }
    
    
    // finding tmin and tmax on z
    float tzmin = (P0.z - o.z) * inv_d.z;
    float tzmax = (P1.z - o.z) * inv_d.z;
    
    // swap if tzmin > tzmax
    if(tzmin > tzmax){ std::swap(tzmin, tzmax); }
    
    // check if intersect box in xyz
    if ((tmin > tzmax) || (tzmin > tmax)){ return false; }
    
    // update tmin and tmax
    if (tzmin > tmin){ tmin = tzmin; }
    if (tzmax < tmax){ tmax = tzmax; }
    
    // store back in Vec2
    times.x = tmin;
    times.y = tmax;
    
    return true;
}
