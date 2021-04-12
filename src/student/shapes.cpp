
#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {

    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!

    // initialize
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
    
    // get ray origin and unit direction
    Vec3 o = ray.point;
    Vec3 d = ray.dir;
    float b = dot(o,d);
    
    // compute distance
    float t;
    float t_in = -1.0f * b + sqrt(b * b - o.norm_squared() + 1.0f);
    float t_out = -1.0f * b - sqrt(b * b - o.norm_squared() + 1.0f);
    
    // find the first intersection which within the dist_bound
    t = fmin(t_in, t_out);
    if(t_in < ray.dist_bounds.x || t_in > ray.dist_bounds.y){ t = t_out; }
    if(t_out < ray.dist_bounds.x || t_out > ray.dist_bounds.y){ t = t_in; }
    // check whether has a valid intersection
    if( isnan(t) ){ return ret; }
    
    if(t < ray.dist_bounds.x || t > ray.dist_bounds.y){ return ret; }
    
    
    // compute hit point
    Vec3 p = o + t * d; // sphere centered at zero
    
    // check whether hit point on sphere
    //if(std::abs(p.norm() - radius) <= EPS_F){
        ret.hit = true;
        ret.distance = t;
        ret.position = p;
        ret.normal = p.unit();  // unit normal, sphere centered at origin
    //}
    
    return ret;
}

} // namespace PT
