#include <algorithm>
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect

    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    
    // get vertex position
    Vec3 p0 = v_0.position;
    Vec3 p1 = v_1.position;
    Vec3 p2 = v_2.position;
    
    float xMin = fmin(p0.x, fmin(p1.x, p2.x));
    float xMax = fmax(p0.x, fmax(p1.x, p2.x));
    float yMin = fmin(p0.y, fmin(p1.y, p2.y));
    float yMax = fmax(p0.y, fmax(p1.y, p2.y));
    float zMin = fmin(p0.z, fmin(p1.z, p2.z));
    float zMax = fmax(p0.z, fmax(p1.z, p2.z));
    
    BBox box(Vec3(xMin, yMin, zMin), Vec3(xMax, yMax, zMax));
    return box;
}

Trace Triangle::hit(const Ray& ray) const {

    // Vertices of triangle - has postion and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
//    (void)v_0;
//    (void)v_1;
//    (void)v_2;

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the three above points.

    // initialize
    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)
    
    // get vertex position
    Vec3 p0 = v_0.position;
    Vec3 p1 = v_1.position;
    Vec3 p2 = v_2.position;
    // get normal vector
    Vec3 n0 = v_0.normal;
    Vec3 n1 = v_1.normal;
    Vec3 n2 = v_2.normal;
    // get edge vector
    Vec3 e1 = p1 - p0;
    Vec3 e2 = p2 - p0;
    // get vector to ray origin and ray direction
    Vec3 s = ray.point - p0;
    Vec3 d = ray.dir;
        
    float scalar = dot(cross(e1,d),e2);
    // scalar is zero when d in same direction as e1
    // or cross(e1,d) perpendicular to e2,
    // basically when d is on plane span by e1 an e2
    
    if(scalar == 0){ return ret; }
        
    // proceed when intersect is possible
    scalar = 1.0f / scalar;

    // solve ray matrix
    float u = scalar * -1 * dot(cross(s, e2), d);
    float v = scalar * dot(cross(e1, d), s);
    float t = scalar * -1 * dot(cross(s, e2), e1);
        
    // check if out of dist_bounds
    if(t < ray.dist_bounds.x || t > ray.dist_bounds.y){ return ret; }
    // check if really hit
    if((u < 0.0) || (v < 0.0)){ return ret; }
    if(u + v > 1.0){ return ret;}
        
    // get intersect point info
    ret.hit = true;
    ret.distance = t; // d is unit direction vector with length 1
    ret.position = p0 + u * e1 + v * e2;
    ret.normal = (1 - u - v) * n0 + u * n1 + v * n2;
    
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
