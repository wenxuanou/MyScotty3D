
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Particle::update(const PT::BVH<PT::Object>& scene, float dt, float radius) {

    // gravity be only force, fixed acceleration
    
    
    // check collision
    
    // build ray on particle
    Ray trajectory(pos, velocity);          // velocity turn to unit vector automatically
    Vec3 particleDir = velocity.unit();     // direction of partical motion
    float dist = velocity.norm() * dt;      // travel distance in dt

    // TODO: handle close collision
    trajectory.dist_bounds = Vec2(0.0f, dist);       // only check collision in current time step
    
    PT::Trace hit = scene.hit(trajectory);      // find hit point
    
    if(hit.hit){
        // if hit, check bounce
        // TODO: account for particle radius, ray hit point is not actual collide point
        // sine of angle between partical ray to surface normal
        float cos_theta = dot(-1.0f * particleDir, hit.normal);
        float dist = hit.distance - radius / cos_theta;   // dist to center of particle when hit
        
        float ddt = dist / velocity.norm();           // actual time for hit
        
        if(dist > 0){
            // particle center when hit, move to hit point
            pos += dist * particleDir;
            // same magnitude, direction reflected
            velocity = velocity - 2.0f * dot(velocity, hit.normal) * hit.normal + acceleration * ddt;
            
        }else{
            // if next hit point smaller than radius, immediate reflect
            pos += (velocity - 2.0f * dot(velocity, hit.normal) * hit.normal) * ddt;
            velocity = (velocity - 2.0f * dot(velocity, hit.normal) * hit.normal) + acceleration * ddt;
        }
        
        // TODO: handle multiple hits in same time step
        age -= ddt;
        update(scene, dt - ddt, radius);    // recursive update for multiple hit in same time step
        
    }else{
        // if not hit, update base on pos and velocity
        pos += velocity * dt;
        velocity += acceleration * dt;
        
        // substract age
        age -= dt;
    }
        
    return age > 0;
}
