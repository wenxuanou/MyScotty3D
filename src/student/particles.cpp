
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Particle::update(const PT::BVH<PT::Object>& scene, float dt, float radius) {

    // gravity be only force, fixed acceleration
    
    
    // check collision
    
    // build ray on particle
    Ray trajectory(pos, velocity);          // velocity turn to unit vector automatically
    Vec3 particleDir = velocity.unit();     // direction of partical motion, unit vector
    float dist = velocity.norm() * dt;      // travel distance in dt

    // TODO: handle close collision
    trajectory.dist_bounds = Vec2(0.0f, dist);       // only check collision in current time step
    
    PT::Trace hit = scene.hit(trajectory);      // find hit point
    
    if(!hit.hit){
        // if not hit, update base on pos and velocity
        pos += velocity * dt;
        velocity += acceleration * dt;
        // substract age
        age -= dt;
        return age > 0.0f;
    }
    
    
    float cos_theta;
    float d;           // center position when radius hit surface
    float ddt = 0.0f;
    
    while(hit.hit && ddt < dt){
        
        cos_theta = dot((-1.0f * particleDir), hit.normal);
        d = radius / abs(cos_theta);    // make sure all positive
        
        if(hit.distance <= d + d / 10.0f){
            // partical already at surface, reflect immediately
            // position not changed
            velocity = velocity - 2.0f * dot(velocity, hit.normal) * hit.normal; // reflect against normal
                        
        }else{
            // if partical not yet reach surface
            
            float timeSpent = (hit.distance - d) / velocity.norm();             // find actual time spend to hit surface
            pos += velocity * timeSpent;                                        // move to contact position
            velocity += acceleration * timeSpent;
            
            ddt += timeSpent;
        }
        
        trajectory = Ray(pos, velocity);
        dist = velocity.norm() * (dt - ddt);
        trajectory.dist_bounds = Vec2(0.0f, dist);
        hit = scene.hit(trajectory);
        
    }
    
    age -= dt;
    return age > 0.0f;
}
