
#include "../rays/env_light.h"
#include "debug.h"

#include <limits>

namespace PT {

Light_Sample Env_Map::sample() const {

    Light_Sample ret;
    ret.distance = std::numeric_limits<float>::infinity();

    // TODO (PathTracer): Task 7
    // Uniformly sample the sphere. Tip: implement Samplers::Sphere::Uniform
//    Samplers::Sphere::Uniform uniform;
//    ret.direction = uniform.sample(ret.pdf);

    // Once you've implemented Samplers::Sphere::Image, remove the above and
    // uncomment this line to use importance sampling instead.
     ret.direction = sampler.sample(ret.pdf);

    ret.radiance = sample_direction(ret.direction);
    return ret;
}

Spectrum Env_Map::sample_direction(Vec3 dir) const {

    // TODO (PathTracer): Task 7
    // Find the incoming light along a given direction by finding the corresponding
    // place in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 image pixels nearest to the exact direction.
    
    // convert xyz to theta phi
    float theta = acos(-1.0f * dir.y);                // map to height
    float phi = atan(dir.z / dir.x);                  // map to width, range from -pi/2 to +pi/2
    
    // determine in which quadrant, get actual angle
    if(dir.x < 0.0f && dir.z > 0.0f){
        phi = PI_F + phi;
    }else if(dir.x > 0.0f && dir.z < 0.0f){
        phi = 2 * PI_F + phi;
    }else if(dir.x < 0.0f && dir.z < 0.0f){
        phi = PI_F + phi;
    }
    
    phi += PI_F;
    
    // get size of env image
    std::pair<size_t, size_t> dim = image.dimension();
    size_t W = dim.first;   // map to 0 ~ 2pi
    size_t H = dim.second;  // map to 0 ~ pi
    
    theta = theta / PI_F;           // map to 0 ~ h
    phi = phi / (2.0f * PI_F);      // map to 0 ~ w
    
    
    // round to integer
    size_t theta_1 = (size_t) floor(theta * H * 1.0f), theta_2 = (size_t) ceil(theta * H * 1.0f);
    size_t phi_1 = (size_t) floor(phi * W * 1.0f), phi_2 = (size_t) ceil(phi * W * 1.0f);
    
    // circling
    theta_1 %= H; theta_2 %= H;
    phi_1 %= W; phi_2 %= W;
    
    // bilinear interpolation
    Spectrum p1 = (image.at(phi_1, theta_1) + image.at(phi_2, theta_1)) / 2.0f;
    Spectrum p2 = (image.at(phi_1, theta_2) + image.at(phi_2, theta_2)) / 2.0f;
    
    return (p1 + p2) / 2.0f;
}

Light_Sample Env_Hemisphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Hemisphere::sample_direction(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Light_Sample Env_Sphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Sphere::sample_direction(Vec3) const {
    return radiance;
}

} // namespace PT
