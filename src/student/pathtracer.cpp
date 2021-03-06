
#include "../rays/pathtracer.h"
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Spectrum Pathtracer::trace_pixel(size_t x, size_t y) {

    Vec2 xy((float)x, (float)y);            // coordinate
    Vec2 wh((float)out_w, (float)out_h);    // width and height

    // TODO (PathTracer): Task 1

    // Generate a sample within the pixel with coordinates xy and return the
    // incoming light using trace_ray.

    // Tip: Samplers::Rect::Uniform
    // Tip: you may want to use log_ray for debugging

    // This currently generates a ray at the bottom left of the pixel every time.
    
    Samplers::Rect::Uniform sampler(Vec2(1.0f));
    float pdf;

    Vec2 xy_sampled = xy + sampler.sample(pdf);
    Ray out = camera.generate_ray(xy_sampled / wh);
        
    // visualize ray
    if(RNG::coin_flip(0.0000005f)) log_ray(out, 10.0f);
    
    return trace_ray(out);
}

Spectrum Pathtracer::trace_ray(const Ray& ray) {

    // Trace ray into scene. If nothing is hit, sample the environment
    Trace hit = scene.hit(ray);
    if(!hit.hit) {
        if(env_light.has_value()) {
            return env_light.value().sample_direction(ray.dir);
        }
        return {};
    }

    // If we're using a two-sided material, treat back-faces the same as front-faces
    const BSDF& bsdf = materials[hit.material];
    if(!bsdf.is_sided() && dot(hit.normal, ray.dir) > 0.0f) {
        hit.normal = -hit.normal;
    }

    // Set up a coordinate frame at the hit point, where the surface normal becomes {0, 1, 0}
    // This gives us out_dir and later in_dir in object space, where computations involving the
    // normal become much easier. For example, cos(theta) = dot(N,dir) = dir.y!
    Mat4 object_to_world = Mat4::rotate_to(hit.normal);
    Mat4 world_to_object = object_to_world.T();
    Vec3 out_dir = world_to_object.rotate(ray.point - hit.position).unit();     // out_dir in object space

    // Debugging: if the normal colors flag is set, return the normal color
    if(debug_data.normal_colors) return Spectrum::direction(hit.normal);

    // Now we can compute the rendering equation at this point.
    // We split it into two stages: sampling lighting (i.e. directly connecting
    // the current path to each light in the scene), then sampling the BSDF
    // to create a new path segment.

    // TODO (PathTracer): Task 5
    // The starter code sets radiance_out to (0.5,0.5,0.5) so that you can test your geometry
    // queries before you implement path tracing. You should change this to (0,0,0) and accumulate
    // the direct and indirect lighting computed below.
    
    
    // sampling light
    Spectrum radiance_out; // no light at the begining
    {
        auto sample_light = [&](const auto& light) {
            // If the light is discrete (e.g. a point light), then we only need
            // one sample, as all samples will be equivalent
            int samples = light.is_discrete() ? 1 : (int)n_area_samples;
            for(int i = 0; i < samples; i++) {

                Light_Sample sample = light.sample(hit.position);   // get light sample at hit point
                Vec3 in_dir = world_to_object.rotate(sample.direction);

                // If the light is below the horizon, ignore it
                float cos_theta = in_dir.y;
                if(cos_theta <= 0.0f) continue;

                // If the BSDF has 0 throughput in this direction, ignore it.
                // This is another oppritunity to do Russian roulette on low-throughput rays,
                // which would allow us to skip the shadow ray cast, increasing efficiency.
                Spectrum attenuation = bsdf.evaluate(out_dir, in_dir);
                if(attenuation.luma() == 0.0f) continue;    // Spectrum.luma() ranges from 0 to 1
      
                // TODO (PathTracer): Task 4
                // Construct a shadow ray and compute whether the intersected surface is
                // in shadow. Only accumulate light if not in shadow.

                // Tip: since you're creating the shadow ray at the intersection point, it may
                // intersect the surface at time=0. Similarly, if the ray is allowed to have
                // arbitrary length, it will hit the light it was cast at. Therefore, you should
                // modify the time_bounds of your shadow ray to account for this. Using EPS_F is
                // recommended.

                // Note: that along with the typical cos_theta, pdf factors, we divide by samples.
                // This is because we're  doing another monte-carlo estimate of the lighting from
                // area lights.
                
                
                // get shadow ray, origin at intersection, point to light source
                Ray shadowRay = Ray(hit.position, sample.direction);
                // set ray bound, use EPS_F avoid self intersection
                shadowRay.dist_bounds = Vec2(EPS_F, sample.distance - EPS_F);
                // check hit point
                Trace shadowHit = scene.hit(shadowRay, true); // accumulate radiance if not hit object
                
                // visualize ray
//                if(RNG::coin_flip(0.00005f)) log_ray(shadowRay, 10.0f);
                
                // accumulate radiance from each light source
                radiance_out +=
                    (cos_theta / (samples * sample.pdf)) * (!shadowHit.hit) * sample.radiance * attenuation;
                
            }
        };

        // If the BSDF is discrete (i.e. uses dirac deltas/if statements), then we are never
        // going to hit the exact right direction by sampling lights, so ignore them.
        if(!bsdf.is_discrete()) {
            for(const auto& light : lights) sample_light(light);
            if(env_light.has_value()) sample_light(env_light.value());
        }
    }

    // TODO (PathTracer): Task 5
    // Compute an indirect lighting estimate using pathtracing with Monte Carlo.

    // (1) Ray objects have a depth field; if it reaches max_depth, you should
    // terminate the path.

    // (2) Randomly select a new ray direction (it may be reflection or transmittance
    // ray depending on surface type) using bsdf.sample()

    // (3) Compute the throughput of the recursive ray. This should be the current ray's
    // throughput scaled by the BSDF attenuation, cos(theta), and inverse BSDF sample PDF.
    // Potentially terminate the path using Russian roulette as a function of the new throughput.
    // Note that allowing the termination probability to approach 1 may cause extra speckling.

    // (4) Create new scene-space ray and cast it to get incoming light. As with shadow rays, you
    // should modify time_bounds so that the ray does not intersect at time = 0. Remember to
    // set the new throughput and depth values.

    // (5) Add contribution due to incoming light with proper weighting. Remember to add in
    // the BSDF sample emissive term.

        
    // bsdf sample
    BSDF_Sample bsdfSample = bsdf.sample(out_dir);          // random bsdf sample at hit
    
    // collect emissive before max ray check and russian roulette
    radiance_out += bsdfSample.emissive;
    
    
    // if ray reach max depth, early return
//    if(RNG::coin_flip( fmin((ray.depth * 1.0f) / (max_depth * 1.0f), 1) ) ){ return radiance_out;  }   // clip probability between 0 and 1
    if(ray.depth >= max_depth){ return radiance_out; }
    
    Spectrum radiance_indirect;                         // indirect radiance from other objects
    
    Vec3 newDir = bsdfSample.direction;                 // get bsdf sample direction in object space
    newDir = object_to_world.rotate(newDir).unit();      // map to world space
    float cos_theta = bsdfSample.direction.y;           // get cosine theta in object space for ease
    
    // build up new ray
    Ray recurRay(hit.position, newDir);
    recurRay.depth = ray.depth + 1;                     // increase depth
    recurRay.throughput = ray.throughput * bsdfSample.attenuation * abs(cos_theta) / bsdfSample.pdf; // update throughput
    recurRay.dist_bounds = Vec2(EPS_F, std::numeric_limits<float>::infinity()); // avoid intersect at 0
    
    
    Spectrum terminateProbability = bsdfSample.attenuation * abs(cos_theta) / bsdfSample.pdf;
    
    float prrScale = 2.0f;                                     // tuning scalar of prr
    float prr = terminateProbability.luma() * prrScale;        // turn spectrum into float
        
    prr = fmin(0, fmax(prr, 1));                               // cap between 0 and 1
    
    // ramdonly terminate
    if(RNG::coin_flip(prr)){ return radiance_out; }
    
    radiance_indirect = trace_ray(recurRay) * bsdfSample.attenuation * abs(cos_theta) / (bsdfSample.pdf * (1.0f - prr));
    // recursive call, include emission from other objects
    
    if(RNG::coin_flip(0.00005f)) log_ray(recurRay, 10.0f);
    
    return radiance_out + radiance_indirect;
}

} // namespace PT
