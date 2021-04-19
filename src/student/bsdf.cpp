
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).
    
    Vec3 normal(0.0f, 1.0f, 0.0f);              // surface normal
    Vec3 proj = normal * dot(dir, normal);      // projection on normal
    Vec3 reflect = 2.0f * proj - dir;
    
    return reflect;
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.
    
    
    // check if ray enter the surface
    Vec3 normal(0.0f, 1.0f, 0.0f);
    float cos_theta_i = dot(out_dir, normal);    // treat out_dir as input, all unit vector
    float eta_i, eta_t; // eta_i: for out_dir; eta_t: for in_dir
    if(cos_theta_i > 0.0f){
        // entering surface, out_dir in vaccum
        eta_t = index_of_refraction;    // eta of in_dir
        eta_i = 1.0f;                   // eta of out_dir
    }else{
        // out of surface, out_dir in surface
        eta_t = 1.0f;                   // eta of in_dir
        eta_i = index_of_refraction;    // eta of out_dir
    }
    
    
    // check if total internal reflection
    float sin_theta_i = sqrt(1.0f - cos_theta_i * cos_theta_i);
    float sin_theta_t = eta_i * sin_theta_i / eta_t;
    
    
    if(sin_theta_t > 1.0f){
//        printf("is internal reflection \n");
//        printf("sin_theta_t: %f \n", sin_theta_t);
        was_internal = true;
        return Vec3();
    }
    
    // if not total internal reflection
    float cos_theta_t = sqrt(1.0f - sin_theta_t * sin_theta_t);
    cos_theta_t = (cos_theta_i > 0.0f)? (-1.0f * cos_theta_t) : (cos_theta_t);
    
    Vec3 in_dir(-1.0f * out_dir.x * sin_theta_t,
                                    cos_theta_t,
                -1.0f * out_dir.z * sin_theta_t);
    
    in_dir.normalize();
    return in_dir;
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    
    float pdf;
    ret.direction = sampler.sample(pdf);    // object space
    ret.attenuation = evaluate(out_dir, ret.direction);
    ret.pdf = pdf;
    
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
//    ret.attenuation = Spectrum(); // What is the ratio of reflected/incoming light?
//    ret.direction = Vec3();       // What direction should we sample incoming light from?
//    ret.pdf = 0.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    
    float cos_theta = dot(out_dir, Vec3(0.0f, 1.0f, 0.0f)); // all unit vectors, find cosine theta
    
    ret.attenuation = reflectance / abs(cos_theta);
    ret.direction = reflect(out_dir);
    ret.pdf = 1.0f;
    
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.
    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
//    ret.attenuation = Spectrum(); // What is the ratio of reflected/incoming light?
//    ret.direction = Vec3();       // What direction should we sample incoming light from?
//    ret.pdf = 0.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
        
    float R0 = pow((1.0f - index_of_refraction) / (1.0f + index_of_refraction), 2);
    float cos_theta_i = dot(out_dir, Vec3(0.0f, 1.0f, 0.0f));
    float R_theta = R0 + (1.0f - R0) * pow((1.0f - abs(cos_theta_i)), 5);    // Fresnel reflectance, 0 to 1, make sure cos_theta positive
    
//    printf("R_theta: %f \n", R_theta);
    
    // figure out entering or leaving
    float eta_i, eta_t;
    if(cos_theta_i > 0.0f){
        eta_i = 1.0f;
        eta_t = index_of_refraction;
    }else{
        eta_i = index_of_refraction;
        eta_t = 1.0f;
    }
    
    
    bool was_internal = false;
    
    Vec3 reflect_dir = reflect(out_dir);
    Vec3 refract_dir = refract(out_dir, index_of_refraction, was_internal);
    
    
    // check if total internal reflection
    if(was_internal){
        // total internal reflection, only reflection
        ret.attenuation = R_theta * reflectance / abs(cos_theta_i);
        ret.direction = reflect_dir;
        ret.pdf = 1.0f;

        return ret;
        
    }else{
        // not total internal reflection, choose reflection or refraction direction
        if(RNG::coin_flip(R_theta)){
//        if(RNG::coin_flip(0.0f)){       // pure reflect
            // reflect
            ret.attenuation = R_theta * reflectance / abs(cos_theta_i);
            ret.direction = reflect_dir;
            ret.pdf = R_theta;
        }else{
            // refract
            ret.attenuation = (pow(eta_t,2) / pow(eta_i,2)) * (1.0f - R_theta) * transmittance / abs(cos_theta_i);
            ret.direction = refract_dir;
            ret.pdf = 1.0f - R_theta;
        }
        
        // pure refraction
//        ret.attenuation = (pow(eta_t,2) / pow(eta_i,2)) * (1.0f - R_theta) * transmittance / abs(cos_theta_i);
//        ret.direction = refract_dir;
//        ret.pdf = 1.0f - R_theta;
        
    }
    
    
    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    BSDF_Sample ret;
    ret.attenuation = Spectrum(); // What is the ratio of reflected/incoming light?
    ret.direction = Vec3();       // What direction should we sample incoming light from?
    ret.pdf = 0.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    
    
    bool was_internal;
    Vec3 in_dir = refract(out_dir, index_of_refraction, was_internal);
    
    if(was_internal){ return ret; } // if total internal reflection, no refraction
    
    float cos_theta = dot(out_dir, Vec3(0.0f, 1.0f, 0.0f));
    
    ret.attenuation = transmittance / abs(cos_theta);
    ret.direction = in_dir;         // refraction light path is determined
    ret.pdf = 1.0f;
    
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
