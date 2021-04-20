
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"

namespace Samplers {

Vec2 Rect::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 1
    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()

    // the PDF should integrate to 1 over the whole rectangle
    pdf = 1.0f / (size.x * size.y);
    
    float xRand = size.x * RNG::unit();
    float yRand = size.y * RNG::unit();
    
    return Vec2(xRand, yRand);
}

Vec3 Hemisphere::Cosine::sample(float& pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.
    return Vec3();
}

Vec3 Sphere::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 7
    // Generate a uniformly random point on the unit sphere (or equivalently, direction)
    // Tip: start with Hemisphere::Uniform

//    pdf = 1.0f; // what was the PDF at the chosen direction?
    
    Vec3 dir = hemi.sample(pdf);
    
    if(RNG::coin_flip(0.5f)){
        dir *= Vec3(1.0f, -1.0f, 1.0f); // randomly choose upper or lower hemisphere
    }
    
    pdf *= 0.5f;
    
    return dir;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;
    
    float sin_theta;
    
    // find total luminance
    for(size_t row = 0; row < h; row++){
        for(size_t col = 0; col < w; col++){
            // get sine theta
            sin_theta = sin(row * PI_F / (1.0f * h)); // map index to radians, 0 to pi
            
            float temp = image.at(col, row).luma() * sin_theta;
            total += temp;

            pdf.push_back(temp);  // push back pdf for normalize
            cdf.push_back(total);

        }
    }
    
    // normalize and fill up cdf
    for(size_t i = 0; i < pdf.size(); i++){
        pdf[i] = pdf[i] / total;
        cdf[i] = cdf[i] / total;
    }
    
    // debug
    if(cdf[cdf.size() - 1] != 1){
        printf("cdf error!! \n");
        printf("cdf.size(): %zu, cdf[10]: %f \n", cdf.size(), cdf[10]);
        printf("pdf.size(): %zu, pdf[10]: %f \n", pdf.size(), pdf[10]);
    }
    
    return;
}

Vec3 Sphere::Image::sample(float& out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

//    out_pdf = 1.0f; // what was the PDF (again, PMF here) of your chosen sample?
    
    // get iterator
    auto start = cdf.begin();
    auto end = cdf.end();
    // inverse sample random number from cdf
    auto sampleIt = lower_bound(start, end, RNG::unit());   // value >= rand(0~1)
    
    if(sampleIt == end){
        printf("inverse sampling error!!!\n");
        return Vec3();
    }
    
    size_t sampleId = distance(start, sampleIt);            // convert to index
    
    // get pdf
    out_pdf = pdf[sampleId];
    
    // get 2D index
    size_t col = sampleId % w;
    size_t row = sampleId / w;
    // convert to radians
    float theta = row * PI_F / (1.0f * h);
    float phi = col * (2.0f * PI_F) / (1.0f * h);
    
    Vec3 dir(cos(phi) * sin(theta),
                        cos(theta),
             sin(phi) * sin(theta));
    
    return dir;
}

Vec3 Point::sample(float& pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float& pmf) const {
    if(RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float& pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
