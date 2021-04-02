
#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.

    screen_coord = screen_coord - Vec2(0.5f); // make center the origin
    
    // get screen height and screen width
    float sh = 2.0f * focal_dist * tan((vert_fov / 2.0f) / 180.0f * M_PI);
    float sw = sh * aspect_ratio;
        
    // scale coordinate
    screen_coord = screen_coord * Vec2(sw, sh);
        
    // start position with aperature
    Vec2 apertureSize(aperture);
    Samplers::Rect::Uniform sampler(apertureSize);
    float pdf;
    Vec2 aperturePos = sampler.sample(pdf) - Vec2((aperture / 2.0f)); // centered on origin
    
    // world coordinate, homogeneous
    // focal distance away from camera
    Vec3 world_coord(screen_coord.x, screen_coord.y, -1.0f * focal_dist);
    Vec3 aperture_world(aperturePos.x, aperturePos.y, 0.0f);
    
    world_coord = iview * world_coord;
    aperture_world = iview * aperture_world;
    
    Vec3 dir = world_coord - aperture_world;
    
    // output ray from camera, point to screen point
    return Ray(aperture_world, dir);
}
