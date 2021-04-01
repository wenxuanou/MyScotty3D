
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
    float sh = 2.0f * tan((vert_fov / 2.0f) / 180.0f * M_PI);
    float sw = sh * aspect_ratio;
        
    // scale coordinate
    screen_coord = screen_coord * Vec2(sw, sh);
        
    // world coordinate, homogeneous
    // 1 unit negative away from camera
    Vec3 world_coord(screen_coord.x, screen_coord.y, -10.0f);
    world_coord = iview * world_coord;
        
    Vec3 dir = world_coord - position;
    
    // output ray from camera, point to screen point
    return Ray(position, dir);
}
