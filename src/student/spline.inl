
#include "../geometry/spline.h"
#include "debug.h"
#include <cstdio>   // for debug

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    
    // time in range [0,1]
    float time2 = time * time;          // time square
    float time3 = time * time * time;   // time cube
    
    // compute Hermite basis
    float h00 = 2.0f * time3 - 3.0f * time2 + 1.0f;
    float h10 = time3 - 2.0f * time2 + time;
    float h01 = -2.0f * time3 + 3.0f * time2;
    float h11 = time3 - time2;
    
    // return interpolated value p(t)
    // make sure work for vector, angles, colors
        
    return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...

    
    // if no knots at all
    if(!any()){
        return cubic_unit_spline(0.0f, T(), T(), T(), T()); // value with default constructor
    }
    
    // if only one knot
    if(control_points.size() == 1){
        // point to the first element in map
        auto it = control_points.begin();
        
        T pos = it -> second;
        
        return cubic_unit_spline(0.0f, pos, T(), T(), T()); // value of the only knot
    }
    
    // check initial knot time
    auto start = control_points.begin();
    float init_time = start -> first;
    // if time before init_time
    if(time <= init_time){
        T pos = start -> second;
        return cubic_unit_spline(0.0f, pos, T(), T(), T());    // value of the initial point
    }
    
    // check final knot time
    auto end = control_points.end(); end--;
    float final_time = end -> first;
    // if time after final_time
    if(time >= final_time){
        T pos = end -> second;
        return cubic_unit_spline(0.0f, pos, T(), T(), T());   // value of the final point
    }
    
    
    
    // with 2 or more knots, for time between initial and final time
    // find the 4 knots, t0 < t1 <= t < t2 < t3
    auto k2 = control_points.upper_bound(time); // iterator for k2
    auto k1 = k2; k1--; // k1
    
    
    float t1 = k1 -> first; // time
    float t2 = k2 -> first;
    T pos1 = k1 -> second;  // position
    T pos2 = k2 -> second;
    
    
    // check virtual knot
    float t0, t3;       // time
    T pos0, pos3;   // position
    
    if(k1 == start){
        t0 = t1 - (t2 - t1);
        pos0 = pos1 - (pos2 - pos1);
    }else{
        auto k0 = k1; k0--;
        t0 = k0 -> first;
        pos0 = k0 -> second;
    }
    
    if(k2 == end){
        t3 = t2 + (t2 - t1);
        pos3 = pos2 + (pos2 - pos1);
    }else{
        auto k3 = k2; k3++;
        t3 = k3 -> first;
        pos3 = k3 -> second;
    }
    
    // Catmull-Rom
    // normalize, make t in [0,1]
    time = (time - t1) / (t2 - t1);
    T tan1 = (pos2 - pos0) / (t2 - t0) * (t2 - t1); // tangent, scale by normalization factor
    T tan2 = (pos3 - pos1) / (t3 - t1) * (t2 - t1);
        
    return cubic_unit_spline(time, pos1, pos2, tan1, tan2);
    
}
