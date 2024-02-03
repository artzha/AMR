#include "navigation_params.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include "shared/math/math_util.h"

using std::max;
using std::min;

float run1DTOC(const MotionLimits &limits, const float x0, const float v0,
               const float xf, const float vf,
               const float dt = 0.05 // timestep
) {
  // Note This function only works with positive velocities
  const float distance_left = xf - x0;
  if (distance_left <= 0 or v0 <= 0)
    return 0;

  // Forward
  float dv_a = limits.max_acceleration * dt;  // in m/s
  float dv_d = limits.max_decceleration * dt; // in m/s

  float v_accel = min(v0 + dv_a, limits.max_speed);

  float cruise_stopping_distance = v0 * dt + getStoppingDistance(v0, limits);
  float accelerate_stopping_distance = 0.5 * (v0 + v_accel) * dt + getStoppingDistance(v_accel, limits);


  // Make sure we have enough distnace to stop with current speed
  float toc_velocity = 0;
  if (cruise_stopping_distance < distance_left) {
      // Check if have enough distance if we accelerate
      if (accelerate_stopping_distance < distance_left) {
        // Check if we are already at desired final speed or max spee
        if (v0 < vf) {
            toc_velocity = max<float>(v0 + dv_a, limits.max_speed);
        }
      }
  }
  else {
    // If not enough stopping distance at current speed
    if (v0 > 0) {
      // If we have nonzero velocity
      toc_velocity = max<float>(v0 - dv_d, 0.0);
    }
  }

  // Add debugging printout here
  bool debug = true;
  if (debug) {
    std::ofstream file("1DTOC.txt"); // Using constructor to open the file
    file << "x0 " << x0 << " v0 " << v0 << " xf " << xf << " vf " << vf << " toc_v" << toc_velocity << '\n';
    file.close();
  }

  return toc_velocity;
}


float getStoppingDistance(const float v, const MotionLimits &limits) {
  return Sq(v) / (2 * limits.max_decceleration); // return meters
}
