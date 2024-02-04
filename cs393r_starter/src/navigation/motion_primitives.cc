#include "motion_primitives.h"
#include <iostream>

using std::max;
using std::min;
using std::cout;
using std::endl;

namespace navigation {

float getStoppingDistance(const float v, const MotionLimits &limits) {
  return math_util::Sq(v) / (2 * limits.max_decceleration); // return meters
}

float run1DTOC(const MotionLimits &limits, const float x0, const float v0,
               const float xf, const float vf,
               const float dt // timestep
) {
  // Note This function only works with positive velocities
  char phase = '?'; // A: accelerate, D: deccelerate, C: cruise, X: stop

  float dv_a = limits.max_acceleration * dt;  // in m/s
  float dv_d = limits.max_decceleration * dt; // in m/s

  const float distance_left = xf - x0;
  const float v0_stopping_distance = getStoppingDistance(v0, limits);
  if (distance_left <= 0 || v0_stopping_distance > distance_left) {
    phase = 'X';
    cout << "WARNING!! OVERSHOOT" << "left dist: " << distance_left
         << " v0_stopping_distance" << v0_stopping_distance;
    return max<float>(0, v0 - dv_d);
  }

  float v_accel = min(v0 + dv_a, limits.max_speed);

  float cruise_stopping_distance = v0 * dt + v0_stopping_distance;
  float accelerate_stopping_distance = 0.5 * (v0 + v_accel) * dt + getStoppingDistance(v_accel, limits);

  cout << "\n" << v0 << " " << distance_left << " " << cruise_stopping_distance << " " << accelerate_stopping_distance << "\n" << endl;

  float toc_velocity = 0;
  if (cruise_stopping_distance < distance_left) {
      // Check if have enough distance if we accelerate
      if (accelerate_stopping_distance < distance_left) {
        // Check if we are already at desired final speed or max spee
        phase = 'A';
        toc_velocity = min<float>(v0 + dv_a, limits.max_speed);
      }
      else {
        phase = 'C';
        toc_velocity = v0;
      }
  }
  else {
    // If not enough stopping distance at current speed
    if (v0 > 0) {
      // If we have nonzero velocity
      phase = 'D';
      toc_velocity = max<float>(v0 - dv_d, 0.0);
    }
  }

  // Add debugging printout here
  // bool debug = true;
  if (true) {
    std::ofstream file("1DTOC.txt"); // Using constructor to open the file
    printf("\nPhase: %c\n", phase);
    file << "x0 " << x0 << " v0 " << v0 << " xf " << xf << " vf " << vf << " toc_v" << toc_velocity << '\n';
    file.close();
  }

  return toc_velocity;
}

} // namesapace navigation