#ifndef NAVIGATION_PARAMS_H
#define NAVIGATION_PARAMS_H

namespace navigation {
struct MotionLimits {
  float max_acceleration;
  float max_decceleration;
  float max_speed;

  MotionLimits(float max_acceleration, float max_decceleration, float max_speed)
      : max_acceleration(max_acceleration),
        max_decceleration(max_decceleration),
        max_speed(max_speed) {}
};

struct NavigationParams {
  double dt;
  MotionLimits linear_limits;
  MotionLimits angular_limits;
  double goal_tolerance;

  NavigationParams() :
    dt(0.05),
    linear_limits(1.0, 1.0, 1.0),
    angular_limits(0.5, 0.5, 0.5),
    goal_tolerance(1.0) {}
};
}  // namespace navigation
#endif  // NAVIGATION_PARAMS_H
