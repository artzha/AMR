#include "constant_curvature_arc.h"

#include "motion_primitives.h"

namespace motion_primitives {
void ConstantCurvatureArc::getControlOnCurve(
    const navigation::MotionLimits& linear_limits,
    const float linear_vel,
    const float dt,
    float& cmd_linear_vel) {
  /**
   * @brief Get the control on the curve
   *
   * @param linear_limits
   * @param linear_vel
   * @param cmd_linear_vel
   **/
  // TODO: create a new linear_limits
  // linear_accel_limit = min(linear_limits.max, 1 / curvature * angular_limits.max)

  cmd_linear_vel =
      run1DTOC(linear_limits, 0, linear_vel, arc_length_, cmd_linear_vel, dt);
}
}  // namespace motion_primitives
