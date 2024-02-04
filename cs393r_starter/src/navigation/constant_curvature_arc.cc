#include "constant_curvature_arc.h"
#include "motion_primitives.h"

namespace navigation {
void ConstantCurvatureArc::getControlOnCurve(
  const navigation::MotionLimits& linear_limits,
  const float linear_vel,
  const float dt,
  float& cmd_linear_vel
) {
  /**
    * @brief Get the control on the curve
    * 
    * @param linear_limits 
    * @param linear_vel 
    * @param cmd_linear_vel
  **/
  cmd_linear_vel = run1DTOC(linear_limits, 
    0, 
    linear_vel,
    arc_length_,
    cmd_linear_vel,
    dt
  );
}
} // namespace navigation
