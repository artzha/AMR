#include "eigen3/Eigen/Dense"
#include "motion_primitives.h"

using Eigen::Vector2f;

namespace navigation {
class ConstantCurvatureArc {
  public:
    ConstantCurvatureArc(float curvature, float arc_length): 
      curvature_(curvature), arc_length_(arc_length) {}

    void getControlOnCurve(
      const navigation::MotionLimits& linear_limits,
      const float linear_vel,
      const float dt,
      float& cmd_linear_vel
    );

    float curvature() const { return curvature_; }

    float arc_length() const { return arc_length_; }
  
  private:
    float curvature_;
    float arc_length_;

};
}