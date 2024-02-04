#pragma once

#include "navigation_params.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include "shared/math/math_util.h"

namespace navigation {
float run1DTOC(const MotionLimits &limits, const float x0, const float v0,
               const float xf, const float vf,
               const float dt = 0.05);
}