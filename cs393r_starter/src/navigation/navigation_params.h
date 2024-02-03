

struct MotionLimits {
    float max_acceleration;
    float max_decceleration;
    float max_speed;

    MotionLimits(float max_acceleration, float max_decceleration, float max_speed) :
       max_acceleration(max_acceleration), max_decceleration(max_decceleration), max_speed(max_speed) {}
};