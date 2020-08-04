#include <Arduino.h>

void CalculateControlVector(float *euler_angles, float *g, float *motor_control_vector, float delta_time);

bool checkForActiveSignal();