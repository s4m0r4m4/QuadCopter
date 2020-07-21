#include <Arduino.h>

/**************************************************************
 * Function: calculateControlVector
**************************************************************/
void calculateControlVector(float *euler_angles, float *g, float *motor_control_vector, float delta_time);

/**************************************************************
 * Function: checkForActiveSignal
**************************************************************/
bool checkForActiveSignal();