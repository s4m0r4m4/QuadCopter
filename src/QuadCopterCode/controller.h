#include <Arduino.h>

/**************************************************************
 * Function: calculateControlVector
**************************************************************/
void calculateControlVector(float *euler_angles, float *g, float *escControlVec, float delta_time);

/**************************************************************
 * Function: checkForActiveSignal
**************************************************************/
bool checkForActiveSignal();

float thrustToMotorValNonlinear(float deltaThrust, float val0);

/**************************************************************
 * Function: motorValToThrustNonlinear
**************************************************************/
float motorValToThrustNonlinear(float val0);