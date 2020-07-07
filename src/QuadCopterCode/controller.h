#include <Arduino.h>

/**************************************************************
 * Function: calculateControlVector
**************************************************************/
void calculateControlVector();

/**************************************************************
 * Function: checkForActiveSignal
**************************************************************/
bool checkForActiveSignal();

float thrustToMotorValNonlinear(float deltaThrust, float val0);

/**************************************************************
 * Function: motorValToThrustNonlinear
**************************************************************/
float motorValToThrustNonlinear(float val0);