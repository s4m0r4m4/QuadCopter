#include <Arduino.h>

/**************************************************************
 * Function: setupRadioReceiver
**************************************************************/
void setupRadioReceiver(volatile float *radioVals);

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------
float RunningAverage(int M, int latest_interrupted_pin);

// For PWM reading on multiple channels
void Rising();

// For PWM reading on multiple channels
void Falling();