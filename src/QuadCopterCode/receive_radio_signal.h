#include <Arduino.h>

/**************************************************************
 * Function: setupRadioReceiver
**************************************************************/
void setupRadioReceiver(volatile float *radioVals);

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------
float runningAverage(int M, int latest_interrupted_pin);

// For PWM reading on multiple channels
void rising();

// For PWM reading on multiple channels
void falling();

// For PWM reading on multiple channels
void rising_calibration();

// For PWM reading on multiple channels
void falling_calibration();