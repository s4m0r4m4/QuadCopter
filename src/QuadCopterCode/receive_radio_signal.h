#include <Arduino.h>

#ifndef RADIO_H
#define RADIO_H

/**************************************************************
 * Function: SetupRadioReceiver
**************************************************************/
void SetupRadioReceiver(volatile float *radioVals);

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------
float RunningAveragePWM(int M, int latest_interrupted_pin);

// For PWM reading on multiple channels
void Rising();

// For PWM reading on multiple channels
void Falling();

#endif