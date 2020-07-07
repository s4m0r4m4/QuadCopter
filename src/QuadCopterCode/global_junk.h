#include <Arduino.h>
#include <quadcopter_constants.h>

// TODO: remove these!

extern volatile unsigned long prev_times[NUM_PINS];
extern volatile float radioRecieverVals[NUM_PINS];
extern volatile float valLeftThrottle;
extern volatile float valRightThrottleUpDown;
extern volatile float valRightThrottleLeftRight;

extern float integrated_pitch_err; // integral error for Controller
extern float integrated_roll_err;  // integral error for Controller