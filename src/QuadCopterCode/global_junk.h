#include <Arduino.h>
#include <quadcopter_constants.h>

// TODO: remove these!

#ifndef GLOBAL_JUNK_H
#define GLOBAL_JUNK_H

#define DEBUG_MODE true

extern volatile unsigned long last_rise_time[NUM_INPUTS];
extern volatile float input_radio_values[NUM_INPUTS];

extern bool is_initializing;

#endif