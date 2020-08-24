#include <Arduino.h>

#ifndef QUADCOPTER_CONSTANTS_H
#define QUADCOPTER_CONSTANTS_H

#define PIN_LED_LEVEL_INDICATOR 8 // pin to light LED if level

#define NUM_PINS 12  // Number of potential input pins
#define NUM_INPUTS 5 // Number of actually used inputs for radio

#define PIN_LEFT_STICK 9
#define INDEX_LEFT_STICK 2

#define PIN_RIGHT_STICK_UPDOWN 10
#define INDEX_RIGHT_STICK_UPDOWN 3

#define PIN_RIGHT_STICK_LEFTRIGHT 11
#define INDEX_RIGHT_STICK_LEFTRIGHT 4

#define PIN_LEFT_KNOB 2 // something's off with this
#define INDEX_LEFT_KNOB 0

#define PIN_RIGHT_KNOB 5
#define INDEX_RIGHT_KNOB 1

static const float DEG2RAD = PI / 180.0f;

#endif