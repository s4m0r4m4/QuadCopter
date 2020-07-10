#include <Arduino.h>

#ifndef QUADCOPTER_CONSTANTS_H
#define QUADCOPTER_CONSTANTS_H

#define LED_STABLE 8
// #define PIN_LEFT_STICK_LEFTRIGHT 8???
#define PIN_LEFT_STICK 9
#define PIN_RIGHT_STICK_UPDOWN 10
#define PIN_RIGHT_STICK_LEFTRIGHT 11
#define PIN_LEFT_KNOB 2
#define PIN_RIGHT_KNOB 5
#define NUM_PINS 12 // Number of potential input pins

static const float DEG2RAD = PI / 180.0f;

#endif