#include <Arduino.h>
//#include <EnableInterrupt.h>
//#include <PinChangeInt.h> //http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
// #include <Average.h>

#define MAX_ANGLE_COMMAND 10
#define ZEROS_ALL_PINS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

const float factor = 1.0;
float timeout = 32000;
float scaled_val;

unsigned long minIns[NUM_PINS];
unsigned long maxIns[NUM_PINS];
int minOuts[NUM_PINS];
int maxOuts[NUM_PINS];


// ###############################################################################################
void setupRadioReceiver()
{
  // Initialize values
  for(int i=0; i<NUM_PINS; i++)
  {
    // pwm_values[i] = 0;
    prev_times[i] = 0;
    radioRecieverVals[i] = 0.0;
  }

  minIns[pinLeftThrottle] = 1170;
  maxIns[pinLeftThrottle] = 1850;
  minOuts[pinLeftThrottle] = 0;
  maxOuts[pinLeftThrottle] = 180;
  minIns[pinRightThrottleUpDown] = 1350;
  maxIns[pinRightThrottleUpDown] = 1710;
  minOuts[pinRightThrottleUpDown] = -MAX_ANGLE_COMMAND;
  maxOuts[pinRightThrottleUpDown] = MAX_ANGLE_COMMAND;
  minIns[pinRightThrottleLeftRight] = 1340;
  maxIns[pinRightThrottleLeftRight] = 1770;
  minOuts[pinRightThrottleLeftRight] = -MAX_ANGLE_COMMAND;
  maxOuts[pinRightThrottleLeftRight] = MAX_ANGLE_COMMAND;

  pinMode(pinLeftThrottle, INPUT);
  pinMode(pinRightThrottleUpDown, INPUT);
  pinMode(pinRightThrottleLeftRight, INPUT);

  //PCintPort::attachInterrupt(pinLeftThrottle, &rising, RISING);
  //PCintPort::attachInterrupt(pinRightThrottleUpDown, &rising, RISING);
  //PCintPort::attachInterrupt(pinRightThrottleLeftRight, &rising, RISING);
}

// ###############################################################################################
void readRecieverData() //
{
 // float raw_val;
 // Serial.print("[pinLeftThrottle] = "); Serial.print(pwm_values[pinLeftThrottle]); Serial.print("/");
 // Serial.print(radioRecieverVals[pinLeftThrottle]); Serial.print("\t");
 // Serial.print("[pinRightThrottleUpDown] = "); Serial.print(pwm_values[pinRightThrottleUpDown]); Serial.print("/");
 // Serial.print(radioRecieverVals[pinRightThrottleUpDown]); Serial.print("\t");
 // Serial.print("[pinRightThrottleLeftRight] = "); Serial.print(pwm_values[pinRightThrottleLeftRight]); Serial.print("/");
 // Serial.print(radioRecieverVals[pinRightThrottleLeftRight]); Serial.print("\t");

  //raw_val = pulseIn(pinRightThrottleUpDown, HIGH, timeout);
  // valRightThrottleUpDown = map(raw_val, 1350, 1650, -MAX_ANGLE_COMMAND, MAX_ANGLE_COMMAND);
  // valRightThrottleLeftRight = 1,0;//map(pulseIn(pinRightThrottleLeftRight, HIGH, timeout), 1350, 1710, -MAX_ANGLE_COMMAND, MAX_ANGLE_COMMAND);
}

//float runningAverage(int M, int latest_interrupted_pin)
//{
//  #define LM_SIZE 15
//  static int LM[LM_SIZE][NUM_PINS];      // LastMeasurements
//  static byte index[NUM_PINS] = ZEROS_ALL_PINS;
//  static long sum[NUM_PINS] = ZEROS_ALL_PINS;
//  static byte count = 0;
//
//  // keep sum updated to improve speed.
//  sum[latest_interrupted_pin] -= LM[index[latest_interrupted_pin]][latest_interrupted_pin];
//  LM[index[latest_interrupted_pin]][latest_interrupted_pin] = M;
//  sum[latest_interrupted_pin] += LM[index[latest_interrupted_pin]][latest_interrupted_pin];
//  index[latest_interrupted_pin]++;
//  index[latest_interrupted_pin] = index[latest_interrupted_pin] % LM_SIZE;
//  if (count < LM_SIZE) count++;
//
//  return sum[latest_interrupted_pin] / count;
//}
//
//// For PWM reading on multiple channels
//void rising()
//{
//  latest_interrupted_pin = //PCintPort::arduinoPin;
//  //PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
//  prev_times[latest_interrupted_pin] = micros();
//  // if (latest_interrupted_pin==pinLeftThrottle){
//  //   Serial.print("| Interrupt = ");
//  //   Serial.print(prev_times[latest_interrupted_pin]);
//  //   Serial.print("\t");
//  // }
//  // Serial.print("Pin = "); Serial.print( latest_interrupted_pin); Serial.print("\t");
//}
//
//// For PWM reading on multiple channels
//void falling() {
//  //latest_interrupted_pin = PCintPort::arduinoPin; // Get the pin #
//  //PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
//  unsigned long nowish = micros();
//  pwm_val = nowish-prev_times[latest_interrupted_pin];
//  // scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
//  //                                                      minOuts[latest_interrupted_pin]*factor, maxOuts[latest_interrupted_pin]*factor)/factor;
//
//  scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
//                                                      minOuts[latest_interrupted_pin], maxOuts[latest_interrupted_pin]);
//  radioRecieverVals[latest_interrupted_pin] = runningAverage(scaled_val, latest_interrupted_pin);
//
//
//  // if (latest_interrupted_pin==9){
//  // Serial.print("micros="); Serial.print(nowish); Serial.print("\t"); Serial.print("prev_time["); Serial.print(latest_interrupted_pin); Serial.print("]=");
//  // Serial.print(prev_times[latest_interrupted_pin]); Serial.print("\t");
//  // Serial.print("pwm_val = "); Serial.print(pwm_val); Serial.print("/"); Serial.println(scaled_val);
//  // }
//}
