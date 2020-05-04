#include <Arduino.h>
//#include <EnableInterrupt.h>
#include <PinChangeInt.h> //http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
// #include <Average.h>

#define MAX_ANGLE_COMMAND 10
#define ZEROS_ALL_PINS {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

const float factor = 1.0;
float timeout = 32000;
float scaled_val;


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

  minIns[pinLeftThrottle] = 1175;
  maxIns[pinLeftThrottle] = 1810;
  minOuts[pinLeftThrottle] = 0;
  maxOuts[pinLeftThrottle] = 180;
  minOuts[pinRightThrottleUpDown] = -MAX_ANGLE_COMMAND;
  maxOuts[pinRightThrottleUpDown] = MAX_ANGLE_COMMAND;
  minOuts[pinRightThrottleLeftRight] = -MAX_ANGLE_COMMAND;
  maxOuts[pinRightThrottleLeftRight] = MAX_ANGLE_COMMAND;
  minOuts[pinRightKnob] = 0.0;
  maxOuts[pinRightKnob] = 100.0;
  minOuts[pinLeftKnob] = 0.0;
  maxOuts[pinLeftKnob] = 100.0;
  minIns[pinRightKnob] = 1000;
  maxIns[pinRightKnob] = 1995;
  minIns[pinLeftKnob] = 877;
  maxIns[pinLeftKnob] = 1880;

  minIns[pinRightThrottleUpDown] = 1310;
  maxIns[pinRightThrottleUpDown] = 1820;
  minIns[pinRightThrottleLeftRight] = 1268;
  maxIns[pinRightThrottleLeftRight] = 2020;

  pinMode(pinLeftThrottle, INPUT);
  pinMode(pinRightThrottleUpDown, INPUT);
  pinMode(pinRightThrottleLeftRight, INPUT);
  pinMode(pinRightKnob, INPUT);
  pinMode(pinLeftKnob, INPUT);

  // Wait while calibration data is collected
  // Serial.println("Gathering data from radio receiver for calibration...");
  // PCintPort::attachInterrupt(pinLeftThrottle, &rising_calibration, RISING);
  // PCintPort::attachInterrupt(pinRightThrottleUpDown, &rising_calibration, RISING);
  // PCintPort::attachInterrupt(pinRightThrottleLeftRight, &rising_calibration, RISING);
  // PCintPort::attachInterrupt(pinRightKnob, &rising_calibration, RISING);
  // PCintPort::attachInterrupt(pinLeftKnob, &rising_calibration, RISING);

  // int iii = 0;
  // while (iii<10000) {iii++; Serial.print(iii);}
  // delay(100);

  // Calibrate
  Serial.print("Avg Val (RightUpDown) = "); Serial.println(pwm_val_array[pinRightThrottleUpDown]);
  Serial.print("Avg Val (RightLeftRight) = "); Serial.println(pwm_val_array[pinRightThrottleLeftRight]);
  minIns[pinRightThrottleUpDown] = 1310; //constrain(pwm_val_array[pinRightThrottleUpDown] - RADIO_STICK_DIFF, 1000, 1600);
  maxIns[pinRightThrottleUpDown] = 1820; //constrain(pwm_val_array[pinRightThrottleUpDown] + RADIO_STICK_DIFF, 1610, 2000); // = 1820;
  minIns[pinRightThrottleLeftRight] = 1268; //constrain(pwm_val_array[pinRightThrottleLeftRight] - RADIO_STICK_DIFF, 1000, 1600); //1268;
  maxIns[pinRightThrottleLeftRight] = 2020; //constrain(pwm_val_array[pinRightThrottleLeftRight] + RADIO_STICK_DIFF, 1610, 2000); //2020;

  // Set up normal interrupts for operation
  Serial.println("Beginning normal radio reciever listener...");
  PCintPort::attachInterrupt(pinLeftThrottle, &rising, RISING);
  PCintPort::attachInterrupt(pinRightThrottleUpDown, &rising, RISING);
  PCintPort::attachInterrupt(pinRightThrottleLeftRight, &rising, RISING);
  PCintPort::attachInterrupt(pinRightKnob, &rising, RISING);
  PCintPort::attachInterrupt(pinLeftKnob, &rising, RISING);
}

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------

float runningAverage(int M, int latest_interrupted_pin)
{
  #define LM_SIZE 15
  static int LM[LM_SIZE][NUM_PINS];      // LastMeasurements
  static byte index[NUM_PINS] = ZEROS_ALL_PINS;
  static long sum[NUM_PINS] = ZEROS_ALL_PINS;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum[latest_interrupted_pin] -= LM[index[latest_interrupted_pin]][latest_interrupted_pin];
  LM[index[latest_interrupted_pin]][latest_interrupted_pin] = M;
  sum[latest_interrupted_pin] += LM[index[latest_interrupted_pin]][latest_interrupted_pin];
  index[latest_interrupted_pin]++;
  index[latest_interrupted_pin] = index[latest_interrupted_pin] % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum[latest_interrupted_pin] / count;
}

// For PWM reading on multiple channels
void rising()
{
  latest_interrupted_pin = PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
  prev_times[latest_interrupted_pin] = micros();
  // if (latest_interrupted_pin==pinLeftThrottle){
  //   Serial.print("| Interrupt = ");
  //   Serial.print(prev_times[latest_interrupted_pin]);
  //   Serial.print("\t");
  // }
  // Serial.print("Pin = "); Serial.print( latest_interrupted_pin); Serial.print("\t");
}

// For PWM reading on multiple channels
void falling() {
  latest_interrupted_pin = PCintPort::arduinoPin; // Get the pin #
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
  unsigned long nowish = micros();
  pwm_val = nowish - prev_times[latest_interrupted_pin];
  // scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
  //                                                      minOuts[latest_interrupted_pin]*factor, maxOuts[latest_interrupted_pin]*factor)/factor;

  scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
                                                      minOuts[latest_interrupted_pin], maxOuts[latest_interrupted_pin]);
  radioRecieverVals[latest_interrupted_pin] = runningAverage(scaled_val, latest_interrupted_pin);

  // if (latest_interrupted_pin==9){
  // Serial.print("micros="); Serial.print(nowish); Serial.print("\t"); Serial.print("prev_time["); Serial.print(latest_interrupted_pin); Serial.print("]=");
  // Serial.print(prev_times[latest_interrupted_pin]); Serial.print("\t");
  // if (latest_interrupted_pin==pinRightThrottleLeftRight){
  //   Serial.print("pwm_val = "); Serial.print(pwm_val); Serial.print("/"); Serial.println(scaled_val);
  // }
  // }
}

// For PWM reading on multiple channels
void rising_calibration()
{
  latest_interrupted_pin = PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling_calibration, FALLING);
  prev_times[latest_interrupted_pin] = micros();
  // Serial.print("prev_times["); Serial.print(latest_interrupted_pin); Serial.print("] = "); Serial.print(prev_times[latest_interrupted_pin]);
}

// For PWM reading on multiple channels
void falling_calibration() {
  latest_interrupted_pin = PCintPort::arduinoPin; // Get the pin #
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising_calibration, RISING);
  pwm_val_array[latest_interrupted_pin] = micros() - prev_times[latest_interrupted_pin]; //runningAverage(, latest_interrupted_pin);
  // Serial.print("pwm_val_array["); Serial.print(latest_interrupted_pin); Serial.print("] = "); Serial.print(pwm_val_array[latest_interrupted_pin]);
  // if (latest_interrupted_pin==9){
  // Serial.print("micros="); Serial.print(nowish); Serial.print("\t"); Serial.print("prev_time["); Serial.print(latest_interrupted_pin); Serial.print("]=");
  // Serial.print(prev_times[latest_interrupted_pin]); Serial.print("\t");
  // if (latest_interrupted_pin==pinRightThrottleLeftRight){
  //   Serial.print("pwm_val = "); Serial.print(pwm_val); Serial.print("/"); Serial.println(scaled_val);
  // }
  // }
}
