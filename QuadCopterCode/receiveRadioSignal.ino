#include <Arduino.h>

// #include <Average.h>

//int pinKnobUpLeft = 11;
#define pinLeftThrottle 9
#define pinRightThrottleUpDown 10
#define pinRightThrottleLeftRight 11
float scaled_val;


// ###############################################################################################
void setupRadioReceiver()
{
//  pinMode(pinKnobUpLeft, INPUT);
//  pinMode(pin10, INPUT);
//  pinMode(pin9, INPUT);
  pinMode(pinLeftThrottle, INPUT);
  pinMode(pinRightThrottleUpDown, INPUT);
  pinMode(pinRightThrottleLeftRight, INPUT);
}

// ###############################################################################################
void readRecieverData() //
{
 float timeout = 32000;
 float factor = 10;

 float raw_val;
//  val11 = pulseIn(pinKnobUpLeft, HIGH, timeout);
//  Serial.print(val11); Serial.print("\t");
//  val10 = pulseIn(pin10, HIGH, timeout);
//  Serial.print(val10); Serial.print("\t");
//  val9 = pulseIn(pin9, HIGH, timeout);
//  Serial.print(val9); Serial.print("\t");

  raw_val = (float)pulseIn(pinLeftThrottle, HIGH, timeout);
  scaled_val = map(raw_val, 1150, 1850, 0*factor, 179*factor)/factor;
  valLeftThrottle = runningAverage(scaled_val);
//  Serial.print(scaled_val); Serial.print("\t");
//  Serial.print(valLeftThrottle); Serial.print("\t");

//  valLeftThrottle = (int)(map(pulseIn(pinLeftThrottle, HIGH, timeout)*0.075 + valLeftThrottle_last1*0.525 + valLeftThrottle_last2*0.4), 1150, 1850, 90, 180));

//  Serial.print(valLeftThrottle); Serial.print("\n");
  valRightThrottleUpDown = pulseIn(pinRightThrottleUpDown, HIGH, timeout);
//  Serial.print(pinRightThrottleUpDown); Serial.print("\t");
  valRightThrottleLeftRight = pulseIn(pinRightThrottleLeftRight, HIGH, timeout);
//  Serial.println(pinRightThrottleLeftRight);
}

float runningAverage(int M) {
  #define LM_SIZE 15
  static int LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;
  static long sum = 0;
  static byte count = 0;

  // keep sum updated to improve speed.
  sum -= LM[index];
  LM[index] = M;
  sum += LM[index];
  index++;
  index = index % LM_SIZE;
  if (count < LM_SIZE) count++;

  return sum / count;
}
