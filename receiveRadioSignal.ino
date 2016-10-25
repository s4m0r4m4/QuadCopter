

int pinKnobUpLeft = 11;
int pin10 = 10;
int pin9 = 9;
int pinLeftThrottle = 9;
int pinRightThrottleUpDown = 10;
int pinRightThrottleLeftRight = 11;


// ###############################################################################################       
void readRecieverData() //
{ 
  int timeout = 10000000;
//  val11 = pulseIn(pinKnobUpLeft, HIGH, timeout);
//  Serial.print(val11); Serial.print("\t");
//  val10 = pulseIn(pin10, HIGH, timeout);
//  Serial.print(val10); Serial.print("\t");
//  val9 = pulseIn(pin9, HIGH, timeout);
//  Serial.print(val9); Serial.print("\t");
  valLeftThrottle = map((pulseIn(pinLeftThrottle, HIGH, timeout)),1150,1850,0,179);

//  valLeftThrottle = pulseIn(pinLeftThrottle, HIGH, timeout);
//  Serial.print(valLeftThrottle); Serial.print("\n");
//  val5 = pulseIn(pinRightThrottleUpDown, HIGH, timeout);
//  Serial.print(pinRightThrottleUpDown); Serial.print("\t");
//  val3 = pulseIn(pinRightThrottleLeftRight, HIGH, timeout);
//  Serial.println(pinRightThrottleLeftRight);
}


void setupRadioReceiver()
{  
//  pinMode(pinKnobUpLeft, INPUT);  
//  pinMode(pin10, INPUT);
//  pinMode(pin9, INPUT);
  pinMode(pinLeftThrottle, INPUT);  
//  pinMode(pinRightThrottleUpDown, INPUT);
//  pinMode(pinRightThrottleLeftRight, INPUT);
}

