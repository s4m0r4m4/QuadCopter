#include <Arduino.h>
//#include <EnableInterrupt.h>
#include <PinChangeInt.h> //http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
#include <quadcopter_constants.h>

#define MAX_ANGLE_COMMAND 10
#define ZEROS_ALL_PINS                     \
    {                                      \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 \
    }

const float factor = 1.0;
float timeout = 32000;
float scaled_val;

volatile unsigned long pwm_val_array[NUM_PINS];
unsigned long minIns[NUM_PINS];
unsigned long maxIns[NUM_PINS];
int minOuts[NUM_PINS];
int maxOuts[NUM_PINS];

/**************************************************************
 * Function: setupRadioReceiver
**************************************************************/
void setupRadioReceiver()
{
    // Initialize values
    for (int i = 0; i < NUM_PINS; i++)
    {
        prev_times[i] = 0;
        radioRecieverVals[i] = 0.0;
    }

    minIns[PIN_LEFT_STICK] = 1175;
    maxIns[PIN_LEFT_STICK] = 1810;
    minOuts[PIN_LEFT_STICK] = 0;
    maxOuts[PIN_LEFT_STICK] = 180;
    minOuts[PIN_RIGHT_STICK_UPDOWN] = -MAX_ANGLE_COMMAND;
    maxOuts[PIN_RIGHT_STICK_UPDOWN] = MAX_ANGLE_COMMAND;
    minOuts[PIN_RIGHT_STICK_LEFTRIGHT] = -MAX_ANGLE_COMMAND;
    maxOuts[PIN_RIGHT_STICK_LEFTRIGHT] = MAX_ANGLE_COMMAND;
    minOuts[PIN_RIGHT_KNOB] = 0.0;
    maxOuts[PIN_RIGHT_KNOB] = 100.0;
    minOuts[PIN_LEFT_KNOB] = 0.0;
    maxOuts[PIN_LEFT_KNOB] = 100.0;
    minIns[PIN_RIGHT_KNOB] = 1000;
    maxIns[PIN_RIGHT_KNOB] = 1995;
    minIns[PIN_LEFT_KNOB] = 877;
    maxIns[PIN_LEFT_KNOB] = 1880;

    minIns[PIN_RIGHT_STICK_UPDOWN] = 1310;
    maxIns[PIN_RIGHT_STICK_UPDOWN] = 1820;
    minIns[PIN_RIGHT_STICK_LEFTRIGHT] = 1268;
    maxIns[PIN_RIGHT_STICK_LEFTRIGHT] = 2020;

    pinMode(PIN_LEFT_STICK, INPUT);
    pinMode(PIN_RIGHT_STICK_UPDOWN, INPUT);
    pinMode(PIN_RIGHT_STICK_LEFTRIGHT, INPUT);
    pinMode(PIN_RIGHT_KNOB, INPUT);
    pinMode(PIN_LEFT_KNOB, INPUT);

    // Wait while calibration data is collected
    // Serial.println("Gathering data from radio receiver for calibration...");
    // PCintPort::attachInterrupt(PIN_LEFT_STICK, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_LEFT_KNOB, &rising_calibration, RISING);

    // int iii = 0;
    // while (iii<10000) {iii++; Serial.print(iii);}
    // delay(100);

    // Calibrate
    Serial.print("Avg Val (RightUpDown) = ");
    Serial.println(pwm_val_array[PIN_RIGHT_STICK_UPDOWN]);
    Serial.print("Avg Val (RightLeftRight) = ");
    Serial.println(pwm_val_array[PIN_RIGHT_STICK_LEFTRIGHT]);
    minIns[PIN_RIGHT_STICK_UPDOWN] = 1310;    //constrain(pwm_val_array[PIN_RIGHT_STICK_UPDOWN] - RADIO_STICK_DIFF, 1000, 1600);
    maxIns[PIN_RIGHT_STICK_UPDOWN] = 1820;    //constrain(pwm_val_array[PIN_RIGHT_STICK_UPDOWN] + RADIO_STICK_DIFF, 1610, 2000); // = 1820;
    minIns[PIN_RIGHT_STICK_LEFTRIGHT] = 1268; //constrain(pwm_val_array[PIN_RIGHT_STICK_LEFTRIGHT] - RADIO_STICK_DIFF, 1000, 1600); //1268;
    maxIns[PIN_RIGHT_STICK_LEFTRIGHT] = 2020; //constrain(pwm_val_array[PIN_RIGHT_STICK_LEFTRIGHT] + RADIO_STICK_DIFF, 1610, 2000); //2020;

    // Set up normal interrupts for operation
    Serial.println("Beginning normal radio reciever listener...");
    PCintPort::attachInterrupt(PIN_LEFT_STICK, &rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &rising, RISING);
    PCintPort::attachInterrupt(PIN_LEFT_KNOB, &rising, RISING);
}

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------

float runningAverage(int M, int latest_interrupted_pin)
{
    #define LM_SIZE 15
    static int LM[LM_SIZE][NUM_PINS]; // LastMeasurements
    static byte index[NUM_PINS] = ZEROS_ALL_PINS;
    static long sum[NUM_PINS] = ZEROS_ALL_PINS;
    static byte count = 0;

    // keep sum updated to improve speed.
    sum[latest_interrupted_pin] -= LM[index[latest_interrupted_pin]][latest_interrupted_pin];
    LM[index[latest_interrupted_pin]][latest_interrupted_pin] = M;
    sum[latest_interrupted_pin] += LM[index[latest_interrupted_pin]][latest_interrupted_pin];
    index[latest_interrupted_pin]++;
    index[latest_interrupted_pin] = index[latest_interrupted_pin] % LM_SIZE;
    if (count < LM_SIZE)
        count++;

    return sum[latest_interrupted_pin] / count;
}

// For PWM reading on multiple channels
void rising()
{
    uint8_t latest_interrupted_pin;

    latest_interrupted_pin = PCintPort::arduinoPin;
    PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
    prev_times[latest_interrupted_pin] = micros();
    // if (latest_interrupted_pin==PIN_LEFT_STICK){
    //   Serial.print("| Interrupt = ");
    //   Serial.print(prev_times[latest_interrupted_pin]);
    //   Serial.print("\t");
    // }
    // Serial.print("Pin = "); Serial.print( latest_interrupted_pin); Serial.print("\t");
}

// For PWM reading on multiple channels
void falling()
{

    // TODO: can this be a const?
    uint8_t latest_interrupted_pin = PCintPort::arduinoPin; // Get the pin #
    PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
    unsigned long nowish = micros();
    unsigned long pwm_val = nowish - prev_times[latest_interrupted_pin];
    // scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
    //                                                      minOuts[latest_interrupted_pin]*factor, maxOuts[latest_interrupted_pin]*factor)/factor;

    scaled_val = map(pwm_val, minIns[latest_interrupted_pin], maxIns[latest_interrupted_pin],
                     minOuts[latest_interrupted_pin], maxOuts[latest_interrupted_pin]);
    radioRecieverVals[latest_interrupted_pin] = runningAverage(scaled_val, latest_interrupted_pin);

    // if (latest_interrupted_pin==9){
    // Serial.print("micros="); Serial.print(nowish); Serial.print("\t"); Serial.print("prev_time["); Serial.print(latest_interrupted_pin); Serial.print("]=");
    // Serial.print(prev_times[latest_interrupted_pin]); Serial.print("\t");
    // if (latest_interrupted_pin==PIN_RIGHT_STICK_LEFTRIGHT){
    //   Serial.print("pwm_val = "); Serial.print(pwm_val); Serial.print("/"); Serial.println(scaled_val);
    // }
    // }
}

// For PWM reading on multiple channels
void rising_calibration()
{
    uint8_t latest_interrupted_pin = PCintPort::arduinoPin;
    PCintPort::attachInterrupt(latest_interrupted_pin, &falling_calibration, FALLING);
    prev_times[latest_interrupted_pin] = micros();
    // Serial.print("prev_times["); Serial.print(latest_interrupted_pin); Serial.print("] = "); Serial.print(prev_times[latest_interrupted_pin]);
}

// For PWM reading on multiple channels
void falling_calibration()
{
    
    uint8_t latest_interrupted_pin = PCintPort::arduinoPin; // Get the pin #
    PCintPort::attachInterrupt(latest_interrupted_pin, &rising_calibration, RISING);
    pwm_val_array[latest_interrupted_pin] = micros() - prev_times[latest_interrupted_pin]; //runningAverage(, latest_interrupted_pin);
                                                                                           // Serial.print("pwm_val_array["); Serial.print(latest_interrupted_pin); Serial.print("] = "); Serial.print(pwm_val_array[latest_interrupted_pin]);
                                                                                           // if (latest_interrupted_pin==9){
                                                                                           // Serial.print("micros="); Serial.print(nowish); Serial.print("\t"); Serial.print("prev_time["); Serial.print(latest_interrupted_pin); Serial.print("]=");
                                                                                           // Serial.print(prev_times[latest_interrupted_pin]); Serial.print("\t");
                                                                                           // if (latest_interrupted_pin==PIN_RIGHT_STICK_LEFTRIGHT){
                                                                                           //   Serial.print("pwm_val = "); Serial.print(pwm_val); Serial.print("/"); Serial.println(scaled_val);
                                                                                           // }
                                                                                           // }
}
