#include <Arduino.h>
//#include <EnableInterrupt.h>
#include <PinChangeInt.h> //http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
#include <quadcopter_constants.h>
#include <global_junk.h>
#include <receive_radio_signal.h>

#define MAX_ANGLE_COMMAND 10 // degrees
#define ZEROS_ALL_INPUTS \
    {                    \
        0, 0, 0, 0, 0,   \
    }

#define RADIO_STICK_DIFF = 370;

const float factor = 1.0;

uint8_t PIN_TO_CMD_VEC[NUM_PINS];

// All of these arrays are the same ordering
const int minIns[NUM_INPUTS] = {
    877,  // LEFT KNOB
    1000, // RIGHT KNOB
    1175, // LEFT STICK
    1310, // RIGHT STICK UP-DOWN
    1268  // RIGHT STICK LEFT-RIGHT
};
const int maxIns[NUM_INPUTS] = {
    1880,
    1995,
    1810,
    1820,
    2020};
const int minOuts[NUM_INPUTS] = {
    0,
    0,
    0,
    -MAX_ANGLE_COMMAND,
    -MAX_ANGLE_COMMAND};
const int maxOuts[NUM_INPUTS] = {
    100,
    100,
    180,
    MAX_ANGLE_COMMAND,
    MAX_ANGLE_COMMAND};

/**************************************************************
 * Function: setupRadioReceiver
**************************************************************/
void setupRadioReceiver(volatile float *radioRecieverVals)
{
    // Setup pin-to-index mapping
    PIN_TO_CMD_VEC[PIN_LEFT_KNOB] = INDEX_LEFT_KNOB;
    PIN_TO_CMD_VEC[PIN_RIGHT_KNOB] = INDEX_RIGHT_KNOB;
    PIN_TO_CMD_VEC[PIN_LEFT_STICK] = INDEX_LEFT_STICK;
    PIN_TO_CMD_VEC[PIN_RIGHT_STICK_LEFTRIGHT] = INDEX_RIGHT_STICK_LEFTRIGHT;
    PIN_TO_CMD_VEC[PIN_RIGHT_STICK_UPDOWN] = INDEX_RIGHT_STICK_UPDOWN;

    pinMode(PIN_LEFT_STICK, INPUT);
    pinMode(PIN_RIGHT_STICK_UPDOWN, INPUT);
    pinMode(PIN_RIGHT_STICK_LEFTRIGHT, INPUT);
    pinMode(PIN_RIGHT_KNOB, INPUT);
    pinMode(PIN_LEFT_KNOB, INPUT);

    // Initialize values
    for (int i = 0; i < NUM_INPUTS; i++)
    {
        last_rise_time[i] = micros();
        radioRecieverVals[i] = 0.0;
    }

    // Wait while calibration data is collected
    // Serial.println(F("Gathering data from radio receiver for calibration..."));
    // PCintPort::attachInterrupt(PIN_LEFT_STICK, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &rising_calibration, RISING);
    // PCintPort::attachInterrupt(PIN_LEFT_KNOB, &rising_calibration, RISING);

    // int iii = 0;
    // while (iii<10000) {iii++; Serial.print(iii);}
    // delay(100);

    // Calibrate
    // TOOD: what was I doing here????????
    // minIns[PIN_RIGHT_STICK_UPDOWN] = 1310;    //constrain(pwm_val_array[PIN_RIGHT_STICK_UPDOWN] - RADIO_STICK_DIFF, 1000, 1600);
    // maxIns[PIN_RIGHT_STICK_UPDOWN] = 1820;    //constrain(pwm_val_array[PIN_RIGHT_STICK_UPDOWN] + RADIO_STICK_DIFF, 1610, 2000); // = 1820;
    // minIns[PIN_RIGHT_STICK_LEFTRIGHT] = 1268; //constrain(pwm_val_array[PIN_RIGHT_STICK_LEFTRIGHT] - RADIO_STICK_DIFF, 1000, 1600); //1268;
    // maxIns[PIN_RIGHT_STICK_LEFTRIGHT] = 2020; //constrain(pwm_val_array[PIN_RIGHT_STICK_LEFTRIGHT] + RADIO_STICK_DIFF, 1610, 2000); //2020;

    // Set up normal interrupts for operation
    Serial.println(F("Beginning normal radio reciever listener..."));
    PCintPort::attachInterrupt(PIN_LEFT_STICK, &falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &falling, RISING);
    PCintPort::attachInterrupt(PIN_LEFT_KNOB, &falling, RISING);
}

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------
float runningAverage(int M, int interrupt_val_index)
{
#define LM_SIZE 15
    static int LM[LM_SIZE][NUM_INPUTS]; // LastMeasurements
    static byte index[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static long sum[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static byte count = 0;

    // keep sum updated to improve speed.
    sum[interrupt_val_index] -= LM[index[interrupt_val_index]][interrupt_val_index];
    LM[index[interrupt_val_index]][interrupt_val_index] = M;
    sum[interrupt_val_index] += LM[index[interrupt_val_index]][interrupt_val_index];
    index[interrupt_val_index]++;
    index[interrupt_val_index] = index[interrupt_val_index] % LM_SIZE;
    if (count < LM_SIZE)
        count++;

    return sum[interrupt_val_index] / count;
}

// For PWM reading on multiple channels
void rising()
{
    uint8_t interrupt_val_index;

    // Serial.print("Rising Pin: ");
    // Serial.print(PCintPort::arduinoPin);

    interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin];
    // Serial.print(" - Interrupt: ");
    // Serial.println(interrupt_val_index);

    last_rise_time[interrupt_val_index] = micros();
    if (interrupt_val_index == INDEX_LEFT_STICK)
    {
        Serial.print(F("| RISING Interrupt = "));
        Serial.print(last_rise_time[interrupt_val_index]);
        Serial.print(F("\t"));
    }

    PCintPort::attachInterrupt(interrupt_val_index, &falling, FALLING);
}

// For PWM reading on multiple channels
void falling()
{
    ;

    // Serial.print("Falling Pin: ");
    // Serial.print(PCintPort::arduinoPin);

    const uint8_t interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin]; // Translate pin # to index

    // Serial.print(" - Interrupt: ");
    // Serial.println(interrupt_val_index);

    const unsigned long nowish = micros();
    const unsigned long pwm_val = nowish - last_rise_time[interrupt_val_index];
    // scaled_val = map(pwm_val, minIns[interrupt_val_index], maxIns[interrupt_val_index],
    //                                                      minOuts[interrupt_val_index]*factor, maxOuts[interrupt_val_index]*factor)/factor;

    const float scaled_val = map(pwm_val, minIns[interrupt_val_index], maxIns[interrupt_val_index],
                                 minOuts[interrupt_val_index], maxOuts[interrupt_val_index]);

    radioRecieverVals[interrupt_val_index] = runningAverage(scaled_val, interrupt_val_index);

    if (interrupt_val_index == INDEX_LEFT_STICK)
    {
        Serial.print(F("| FALLING average = "));
        Serial.println(radioRecieverVals[interrupt_val_index]);
        Serial.println(F("\t"));
    }

    PCintPort::attachInterrupt(interrupt_val_index, &rising, RISING);
}

// For PWM reading on multiple channels
// void rising_calibration()
// {
//     uint8_t interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin]; // Translate pin # to index
//     PCintPort::attachInterrupt(interrupt_val_index, &falling_calibration, FALLING);
//     prev_times[interrupt_val_index] = micros();
//     // Serial.print(F("prev_times[")); Serial.print(interrupt_val_index); Serial.print(F("] = ")); Serial.print(prev_times[interrupt_val_index]);
// }

// // For PWM reading on multiple channels
// void falling_calibration()
// {

//     uint8_t interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin]; // Translate pin # to index
//     PCintPort::attachInterrupt(interrupt_val_index, &rising_calibration, RISING);
//     pwm_val_array[interrupt_val_index] = micros() - prev_times[interrupt_val_index]; //runningAverage(, interrupt_val_index);
//                                                                                      // Serial.print(F("pwm_val_array[")); Serial.print(interrupt_val_index); Serial.print(F("] = ")); Serial.print(pwm_val_array[interrupt_val_index]);
//                                                                                      // if (interrupt_val_index==9){
//                                                                                      // Serial.print(F("micros=")); Serial.print(nowish); Serial.print(F("\t")); Serial.print(F("prev_time[")); Serial.print(interrupt_val_index); Serial.print(F("]="));
//                                                                                      // Serial.print(prev_times[interrupt_val_index]); Serial.print(F("\t"));
//                                                                                      // if (interrupt_val_index==PIN_RIGHT_STICK_LEFTRIGHT){
//                                                                                      //   Serial.print(F("pwm_val = ")); Serial.print(pwm_val); Serial.print(F("/")); Serial.println(scaled_val);
//                                                                                      // }
//                                                                                      // }
// }
