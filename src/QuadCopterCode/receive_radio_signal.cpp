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
    PCintPort::attachInterrupt(PIN_LEFT_STICK, &Falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &Falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &Falling, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &Falling, RISING);
    PCintPort::attachInterrupt(PIN_LEFT_KNOB, &Falling, RISING);
}

// --------------------------------------------------------------------
// Utility Functions and ISRs (interrupt service routines)
// --------------------------------------------------------------------
/**************************************************************
 * Function: RunningAverage
**************************************************************/
float RunningAverage(int new_value, int interrupt_val_index)
{
#define AVERAGE_BUFFER_SIZE 15

    static int LM[AVERAGE_BUFFER_SIZE][NUM_INPUTS]; // LastMeasurements
    static byte index[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static long sum[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static byte count = 0;

    // keep sum updated to improve speed.
    sum[interrupt_val_index] -= LM[index[interrupt_val_index]][interrupt_val_index];
    LM[index[interrupt_val_index]][interrupt_val_index] = new_value;
    sum[interrupt_val_index] += LM[index[interrupt_val_index]][interrupt_val_index];
    index[interrupt_val_index]++;
    index[interrupt_val_index] = index[interrupt_val_index] % AVERAGE_BUFFER_SIZE;
    if (count < AVERAGE_BUFFER_SIZE)
    {
        count++;
    }

    return sum[interrupt_val_index] / count;
}

/**************************************************************
 * Function: Rising (ISR - interrupt service routine)
**************************************************************/
void Rising()
{
    PCintPort::attachInterrupt(PCintPort::arduinoPin, &Falling, FALLING);

    const uint8_t interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin];

    last_rise_time[interrupt_val_index] = micros();
    // if (interrupt_val_index == INDEX_LEFT_STICK)
    // {
    //     Serial.print(F("| RISING Interrupt = "));
    //     Serial.print(last_rise_time[interrupt_val_index]);
    //     Serial.print(F("\t"));
    // }
}

/**************************************************************
 * Function: Falling (ISR - interrupt service routine)
**************************************************************/
void Falling()
{

    PCintPort::attachInterrupt(PCintPort::arduinoPin, &Rising, RISING);

    const uint8_t interrupt_val_index = PIN_TO_CMD_VEC[PCintPort::arduinoPin]; // Translate pin # to index

    const unsigned long pwm_val = micros() - last_rise_time[interrupt_val_index];
    // scaled_val = map(pwm_val, minIns[interrupt_val_index], maxIns[interrupt_val_index],
    //                                                      minOuts[interrupt_val_index]*factor, maxOuts[interrupt_val_index]*factor)/factor;

    const float scaled_val = pwm_val;
    //  map(pwm_val, minIns[interrupt_val_index], maxIns[interrupt_val_index],
    //                              minOuts[interrupt_val_index], maxOuts[interrupt_val_index]);

    input_radio_values[interrupt_val_index] = RunningAverage(scaled_val, interrupt_val_index);

    // if (interrupt_val_index == INDEX_LEFT_STICK)
    // {
    //     Serial.print(F("| FALLING average = "));
    //     Serial.println(radioRecieverVals[interrupt_val_index]);
    //     Serial.println(F("\t"));
    // }
}
