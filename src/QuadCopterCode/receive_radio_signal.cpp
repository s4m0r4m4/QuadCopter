#include <Arduino.h>
//#include <EnableInterrupt.h>
#include <PinChangeInt.h> //http://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
#include <quadcopter_constants.h>
#include <global_junk.h>
#include <receive_radio_signal.h>
#include <math.h>

#define MAX_ANGLE_COMMAND 10 // degrees
#define ZEROS_ALL_INPUTS \
    {                    \
        0, 0, 0, 0, 0,   \
    }

// Radio initializing params
#define RADIO_CALIBRATION_COUNTER_LIMIT 250
uint16_t radio_calibration_counter = 0;

uint8_t PIN_TO_CMD_VEC[NUM_PINS];

// All of these arrays are the same ordering
int reciever_value_average[NUM_INPUTS] = {
    1375, // LEFT KNOB
    1500, // RIGHT KNOB
    1500, // LEFT STICK
    1565, // RIGHT STICK UP-DOWN
    1644  // RIGHT STICK LEFT-RIGHT
};

const int reciever_value_range[NUM_INPUTS] = {
    500, // LEFT KNOB
    500, // RIGHT KNOB
    318, // LEFT STICK
    255, // RIGHT STICK UP-DOWN
    376  // RIGHT STICK LEFT-RIGHT
};

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
 * Function: SetupRadioReceiver
**************************************************************/
void SetupRadioReceiver(volatile float *radioRecieverVals)
{
    // TODO: move to const setup like others above?
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

    // Set up interrupts to read radio PWM signals
    PCintPort::attachInterrupt(PIN_LEFT_STICK, &Rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_UPDOWN, &Rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_STICK_LEFTRIGHT, &Rising, RISING);
    PCintPort::attachInterrupt(PIN_RIGHT_KNOB, &Rising, RISING);
    PCintPort::attachInterrupt(PIN_LEFT_KNOB, &Rising, RISING);
}

/**************************************************************
 * Function: RunningAveragePWM
**************************************************************/
float RunningAveragePWM(long new_value, uint8_t interrupt_val_index)
{
#define BUFFER_LENGTH 15

    static int last_measurements[BUFFER_LENGTH][NUM_INPUTS];
    static byte index[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static long sum[NUM_INPUTS] = ZEROS_ALL_INPUTS;
    static byte count = 0;

    // keep sum updated to improve speed.
    sum[interrupt_val_index] -= last_measurements[index[interrupt_val_index]][interrupt_val_index];
    last_measurements[index[interrupt_val_index]][interrupt_val_index] = new_value;
    sum[interrupt_val_index] += last_measurements[index[interrupt_val_index]][interrupt_val_index];
    index[interrupt_val_index]++;
    index[interrupt_val_index] = index[interrupt_val_index] % BUFFER_LENGTH;
    if (count < BUFFER_LENGTH)
    {
        count++;
    }

    return sum[interrupt_val_index] / count;
}

/**************************************************************
 * Function: ExponentialMovingAverage
**************************************************************/
float ExponentialMovingAverage(long old_value, long new_value)
{

    const float weight_decay_factor = 0.85;

    return old_value * (weight_decay_factor) + new_value * (1 - weight_decay_factor);
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

    const unsigned long raw_pwm_value = micros() - last_rise_time[interrupt_val_index];

    // If we are initializing, adjust the average reciever value towards the current value
    if (is_initializing &&
        ((interrupt_val_index == INDEX_RIGHT_STICK_LEFTRIGHT) ||
         (interrupt_val_index == INDEX_RIGHT_STICK_UPDOWN)))
    {

        radio_calibration_counter++;

        // Keep a running average
        reciever_value_average[interrupt_val_index] =
            ExponentialMovingAverage(
                reciever_value_average[interrupt_val_index],
                raw_pwm_value);

        // Serial.print(interrupt_val_index);
        // Serial.print(" | NEW val: ");
        // Serial.print(raw_pwm_value);
        // Serial.print(" | AVG value: ");
        // Serial.println(reciever_value_average[interrupt_val_index]);

        if (radio_calibration_counter > RADIO_CALIBRATION_COUNTER_LIMIT)
        {
            is_initializing = false;
        }
    }

    const long scaled_val = map(raw_pwm_value,
                                reciever_value_average[interrupt_val_index] - reciever_value_range[interrupt_val_index],
                                reciever_value_average[interrupt_val_index] + reciever_value_range[interrupt_val_index],
                                minOuts[interrupt_val_index],
                                maxOuts[interrupt_val_index]);

    // TODO: consider another exponential moving average instead
    input_radio_values[interrupt_val_index] = RunningAveragePWM(scaled_val, interrupt_val_index);

    // if (interrupt_val_index == INDEX_LEFT_STICK)
    // {
    //     Serial.print(F("| FALLING average = "));
    //     Serial.println(input_radio_values[interrupt_val_index]);
    //     Serial.println(F(" | \t"));
    // }
}
