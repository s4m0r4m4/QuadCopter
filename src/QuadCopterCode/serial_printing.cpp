#include <Arduino.h>

/**************************************************************
 * Function: serialPrintArray
**************************************************************/
void serialPrintArray(float *vec)
{
    Serial.print(vec[0]);
    Serial.print(F("\t"));
    Serial.print(vec[1]);
    Serial.print(F("\t"));
    Serial.print(vec[2]);
    Serial.print(F("\t"));
}

/**************************************************************
 * Function: serialPrintArray4
**************************************************************/
void serialPrintArray4(float *vec)
{
    Serial.print(vec[0]);
    Serial.print(F("\t"));
    Serial.print(vec[1]);
    Serial.print(F("\t"));
    Serial.print(vec[2]);
    Serial.print(F("\t"));
    Serial.print(vec[3]);
    Serial.print(F("\t"));
}

/**************************************************************
 * Function: serialPrintArrayLn
**************************************************************/
void serialPrintArrayLn(float *vec)
{
    serialPrintArray(vec);
    Serial.print("\n");
}