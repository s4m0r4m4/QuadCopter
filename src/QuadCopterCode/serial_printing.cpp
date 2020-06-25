#include <Arduino.h>

/**************************************************************
 * Function: serialPrintArray
**************************************************************/
void serialPrintArray(float *vec)
{
  Serial.print(vec[0]);
  Serial.print("\t");
  Serial.print(vec[1]);
  Serial.print("\t");
  Serial.print(vec[2]);
  Serial.print("\t");
}

/**************************************************************
 * Function: serialPrintArray4
**************************************************************/
void serialPrintArray4(float *vec)
{
  Serial.print(vec[0]);
  Serial.print("\t");
  Serial.print(vec[1]);
  Serial.print("\t");
  Serial.print(vec[2]);
  Serial.print("\t");
  Serial.print(vec[3]);
  Serial.print("\t");
}

/**************************************************************
 * Function: serialPrintArrayLn
**************************************************************/
void serialPrintArrayLn(float *vec)
{
  serialPrintArray(vec);
  Serial.print("\n");
}