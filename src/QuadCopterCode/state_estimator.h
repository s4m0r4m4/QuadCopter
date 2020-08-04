#include <Arduino.h>

void UpdateState(float *a, float *g, float *m, float *q, float delta_time, float *euler_angles);

// TODO: remove these for inline functions?
void MahonyQuaternionUpdate(float *a, float *g, float *m, float *q, float delta_time);
void MadgwickQuaternionUpdate(float *a, float *g, float *m, float *q, float delta_time);