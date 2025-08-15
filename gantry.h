#ifndef GANTRY_H
#define GANTRY_H

#include <Arduino.h>
#include "GLOBALS.h"
#include <math.h>

// Function declarations
bool move(float dist_X_mm, float dist_Y_mm);
void fsm();
void isrEnc1();
void isrEnc2();

#endif
