#include <Arduino.h>
#include "GLOBALS.h"
#include <math.h>

void isrEnc1() {
  uint8_t a = digitalRead(ENC1_A);
  uint8_t b = digitalRead(ENC1_B);
  uint8_t currentState = (b << 1) | a;
  uint8_t trans  = (lastState1 << 2) | currentState;
  lastState1 = currentState;

  if (trans==0b0001 || trans==0b0111 || trans==0b1110 || trans==0b1000) {
    encCount1++;
  }
  else if (trans==0b0010 || trans==0b1011 || trans==0b1101 || trans==0b0100) {
    encCount1--;
  }
}

void isrEnc2() {
  uint8_t a = digitalRead(ENC2_A);
  uint8_t b = digitalRead(ENC2_B);
  uint8_t currentState = (b << 1) | a;
  uint8_t trans  = (lastState2 << 2) | currentState;
  lastState2 = currentState;

  if (trans==0b0001 || trans==0b0111 || trans==0b1110 || trans==0b1000) {
    encCount2++;
  }
  else if (trans==0b0010 || trans==0b1011 || trans==0b1101 || trans==0b0100) {
    encCount2--;
  }
}

void moveStep(long stepA, long stepB, long remainder1, long remainder2) {
  noInterrupts();
  long startingCountA = encCount1;
  long startingCountB = encCount2;
  interrupts();
  long speed = 0;
  long currentCountA = startingCountA;
  long currentCountB = startingCountB;

  if (stepA != 0) {
    analogWrite(M1_PWM, speed);
    while (true) {
      noInterrupts();
      currentCountA = encCount1;
      interrupts();
      long localCountA = currentCountA - startingCountA;
      if (localCountA >= stepA) {
        analogWrite(M1_PWM, 0);
        break;
      }
    }
  }

  if (stepB != 0) {
    analogWrite(M2_PWM, speed);
    while (true) {
      noInterrupts();
      currentCountB = encCount2;
      interrupts();
      long localCountB = currentCountB - startingCountB;
      if (localCountB >= stepB) {
        analogWrite(M2_PWM, 0);
        break;
      }
    }
  }
}

void move(float targetX, float targetY) {
  bool targetReached = false;

  if (state == IDLE) {
    long moveA_mm = targetX + targetY;
    long moveB_mm = targetX - targetY;
    int direction1;
    int direction2;

    targetTicks1 = lround((moveA_mm / MM_PER_REV) * (float)COUNTS_PER_REV);
    targetTicks2 = lround((moveB_mm / MM_PER_REV) * (float)COUNTS_PER_REV);

    if (targetTicks1 >= 0) { direction1 = +1; }
    else { direction1 = -1; }
    if (targetTicks2 >= 0) { direction2 = +1; }
    else { direction2 = -1; }

    if (direction1 > 0) { digitalWrite(M1_DIR, HIGH); }
    else { digitalWrite(M1_DIR, LOW); }
    if (direction2 > 0) { digitalWrite(M2_DIR, HIGH); }
    else { digitalWrite(M2_DIR, LOW); }

    encCount1 = 0;
    encCount2 = 0;

    state = MOVING;
  }
  else if (state == MOVING) {
    long totalCount1, totalCount2;
    long remainder1Ticks, remainder2Ticks;
    long remainder1MM, remainder2MM;
    long remainder1MMabs, remainder2MMabs;
    float stepX_mm, stepY_mm;
    long stepA_enc, stepB_enc;

    while (!targetReached) {
      noInterrupts();
        totalCount1 = encCount1;
        totalCount2 = encCount2;
      interrupts();

      if (abs(targetTicks1) > abs(totalCount1)) { remainder1Ticks = targetTicks1 - totalCount1; }
      else { remainder1Ticks = 0; }
      if (abs(targetTicks2) > abs(totalCount2)) { remainder2Ticks = targetTicks2 - totalCount2; }
      else { remainder2Ticks = 0; }

      remainder1MM = remainder1Ticks / TICKS_PER_MM;
      remainder1MMabs = abs(remainder1MM);
      remainder2MM = remainder2Ticks / TICKS_PER_MM;
      remainder2MMabs = abs(remainder2MM);

      if (remainder1MMabs > 0 && remainder2MMabs > 0) {
        if (remainder1MMabs > 1 || remainder2MMabs > 1) {
          if (remainder1MMabs >= remainder2MMabs) {
            stepX_mm = 1.0f;
            stepY_mm = remainder2MMabs / remainder1MMabs;
          }
          else if (remainder1MMabs < remainder2MMabs) {
            stepY_mm = 1.0f;
            stepX_mm = remainder1MMabs / remainder2MMabs;
          }
        } else {
          stepX_mm = remainder1MMabs;
          stepY_mm = remainder2MMabs;
        }
      } else {
        stepX_mm = fmod(remainder1MMabs, 1.0f);
        stepY_mm = fmod(remainder2MMabs, 1.0f);
      }
      
      stepA_enc = (stepX_mm + stepY_mm) * TICKS_PER_MM;
      stepB_enc = (stepX_mm - stepY_mm) * TICKS_PER_MM;

      if (stepA_enc == 0 && stepB_enc == 0) {
        targetReached = true;
        break;
      }

      moveStep(stepA_enc, stepB_enc, remainder1Ticks, remainder2Ticks);
    }
    state = IDLE;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  lastState1 = (digitalRead(ENC1_B) << 1) | digitalRead(ENC1_A);
  lastState2 = (digitalRead(ENC2_B) << 1) | digitalRead(ENC2_A);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), isrEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isrEnc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), isrEnc2, CHANGE);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void loop() {
  Serial.println("Enter ΔX and ΔY (mm):");
  targetX = Serial.parseFloat();
  targetY = Serial.parseFloat();
  move(targetX, targetY);
  Serial.println("Target Reached");
}
