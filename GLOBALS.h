#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <stdint.h>

// ===== Motor pins =====
extern const int M1_DIR;
extern const int M1_PWM;
extern const int M2_PWM;
extern const int M2_DIR;

// ===== Encoder pins =====
extern const int ENC1_A;
extern const int ENC1_B;
extern const int ENC2_A;
extern const int ENC2_B;

// ===== Motor control values =====
extern const int MOTOR_MAX_SPEED;    // Max PWM value
extern const int MOTOR_MIN_SPEED;    // Min PWM value

// ===== Encoder counters (volatile: modified in ISRs) =====
extern volatile long encCount1;
extern volatile long encCount2;
extern volatile long targetTicks1;
extern volatile long targetTicks2;

// ===== Encoder state tracking =====
extern volatile uint8_t lastState1;
extern volatile uint8_t lastState2;

// === Mechanical constants ===
extern const long CPR_MOTOR;
extern const long GEAR_RATIO;
extern const long COUNTS_PER_REV;
extern const float PULLEY_DIAM;
extern const float MM_PER_REV;
extern const float TICKS_PER_MM;

// ===== Movement state =====
typedef enum {
  IDLE,
  MOVING
} State;

extern State state;

// ===== Target and movement variables =====
extern float targetX, targetY;

// ===== Placeholder speed (for testing) =====
extern int currentSpeed;

#endif // GLOBALS_H
