#include "GLOBALS.h"

// ===== Limit pins =====
const int TOP_LIMIT_PIN = 10;
const int BOTTOM_LIMIT_PIN = 11;
const int LEFT_LIMIT_PIN = 12;
const int RIGHT_LIMIT_PIN = 13;

// ===== Motor pins =====
const int M1_DIR  = 4;
const int M1_PWM  = 5;
const int M2_PWM = 6;
const int M2_DIR = 7;

// ===== Encoder pins =====
const int ENC1_A  = 2;
const int ENC1_B  = 3;
const int ENC2_A = 18;
const int ENC2_B = 19;

// ===== Motor control values =====
const int MOTOR_MAX_SPEED = 255;  // Full speed
const int MOTOR_MIN_SPEED = 0;    // Stopped

// ===== Encoder counters =====
volatile long encCount1 = 0;
volatile long encCount2 = 0;
volatile long targetTicks1 = 0;
volatile long targetTicks2 = 0;

// ===== Encoder state tracking =====
volatile uint8_t lastState1 = 0;
volatile uint8_t lastState2 = 0;
volatile bool move_complete = false;

// === Mechanical constants ===
const long CPR_MOTOR = 48L;
const long GEAR_RATIO = 172L;
const long COUNTS_PER_REV = CPR_MOTOR * GEAR_RATIO;
const float PULLEY_DIAM = 14.0f;
const float MM_PER_REV = PI * PULLEY_DIAM;
const float TICKS_PER_MM = (float)COUNTS_PER_REV / MM_PER_REV;
const float STEP_BASE = 1.0f; // 1mm default
const long STEP_BASE_ENC = STEP_BASE * TICKS_PER_MM;

// ===== Target and movement variables =====
float targetX, targetY;

// ===== Controller Values =====
float Kp = 0;
float Ki = 0;

// ===== State =====
State state = IDLE; 
