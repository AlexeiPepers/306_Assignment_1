#include <Arduino.h>
#include "GLOBALS.h"
#include <math.h>


ISR(PCINT0_vect) {
 // Determine state of each limit switch
  if (digitalRead(LEFT_LIMIT_PIN) == HIGH) {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    state = LIMIT;
    Serial.println("Left limit reached");
  }

  if (digitalRead(RIGHT_LIMIT_PIN) == HIGH) {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    state = LIMIT;
    Serial.println("Right limit reached");
  }

  if (digitalRead(TOP_LIMIT_PIN) == HIGH) {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    state = LIMIT;
    Serial.println("Top limit reached");
  }

  if (digitalRead(BOTTOM_LIMIT_PIN) == HIGH) {
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    state = LIMIT;
    Serial.println("Bottom limit reached");
  }
}

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

bool move(float dist_X_mm, float dist_Y_mm) {
    // Reset encoder counts
    noInterrupts();
    encCount1 = 0;
    encCount2 = 0;
    interrupts();

    float dist_A_mm = dist_X_mm + dist_Y_mm;
    float dist_B_mm = dist_X_mm - dist_Y_mm;

    // Convert into encoder counts
    long dist_A_enc = dist_A_mm * TICKS_PER_MM;
    long dist_B_enc = dist_B_mm * TICKS_PER_MM;

    bool direction_A = dist_A_enc >= 0 ? true : false;
    bool direction_B = dist_B_enc >= 0 ? true : false;

    // Set directions
    digitalWrite(M1_DIR, direction_A ? HIGH : LOW);
    digitalWrite(M2_DIR, direction_B ? HIGH : LOW);

    bool target_reached = false;
    // ---- Pure A translation ----
    if (dist_B_enc == 0) {
        analogWrite(M1_PWM, MOTOR_MAX_SPEED);
        while (!target_reached) {
            noInterrupts();
            long enc_count = encCount1;
            interrupts();
            if (abs(enc_count) >= abs(dist_A_enc)) {
                target_reached = true;
            }
        }
        analogWrite(M1_PWM, 0);
    }
    // ---- Pure B translation ----
    else if (dist_A_enc == 0) {
        analogWrite(M2_PWM, MOTOR_MAX_SPEED);
        while (!target_reached) {
            noInterrupts();
            long enc_count = encCount2;
            interrupts();
            if (abs(enc_count) >= abs(dist_B_enc)) {
                target_reached = true;
            }
        }
        analogWrite(M2_PWM, 0);
    }

    // ---- Diagonal translation ---- 
    else {
        while (!target_reached) {
            long rem_A_enc, rem_B_enc;
            long step_A_enc, step_B_enc;

            // Read encoders
            noInterrupts();
            long start_count_A = encCount1;
            long start_count_B = encCount2;
            interrupts();

            rem_A_enc = abs(dist_A_enc) - abs(start_count_A);
            rem_B_enc = abs(dist_B_enc) - abs(start_count_B);

            // Determine which motor to use as a base
            if (rem_A_enc >= rem_B_enc && rem_A_enc >= STEP_BASE_ENC) {
                step_A_enc = STEP_BASE_ENC;
                step_B_enc = (long)((float)rem_B_enc / rem_A_enc * STEP_BASE_ENC);
            } 
            else if (rem_B_enc > rem_A_enc && rem_B_enc >= STEP_BASE_ENC) {
                step_B_enc = STEP_BASE_ENC;
                step_A_enc = (long)((float)rem_A_enc / rem_B_enc * STEP_BASE_ENC);
            }  
            else {
                step_A_enc = rem_A_enc;
                step_B_enc = rem_B_enc;
                target_reached = true;
            }

            // Start both motors
            analogWrite(M1_PWM, MOTOR_MAX_SPEED);
            analogWrite(M2_PWM, MOTOR_MAX_SPEED);
            
            long local_count_A = 0;
            long local_count_B = 0;
            bool A_flag = false;
            bool B_flag = false;
            while (!(A_flag && B_flag)) {
                // Read encoders
                noInterrupts();
                local_count_A = encCount1 - start_count_A;
                local_count_B = encCount2 - start_count_B;
                interrupts();

                if (local_count_A >= step_A_enc) {
                    analogWrite(M1_PWM, 0);
                    A_flag = true;
                }
                if (local_count_B >= step_B_enc) {
                    analogWrite(M2_PWM, 0);
                    B_flag = true;
                }
            }
        }
    }
    Serial.print("Target Reached");
    return true;
}

void fsm() {
  if (state == IDLE) {
    if (Serial.available() > 0) {
      Serial.print("ΔX="); Serial.print(targetX);
      Serial.print(" mm, ΔY="); Serial.print(targetY);
      Serial.println(" mm");
      state = MOVING;
      move_complete = move(targetX, targetY);
    }
  } 
  
  else if (state == MOVING) {
    if (move_complete) {
      state = IDLE;
      Serial.println("Enter ΔX and ΔY (mm):");
      targetX = Serial.parseFloat();
      targetY = Serial.parseFloat();
    }
  }

  else if (state == LIMIT) {
    while(1) {
      asm("nop");
    }
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

  PCICR |= (1 << PCIE0);      // Enable Pin Change Interrupt group 0
  PCMSK0 |= (1 << PCINT4) | (1 << PCINT5) | (1 << PCINT6) | (1 << PCINT7);
}

void loop() {
  fsm();
}
