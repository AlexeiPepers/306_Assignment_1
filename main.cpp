#include <Arduino.h>
#include "GLOBALS.h"
#include "gantry.h"

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
