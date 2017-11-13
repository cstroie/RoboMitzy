/**
  Motors.cpp

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/


/*
  Vl = V * (1 - b/(2*r))
  Vr = V * (1 + b/(2*r))
*/

#include "Arduino.h"
#include "Motors.h"

Motors::Motors() {
}

void Motors::init() {
  // Use TIMER0 on pins 6 and 5, no change in frequency
  pinMode(M1A, OUTPUT); pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT); pinMode(M2B, OUTPUT);
}

void Motors::init(uint8_t min_speed, uint8_t max_speed) {
  minSpeed = min_speed;
  maxSpeed = max_speed;
  init();
}

/**
  Left motor control
*/
void Motors::left(uint8_t speed, bool dir) {
  if (dir) {
    // Forward, speed control
    analogWrite(M1A, speed);
    digitalWrite(M1B, LOW);
  }
  else {
    // Backward, full speed (no PWM)
    digitalWrite(M1A, LOW);
    digitalWrite(M1B, HIGH);
  }
}

/**
  Right motor control
*/
void Motors::right(uint8_t speed, bool dir) {
  if (dir) {
    // Forward, speed control
    analogWrite(M2A, speed);
    digitalWrite(M2B, LOW);
  }
  else {
    // Backward, full speed (no PWM)
    digitalWrite(M2A, LOW);
    digitalWrite(M2B, HIGH);
  }
}

/**
  Control each motor by speed and direction
*/
void Motors::run(uint8_t lS, bool lD, uint8_t rS, bool rD) {
  left(lS, lD);
  right(rS, rD);
}

/**
  Control the motors by speed and turn, slowing down one wheel
  and speeding up the other
*/
void Motors::run(uint8_t speed, int8_t turn) {
  int16_t left, right;
  // Compute individual motor speeds: - left, + right
  left  = speed + turn;
  right = speed - turn;
  constrain(left,  -255, 255);
  constrain(right, -255, 255);
  // Run the motors
  if (speed == 0) stop();
  else            run(abs(left), left > 0, abs(right), right > 0);
}


/**
  Control the motors by speed and turn, only slowing down one wheel.
  Positive speed is forward, negatve is backward.
  Positive turn is to the right, negative to the left.
  Only forward running.
*/
void Motors::drive(int8_t speed, int8_t turn) {
  // Speed and turn in absolute values
  uint8_t aspeed = speed == -128 ? 0xFF : (abs(speed) << 1);
  uint8_t aturn  = turn  == -128 ? 0xFF : (abs(turn)  << 1);
  // Relative speed and turn
  uint16_t range = maxSpeed - minSpeed;
  uint8_t fast = minSpeed + ((range * aspeed) >> 8);
  uint8_t slow = fast - ((range * aturn) >> 8);
  // Run the motors
  if      (turn > 0)  run(fast, true, slow, true);
  else if (turn < 0)  run(slow, true, fast, true);
  else                run(fast, true, fast, true);
}

/**
  Stop the motors
*/
void Motors::stop(bool br) {
  if (br) {
    // Electrobrake
    digitalWrite(M1A, HIGH);  digitalWrite(M1B, HIGH);
    digitalWrite(M2A, HIGH);  digitalWrite(M2B, HIGH);
  }
  else {
    // Stop
    digitalWrite(M1A, LOW);   digitalWrite(M1B, LOW);
    digitalWrite(M2A, LOW);   digitalWrite(M2B, LOW);
  }
}


