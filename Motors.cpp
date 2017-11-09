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
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
}

void Motors::run(uint8_t leftSpeed, bool leftDir, uint8_t rightSpeed, bool rightDir) {
  if (leftDir) {
    // Left forward
    analogWrite(5, leftSpeed);
    digitalWrite(7, LOW);
  }
  else {
    // Left backward
    //analogWrite(5, 255 - leftSpeed);
    digitalWrite(5, LOW);
    digitalWrite(7, HIGH);
  }
  if (rightDir) {
    // Right forward
    analogWrite(6, rightSpeed);
    digitalWrite(8, LOW);
  }
  else {
    // Right backward
    //analogWrite(6, 255 - rightSpeed);
    digitalWrite(6, LOW);
    digitalWrite(8, HIGH);
  }
}

void Motors::run(int8_t speed, int8_t turn) {
  uint8_t leftSpeed, rightSpeed;

  leftSpeed  = speed - turn;
  rightSpeed = speed + turn;
  
  Serial.print(leftSpeed);
  Serial.print(",");
  Serial.print(rightSpeed);
  Serial.println();
  

  if (speed == 0) { // Off
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  }
  else {            // Forward or backward
    //run(abs(leftSpeed) + minSpeed, leftSpeed > 0, abs(rightSpeed) + minSpeed, rightSpeed > 0);
    run(abs(leftSpeed), leftSpeed > 0, abs(rightSpeed), rightSpeed > 0);
  }
}

void Motors::off() {
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
}

void Motors::brake() {
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  digitalWrite(8, HIGH);
}

