/**
  FPID.cpp

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#include "Arduino.h"
#include "FPID.h"

FPID::FPID() {
}

void FPID::init(float Kp, float Ki, float Kd) {
  // Convert the gains to Q7.8
  kp = Kp * (int16_t)FP_ONE;
  ki = Ki * (int16_t)FP_ONE;
  kd = Kd * (int16_t)FP_ONE;
}

int16_t FPID::step(int16_t error) {
  uint32_t now = millis();
  if (now > oldTime) {
    // Compute the sampling time interval (delta t)
    uint16_t dt = (uint16_t)(now - oldTime);
    // Integral error.time: Q7.8 * Q16 = Q23.8
    iOut = ki * error;
    // Derivative on delta-t: Q7.8 * Q16 = Q23.8
    dOut = kd * (error - oldError);
    // If fast enough, we can spare a multiplication and division
    if (dt > 1) {
      iOut = iOut * dt;
      dOut = dOut / dt;
    }
    // Integral sum
    iOut = constrain(iOut + oldIntgr, MIN24, MAX24);
    // Proportional Q7.8 * Q16 = Q23.8
    pOut = kp * error;
    //Serial.println((pOut + iOut + dOut) >> FP_FBITS);

    // Keep the partial and final results
    result = constrain((int16_t)((pOut + iOut + dOut) >> FP_FBITS), MIN24, MAX24);
    oldTime = now;
    oldError = error;
    oldIntgr = iOut;
  }
  // Return the old or new result
  return result;
}

