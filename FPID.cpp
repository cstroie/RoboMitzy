/**
  FPID.cpp

  Copyright (c)g 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#include "Arduino.h"
#include "FPID.h"

FPID::FPID() {
}

/**
  Ideal PID form, using proportional gain, integral gain and derivative gain
*/
void FPID::init(float Kp, float Ki, float Kd) {
  // Constraints
  Kp = constrain(Kp, MIN8, MAX8);
  Ki = constrain(Ki, MIN8, MAX8);
  Kd = constrain(Kd, MIN8, MAX8);
  // Convert the gains to Q23.8
  kp = Kp * FP_ONE;
  ki = Ki * FP_ONE;
  kd = Kd * FP_ONE;
  // Reset persistent loop variables
  oldError = 0;
  oldIntgr = 0;
}

/**
  Standard PID form, using proportional gain, integral time and derivative time
*/
void FPID::initStd(float Kp, float Ti, float Td) {
  float Ki;
  if (Ti == 0) Ki = 0;
  else         Ki = Kp / Ti;
  init(Kp, Ki, Kp * Td);
}

/**
  Compute the correction (Q7.8) based on error (Q7.8)
*/
int16_t FPID::step(int16_t error) {
  uint32_t now = millis();
  if (now > oldTime) {
    // Compute the sampling time interval (delta t) Q8.0
    uint8_t dt = (uint8_t)(now - oldTime);
    // Integral error Q23.8
    if (ki) iOut = error;       // Q7.8
    // Derivative on delta-t: Q23.8 * Q7.8 = Q31.8
    if (kd) dOut = fp_mul(kd, error - oldError); // Q23.8
    // If fast enough, we can spare a multiplication and division
    if (dt > 1) {
      if (ki) iOut = fp_mul(iOut, dt); // Q23.8
      if (kd) dOut = fp_div(dOut, dt); // Q23.8
    }
    if (ki) {
      // Keep the present integral Q23.8
      oldIntgr = constrain(oldIntgr + iOut, MIN24, MAX24);
      iOut = fp_mul(ki, oldIntgr);   // Q23.8 !
      // Constrain the integral
      iOut = constrain(iOut, MIN32, MAX32);
    }
    // Proportional Q23.8 * Q7.8 = Q31.8
    pOut = constrain(fp_mul(kp, error), MIN24, MAX24);  // Q23.8

    // Keep the partial and final results, saturated
    result = constrain(pOut + iOut + dOut, MIN16, MAX16);
    oldTime = now;
    oldError = error;
  }
  // Return the old or new result
  return result;
}

