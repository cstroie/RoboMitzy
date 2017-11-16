/**
  FastPID.cpp - A PID controlled implemented using fixed-point arithmatic.

  Copyright 2017 Mike Matera <matera@lifealgorithmic.com>
  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This is a high-quality, high-precision PID controller that uses a fixed-point pipeline.
  Conversion from floating point coefficients to integers is done automatically.
  This implementation is suitable for AVR and other processors where floating point math is slow.

  https://github.com/mike-matera/FastPID
*/

#include <Arduino.h>
#include "FastPID.h"

FastPID::~FastPID() {
}

void FastPID::clear() {
  _last_sp  = 0;
  _last_out = 0;
  _sum      = 0;
  _last_err = 0;
  _last_run = 0;
  _cfg_err  = false;
}

bool FastPID::configure(float kp, float ki, float kd, int bits, bool sign) {
  clear();

  // Set parameters
  _p = floatToParam(kp);
  _i = floatToParam(ki);
  _d = floatToParam(kd);

  // Set output bits
  if (bits > 16 || bits < 1)
    _cfg_err = true;
  else {
    if (sign) {
      // Symmetry
      _outmax =  ((0x1ULL << (bits - 1)) - 1) * PARAM_MULT;
      _outmin = -((0x1ULL << (bits - 1)) - 1) * PARAM_MULT;
    }
    else {
      _outmax = ((0x1ULL << bits) - 1) * PARAM_MULT;
      _outmin = 0;
    }
  }

  return !_cfg_err;
}

uint32_t FastPID::floatToParam(float in) {
  if (in > PARAM_MAX || in < 0) {
    _cfg_err = true;
    return 0;
  }
  return in * PARAM_MULT;
}

int16_t FastPID::step(int16_t sp, int16_t fb) {

  // Calculate delta T
  // millis(): Frequencies less than 1Hz become 1Hz.
  //   max freqency 1 kHz (XXX: is this too low?)
  uint32_t now = millis();
  uint32_t hz = 0;
  // Ignore I and D on the first step. They will be
  // unreliable because no time has really passed.
  if (_last_run != 0) {
    // 47-day timebomb
    if (now < _last_run)  hz = uint32_t(1000) / (now + (~_last_run));
    else                  hz = uint32_t(1000) / (now -   _last_run);
    if (hz == 0)          hz = 1;
  }

  _last_run = now;

  // int16 + int16 = int17
  int32_t err = int32_t(sp) - int32_t(fb);
  int64_t P = 0, I = 0, D = 0;

  if (_p) {
    // uint23 * int16 = int39
    P = int64_t(_p) * int64_t(err);
  }

  if (_i && hz) {
    // (int16 * uint32) + int31 = int32
    _sum += int32_t(err) / int32_t(hz);

    // Limit sum to 31-bit signed value so that it saturates, never overflows.
    if      (_sum > INTEG_MAX)  _sum = INTEG_MAX;
    else if (_sum < INTEG_MIN)  _sum = INTEG_MIN;

    // uint23 * int31 = int54
    I = int64_t(_i) * int64_t(_sum);
  }

  if (_d && hz) {
    // int17 - (int16 - int16) = int19
    int32_t deriv = (sp - _last_sp) - (err - _last_err);
    _last_sp  = sp;
    _last_err = err;

    // uint23 * int19 * uint16 = int58
    D = int64_t(_d) * int64_t(deriv) * int64_t(hz);
  }

  // int39 (P) + int54 (I) + int58 (D) = int61
  int64_t diff = P + I + D;

  // Make the output saturate
  if      (diff > _outmax)  diff = _outmax;
  else if (diff < _outmin)  diff = _outmin;

  // Remove the integer scaling factor.
  int16_t out = diff >> PARAM_SHIFT;

  // Fair rounding.
  if (diff & (0x1ULL << (PARAM_SHIFT - 1)))
    out++;

  return out;
}
