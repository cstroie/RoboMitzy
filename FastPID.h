/**
  FastPID.h - A PID controlled implemented using fixed-point arithmatic.

  Copyright 2017 Mike Matera <matera@lifealgorithmic.com>

  This is a high-quality, high-precision PID controller that uses a fixed-point pipeline.
  Conversion from floating point coefficients to integers is done automatically.
  This implementation is suitable for AVR and other processors where floating point math is slow.

  https://github.com/mike-matera/FastPID
*/

#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>

#define INTEG_MAX    (INT32_MAX >> 1)
#define INTEG_MIN    (INT32_MIN >> 1)

#define PARAM_SHIFT  23
#define PARAM_BITS   25
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT)
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)-1) >> (PARAM_BITS - PARAM_SHIFT))

/**
  A fixed point PID controller with a 64-bit internal calculation pipeline.
*/
class FastPID {

  public:
    FastPID() {
      clear();
    }

    FastPID(float kp, float ki, float kd, uint16_t db = 0, int bits = 16, bool sign = false) {
      configure(kp, ki, kd, db, bits, sign);
    }

    ~FastPID();

    void clear();
    bool configure(float kp, float ki, float kd, uint16_t db = 0, int bits = 16, bool sign = false);
    int16_t step(int16_t sp, int16_t fb);
    bool err() {
      return _cfg_err;
    }

  private:
    uint32_t floatToParam(float);
    void setCfgErr();

  private:
    // Configuration
    uint32_t _p, _i, _d;
    uint32_t _db;
    int64_t _outmax, _outmin;

    // State
    int16_t _last_sp, _last_out;
    int32_t _sum, _last_err;
    uint32_t _last_run;
    int64_t _ctl;
    bool _cfg_err;
};

#endif /* FastPID_H */
