/**
  FPID.h

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef FPID_H
#define FPID_H

#include "Arduino.h"
#include "FixedPoint.h"

#define MAX32    (INT32_MAX >> 1)
#define MIN32    (INT32_MIN >> 1)
#define MAX24    (INT32_MAX >> FP_FBITS)
#define MIN24    (INT32_MIN >> FP_FBITS)

class FPID {
  public:
    FPID();
    void    init(float Kp, float Ki, float Kd);
    int16_t step(int16_t error);

  private:
    int16_t kp;
    int16_t ki;
    int16_t kd;

    int32_t pOut;
    int32_t iOut;
    int32_t dOut;

    uint32_t oldTime;
    int32_t  oldIntgr;
    int16_t  oldError;
    int16_t  result;
};

#endif /* FPID_H */
