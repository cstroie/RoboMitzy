/**
  FPID.h

  Copyright (c) 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef FPID_H
#define FPID_H

#include "Arduino.h"
#include "FixedPoint.h"

class FPID {
  public:
    FPID();
    void    init(float Kp, float Ki, float Kd);
    void    initStd(float Kp, float Ti, float Td);
    int16_t step(int16_t error);

  private:
    int32_t   kp,   ki,   kd;   // Q23.8
    int32_t   pOut, iOut, dOut; // Q23.8

    uint32_t  oldTime;
    int32_t   oldIntgr;
    int16_t   oldError;
    int32_t   result;           // Q7.8
};

#endif /* FPID_H */
