/**
  Motors.h

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef MOTORS_H
#define MOTORS_H

#include "Arduino.h"

class Motors {
  public:
    Motors();
    void init();

    void run(uint8_t leftSpeed, bool leftDir, uint8_t rightSpeed, bool rightDir);
    void run(int8_t speed, int8_t turn);
    void off();
    void brake();

    uint8_t minSpeed = 60;
    uint8_t maxSpeed = 255;
};

#endif /* MOTORS_H */
