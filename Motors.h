/**
  Motors.h

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef MOTORS_H
#define MOTORS_H

#define M1A 5 // PWM
#define M1B 7
#define M2A 6 // PWM
#define M2B 8

#include "Arduino.h"

class Motors {
  public:
    Motors();
    void init();

    void left(uint8_t speed, bool dir);
    void right(uint8_t speed, bool dir);
    void run(uint8_t lS, bool lD, uint8_t rS, bool rD);
    void run(uint8_t speed, int8_t turn);
    void drive(int8_t speed, int8_t turn);
    void stop(bool br = false);

    uint8_t minSpeed = 60;
    uint8_t maxSpeed = 255;
};

#endif /* MOTORS_H */
