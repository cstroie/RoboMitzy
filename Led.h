/**
  Led.h

  Copyright (c) 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef LED_H
#define LED_H

#include "Arduino.h"

class Led {
  public:
    Led(uint8_t pin);
    void init(uint32_t period, uint8_t dutycycle);
    void setPin(uint8_t pin);
    void blink();
    void wait(uint8_t wait, uint32_t period, uint8_t dutycycle);

  private:
    uint8_t   ledPIN = 13;
    uint32_t  ledSwitch;
    uint32_t  ledOn;
    uint32_t  ledOff;
    uint8_t   ledState;
};

#endif /* LED_H */
