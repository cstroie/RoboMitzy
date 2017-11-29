/**
  Led.cpp

  Copyright (c) 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#include "Arduino.h"
#include "Led.h"

Led::Led(uint8_t pin) {
  setPin(pin);
}

void Led::init(uint32_t period, uint8_t dutycycle) {
  ledOn     = period * dutycycle * 0.01;
  ledOff    = period - ledOn;
  ledSwitch = millis() + ledOff;
  ledState  = LOW;
  digitalWrite(ledPIN, ledState);
}

void Led::setPin(uint8_t pin) {
  // Set the IR leds pin and configure the output
  ledPIN = pin;
  pinMode(ledPIN, OUTPUT);
  digitalWrite(ledPIN, LOW);
}

void Led::blink() {
  if (millis() > ledSwitch) {
    if (ledState) {
      ledSwitch += ledOff;
      ledState = LOW;
    }
    else {
      ledSwitch += ledOn;
      ledState = HIGH;
    }
    digitalWrite(ledPIN, ledState);
  }
}

void Led::wait(uint8_t count, uint32_t period, uint8_t dutycycle) {
  // Save
  uint32_t  oldOn  = ledOn;
  uint32_t  oldOff = ledOff;
  // Re-initialize
  init(period, dutycycle);
  // Busyloop
  while (count--) {
    digitalWrite(ledPIN, HIGH); delay(ledOn);
    digitalWrite(ledPIN, LOW);  delay(ledOff);
  }
  // Restore
  ledOn  = oldOn;
  ledOff = oldOff;
  // Re-initialize
  init(period, dutycycle);
}

