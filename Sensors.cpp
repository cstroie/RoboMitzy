/**
  Sensors.cpp

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#include "Arduino.h"
#include "Sensors.h"

Sensors::Sensors() {
}

void Sensors::init() {
  // Set prescaler to 16 (1MHz)
  ADCSRA |=  _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS0);
  ADCSRA &= ~_BV(ADPS1);
  // Enable the ADC
  ADCSRA |=  _BV(ADEN);
  // Wait for voltage to settle (bandgap stabilizes in 40-80 us)
  delay(10);
}

void Sensors::readAllChannels() {
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRaw[c] = readChannel(c);
  }
}

uint8_t Sensors::readChannel(uint8_t channel) {
  // Set the analog reference to DEFAULT, select the channel (low 4 bits).
  // This also sets ADLAR (left-adjust result) to 1, so we use 8 bits.
  // Needs external capacitor at AREF pin.
  ADMUX = _BV(REFS0) | _BV(ADLAR) | (channel & 0x07);
  // Wait for voltage to settle
  delay(1);
  // Start conversion
  ADCSRA |= _BV(ADSC);
  // Wait to finish
  while (bit_is_set(ADCSRA, ADSC));
  // Read register ADCH
  return ADCH;
}

void Sensors::readMinMax() {
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRaw[c] = readChannel(c);
    if (chnRaw[c] < chnMin[c]) chnMin[c] = chnRaw[c];
    if (chnRaw[c] > chnMax[c]) chnMax[c] = chnRaw[c];
  }
}

