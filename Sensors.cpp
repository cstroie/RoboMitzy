/**
  Sensors.cpp

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#include "Arduino.h"
#include "Sensors.h"

Sensors::Sensors() {
}

/**
  Initialize the ADC
*/
void Sensors::init(uint8_t pin) {
  // Set prescaler to 16 (1MHz)
  ADCSRA |=  _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS0);
  ADCSRA &= ~_BV(ADPS1);
  // Enable the ADC
  //ADCSRA |=  _BV(ADEN);
  // Wait for voltage to settle (bandgap stabilizes in 40-80 us)
  delay(10);
  // Dummy analog read
  analogRead(A0);
  // Keep the IR leds pin and configure the output
  pinIR = pin;
  pinMode(pinIR, OUTPUT);
  ledOnIR();
}

void Sensors::ledOnIR() {
  digitalWrite(pinIR, HIGH);
}

void Sensors::ledOffIR() {
  digitalWrite(pinIR, LOW);
}

/**
  Read the channels
*/
void Sensors::readAllChannels() {
  // Read each channel and compute the relative values for all but the last
  for (uint8_t c = 0; c < CHANNELS; c++)
    chnRaw[c] = readChannel(c);
  // Compute the relative value for the last channel
  calcRelative(CHANNELS - 1);
}

/**
  Read one channel and calculate the relative value of the previous one,
  while waiting for voltage to settle after changing the MUX.

  The calc routine takes 26us.  We need to wait 10us for less than 100k
  impedance and 150us for 1M.
*/
uint8_t Sensors::readChannel(uint8_t channel) {
  // Set the MUX
  setChannel(channel);
  // Wait for voltage to settle after changing the MUX
  if (channel == 0) {
    // Can not compute anything for the first channel, just wait
    delayMicroseconds(25 + 175);
    //delay(1);
  }
  else {
    // Compute the relative value of the previous channel
    calcRelative(channel - 1);
    // And wait some more...
    delayMicroseconds(175);
    //delay(1);
  }
  // Read the channel
  return readRaw();
}

/**
  Set the MUX accordingly, also set left-adjusted
*/
void Sensors::setChannel(uint8_t channel) {
  // Set the analog reference to DEFAULT, select the channel (low 4 bits).
  // This also sets ADLAR (left-adjust result) to 1, so we use 8 bits.
  // Needs external capacitor at AREF pin.
  ADMUX = _BV(REFS0) | _BV(ADLAR) | (channel & 0x07);
}

/**
  Read the channel: start conversion and wait to finish, return 8 bits
*/
uint8_t Sensors::readRaw() {
  // Start conversion
  ADCSRA |= _BV(ADSC);
  // Wait to finish (13 cycles)
  while (bit_is_set(ADCSRA, ADSC));
  // Read register ADCH
  return ADCH;
}

/**
  Read the channels, update the minimum and maximum and keep data for histogram
*/
void Sensors::calibrate() {
  for (uint8_t c = 0; c < CHANNELS; c++) {
    // Set the MUX
    setChannel(c);
    // Wait for voltage to settle
    delayMicroseconds(100);
    // Read the channel
    chnRaw[c] = readRaw();
    if (chnRaw[c] < chnMin[c]) chnMin[c] = chnRaw[c];
    if (chnRaw[c] > chnMax[c]) chnMax[c] = chnRaw[c];
    // Use the two paramedian sensors to fill the polarity histogram
    if ((c == 3) or (c == 4)) {
      // Get the histogram index
      uint8_t idx = (chnRaw[c] >> 4);
      polHst[idx]++;
    }
  }
}

/**
  Polarity histogram reset
*/
void Sensors::polReset() {
  for (uint8_t i = 0; i < 16; i++)
    polHst[i] = 0;
}

/**
  Get the surface polarity:
  + positive: black line on white background
  - negative: white line on black background
*/
bool Sensors::getPolarity() {
  uint16_t total, count;
  uint8_t center;
  // Count all items in histogram
  for (uint8_t i = 0; i < 16; i++)
    total += polHst[i];
  // Half the total
  total = total >> 1;
  // Find the binary distribution
  for (uint8_t i = 0; i < 16; i++) {
    count += polHst[i];
    // Stop when reaching half the total
    if (count >= total) {
      center = i;
      break;
    }
  }
  if (center < 8) polarity = true;
  else            polarity = false;
  return polarity;
}

/**
  Calculate channel range: max-min
*/
bool Sensors::validate() {
  bool valid = true;
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRange[c] = chnMax[c] - chnMin[c];
    // Validate (range should be greater than half the sensor definition)
    if (chnRange[c] < 0x7F) valid = false;
  }
  return valid;
}

/**
  Calculate the relative sensor value
*/
void Sensors::calcRelative(uint8_t channel) {
  // Upscale to 16 bits
  uint16_t y = chnRaw[channel];
  // Keep the raw value inside the calibrated interval: saturate
  if      (y < chnMin[channel]) y = chnMin[channel];
  else if (y > chnMax[channel]) y = chnMax[channel];
  // Do the integer math
  y -= chnMin[channel];
  chnVal[channel] = (uint8_t)((y << 8) / chnRange[channel]);
}

/**
  Get the error for the PID controller
*/
int16_t Sensors::getError() {
  int16_t result;
  // Read the sensors
  readAllChannels();
  // Compute the error using the relative values and the weights of the channels
  for (uint8_t c = 0; c < CHANNELS; c++)
    result += chnVal[c] * chnWht[c];
  // Return the result
  return result;
}

