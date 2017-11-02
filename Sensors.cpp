/**
  Sensors.cpp

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#define MUX_DELAY_US 150

#include "Arduino.h"
#include "Sensors.h"

Sensors::Sensors() {
}

/**
  Initialize the line sensor
*/
void Sensors::init(uint8_t pin) {
  // Start with a dummy analog read
  analogRead(A0);
  // Disconnect the digital inputs from the ADC channels
  DIDR0 = 0xFF;
  // Set prescaler to 16 (1MHz)
  ADCSRA |=  _BV(ADPS2);
  ADCSRA &= ~_BV(ADPS1);
  ADCSRA &= ~_BV(ADPS0);
  // Wait for voltage to settle (bandgap stabilizes in 40-80 us)
  delay(10);
  // Set the IR leds pin and configure the output
  pinIR = pin;
  pinMode(pinIR, OUTPUT);
  // Turn the leds on
  ledOnIR();
  // Calibration reset
  reset();
}

/**
  Turn on the infrared leds
*/
void Sensors::ledOnIR() {
  digitalWrite(pinIR, HIGH);
}

/**
  Turn off the infrared leds
*/
void Sensors::ledOffIR() {
  digitalWrite(pinIR, LOW);
}

/**
  Read the channels
*/
void Sensors::readAllChannels() {
  // Read each channel, while computing the relative values for all
  // but the last, then compute for the last
  for (uint8_t c = 0; c < CHANNELS; c++)
    chnRaw[c] = readChannel(c);
  // Compute the relative value for the last channel
  calcRelative(CHANNELS - 1);
}

/**
  Read one channel and calculate the relative value of the previous one
  while waiting for voltage to settle after changing the MUX.

  The calc routine takes 26us.  We need to wait 10us for an impedance
  smaller than 100k and 150us for 1M.
*/
uint8_t Sensors::readChannel(uint8_t channel) {
  // Set the MUX
  setChannel(channel);
  // Wait for voltage to settle after changing the MUX
  if (channel == 0) {
    // Can not compute anything for the first channel, just wait
    delayMicroseconds(MUX_DELAY_US);
  }
  else {
    // Compute the relative value of the previous channel
    calcRelative(channel - 1);
    // Then wait some more...
    delayMicroseconds(MUX_DELAY_US - 25);
  }
  // Read the channel
  return readRaw();
}

/**
  Set the MUX accordingly, also set left-adjusted
*/
void Sensors::setChannel(uint8_t channel) {
  // Set the analog reference to DEFAULT
  // Select the channel (lower 3 bits)
  // Set ADLAR (left-adjust result): 8 bits ADC
  // TODO: Needs external capacitor at AREF pin!
  ADMUX = _BV(REFS0) | _BV(ADLAR) | (channel & 0x07);
}

/**
  Read the channel: start conversion and wait to finish
  Return 8 bits
*/
uint8_t Sensors::readRaw() {
  // Start conversion
  ADCSRA |= _BV(ADSC);
  // Wait to finish (13 ADC cycles)
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
    delayMicroseconds(MUX_DELAY_US);
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
  Calibration and polarity histogram reset
*/
void Sensors::reset() {
  // Reset the channels calibration
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRaw[c] = 0x00;
    chnMin[c] = 0xFF;
    chnMax[c] = 0x00;
    chnRng[c] = 0x00;
  }
  // Reset the polarity histogram
  for (uint8_t i = 0; i < HST_SIZE; i++)
    polHst[i] = 0;
}

/**
  Get the surface polarity:
  + positive: black line on white background
  - negative: white line on black background
*/
bool Sensors::getPolarity() {
  uint16_t total, count;
  uint8_t  center;
  // Count all items in histogram
  for (uint8_t i = 0; i < HST_SIZE; i++)
    total += polHst[i];
  // Half that total
  total = total >> 1;
  // Find the binary distribution
  for (uint8_t center = 0; center < HST_SIZE; center++) {
    count += polHst[center];    // Accumulate
    if (count >= total) break;  // Stop when reaching half the total
  }
  polarity = center < 8;
  return polarity;
}

/**
  Calculate channel range: max-min
*/
bool Sensors::validate() {
  bool valid = true;
  for (uint8_t c = 0; c < CHANNELS; c++) {
    // Get the sensor range
    chnRng[c] = chnMax[c] - chnMin[c];
    // Validate (range should be greater than half the sensor definition)
    if (chnRng[c] < 0x7F) valid = false;
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
  chnVal[channel] = (uint8_t)((y << 8) / chnRng[channel]);
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

