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
  // Compute the position coefficients
  coeff();
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
  Read the channels; needs 542us (537us) at 16MHz
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

  The calc routine takes 13us.  We need to wait 10us for an impedance
  smaller than 100k and 150us for 1M.  We'll wait 50us
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
    delayMicroseconds(MUX_DELAY_US - CALC_DELAY_US);
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
  Calculate the relative sensor value; needs 13us at 16MHz
*/
void Sensors::calcRelative(uint8_t channel) {
  // Upscale to 16 bits
  uint16_t y = chnRaw[channel];
  // Constrain the raw value inside the calibrated interval
  constrain(y, chnMin[channel], chnMax[channel]);
  // Do the integer math
  y -= chnMin[channel];
  chnVal[channel] = (uint8_t)((y << 8) / ((uint16_t)chnRng[channel] + 1));
}

/**
  Read the channels, update the minimum and maximum, compute the sensor
  range, validate the sensors and collect data for histogram
*/
bool Sensors::calibrate() {
  bool valid = true;
  for (uint8_t c = 0; c < CHANNELS; c++) {
    // Set the MUX
    setChannel(c);
    // Wait for voltage to settle
    delayMicroseconds(MUX_DELAY_US);
    // Read the channel
    chnRaw[c] = readRaw();
    if (chnRaw[c] < chnMin[c]) chnMin[c] = chnRaw[c];
    if (chnRaw[c] > chnMax[c]) chnMax[c] = chnRaw[c];
    // Get the sensor range
    chnRng[c] = chnMax[c] - chnMin[c];
    // The range should be greater than 3/4 of the sensor definition
    if (chnRng[c] < THRESHOLD) valid = false;
  }
  // Collect data for polarity histogram if readings are valid
  if (valid) {
    for (uint8_t c = 0; c < CHANNELS; c++) {
      // Get the histogram index
      uint8_t idx = (chnRaw[c] >> 4);
      polHst[idx]++;
    }
  }
  return valid;
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
  Compute the position coefficients in Q24.8
*/
void Sensors::coeff() {
  int32_t x = FP_ONE;
  for (uint8_t c = 0; c < (CHANNELS / 2); c++) {
    chnCff[CHANNELS / 2 - c - 1]  = -x;
    chnCff[CHANNELS / 2 + c]      =  x;
    x = (int32_t)(((int64_t)x * (int64_t)chnWht + FP_ONE) >> FP_FBITS);
  }
  //for (uint8_t c = 0; c < CHANNELS; c++) Serial.println(chnCff[c]);
}

/**
  Detect if the robot is on floor or it has been lifted up
*/
bool Sensors::onFloor() {
  bool proximity = false;
  for (uint8_t c = 0; c < CHANNELS; c++)
    if (chnRaw[c] < 0xF0) {
      proximity = true;
      break;
    }
  return proximity;
}

/**
  Detect if the robot is on line
*/
bool Sensors::onLine() {
  bool line = false;
  for (uint8_t c = 0; c < CHANNELS; c++)
    if (chnRaw[c] > ((chnMax[c] - chnMin[c]) >> 1)) {
      line = true;
      break;
    }
  return line;
}

/**
  Get the line position for the PID controller (>675us)
*/
int16_t Sensors::getPosition() {
  int32_t result = 0;
  // Read the sensors (>540us)
  readAllChannels();
  // Compute the line position using the relative values and
  // the channels coefficients, Q24.8 (150us)
  if (polarity) {
    // Black on white
    for (uint8_t c = 0; c < CHANNELS; c++)
      result += ((fpd_t)((fp_t)chnVal[c] << FP_FBITS) * (fpd_t)chnCff[c]) >> FP_FBITS;
  }
  else {
    for (uint8_t c = 0; c < CHANNELS; c++)
      result += ((fpd_t)((fp_t)(255 - chnVal[c]) << FP_FBITS) * (fpd_t)chnCff[c]) >> FP_FBITS;
  }

  // Return the result
  return result >> FP_FBITS;
}

