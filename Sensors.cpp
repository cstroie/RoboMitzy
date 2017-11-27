/**
  Sensors.cpp

  Copyright (c) 2017 Costin STROIE <costinstroie@eridu.eu.org>

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
  Read the channels (518us)
*/
void Sensors::readAllChannels() {
  // Read each channel, while computing the digital value
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRaw[c] = readChannel(c);
    //chnRaw[c] = chnTst[c];
    if (chnRaw[c] > chnThr[c])  chnVal[c] = true;
    else                        chnVal[c] = false;
  }
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
  delayMicroseconds(MUX_DELAY_US);
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
  Read the channels, update the minimum and maximum, compute the sensor
  range and threshold, validate the sensors and collect data for histogram
*/
bool Sensors::calibrate() {
  calibrated = true;
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
    // The range should be greater the specified THRESHOLD
    if (chnRng[c] < THRESHOLD) calibrated = false;
    // Get each sensor threshold
    chnThr[c] = chnMin[c] + (chnRng[c] >> 1);
  }
  // Collect data for polarity histogram if readings are valid
  if (calibrated) {
    for (uint8_t c = 0; c < CHANNELS; c++) {
      // Get the histogram index
      uint8_t idx = (chnRaw[c] >> 4);
      polHst[idx]++;
    }
  }
  return calibrated;
}

/**
  Calibration and polarity histogram reset
*/
void Sensors::reset() {
  // Reset the calibration
  calibrated = false;
  // Reset the channels calibration
  for (uint8_t c = 0; c < CHANNELS; c++) {
    chnRaw[c] = 0x00;
    chnMin[c] = 0xFF;
    chnMax[c] = 0x00;
    chnRng[c] = 0x00;
    chnThr[c] = 0xFF;
  }
  // Reset the polarity histogram
  for (uint8_t i = 0; i < HST_SIZE; i++)
    polHst[i] = 0;
  // Reset the line position
  linePosition = 0;
  onLine = false;
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
  Compute the position coefficients in Q23.8
*/
void Sensors::coeff() {
  int32_t x = FP_ONE;
  uint8_t chnHalf = CHANNELS >> 1;
  for (uint8_t c = 0; c < chnHalf; c++) {
    chnCff[chnHalf + c]      =  -x;
    chnCff[chnHalf - c - 1]  =   x;
    x *= chnWht;
  }
  //for (uint8_t c = 0; c < CHANNELS; c++) Serial.println(chnCff[c]);
}

/**
  Detect if the robot is on floor or it has been lifted up
*/
bool Sensors::onFloor() {
  bool proximity = false;
  for (uint8_t c = 0; c < CHANNELS; c++)
    if (not chnVal[c]) {
      proximity = true;
      break;
    }
  return proximity;
}

/**
  Get the line position for the PID controller, Q7.8 (540us)
*/
int16_t Sensors::getPosition() {
  int32_t result = 0;
  uint8_t count  = 0;
  // Read the sensors
  readAllChannels();
  // Compute the line position using the digital values and
  // the channels coefficients, Q23.8
  for (uint8_t c = 0; c < CHANNELS; c++)
    if (chnVal[c]) {
      result += chnCff[c];
      count++;
    }
  // Reverse the polarity
  if (not polarity) result = -result;
  // Get the average Q23.8 / Q8.0 = Q15.8
  if (count > 0) {
    // On line, compute an average and store it as last line position
    linePosition = constrain(fp_div(result, count), MIN16, MAX16);
    onLine = true;
  }
  else
    // Off line, just set the flag
    onLine = false;
  // Return the line position
  return linePosition;
}

