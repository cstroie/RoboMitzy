/**
  Sensors.h

  Copyright (c) 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef SENSORS_H
#define SENSORS_H

#define CHANNELS      8    // Number of ADC channels
#define HST_SIZE      16   // Histogram size

#define THRESHOLD     0x80

#define MUX_DELAY_US  50
#define CALC_DELAY_US 13

#include "Arduino.h"
#include "FixedPoint.h"

class Sensors {
  public:
    Sensors();
    void    init(uint8_t pin);
    void    ledOnIR();
    void    ledOffIR();

    void    readAllChannels();
    uint8_t readChannel(uint8_t channel);
    void    setChannel(uint8_t channel);
    uint8_t readRaw();

    bool    calibrate();
    bool    calibrated;

    void    coeff();
    int16_t getPosition();      // Compute the line position
    int16_t linePosition;       // Last line position
    bool    onLine;             // Check if the robot is on line

    void    reset();
    bool    getPolarity();
    bool    onFloor();          // Check if the robot has been lifted up

    // Channel readings, limits and computed values
    uint8_t chnRaw[CHANNELS];   // Raw analog channel readings
    uint8_t chnMax[CHANNELS];   // Maximum read value during calibration
    uint8_t chnMin[CHANNELS];   // Minimum read value during calibration
    uint8_t chnRng[CHANNELS];   // Channel range: max - min, precalculated
    uint8_t chnThr[CHANNELS];   // Channel threshohld
    uint8_t chnVal[CHANNELS];   // Calibrated channel digital value

    // Channel weights: x * 2^n, n=0..3, x<=4.63 for Q7.8
    float   chnWht = 2;
    int32_t chnCff[CHANNELS];   // Channel coefficients, computed at runtime, Q23.8

    bool    polarity = true;    // Surface polarity (black/white, white/black)
    uint16_t polHst[HST_SIZE];  // Polarity histogram

  private:
    uint8_t pinIR;

    // Test values
    uint8_t chnTst[CHANNELS] = {0, 0, 0, 0, 0xA0, 0, 0 ,0};
};

#endif /* SENSORS_H */
