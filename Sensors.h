/**
  Sensors.h

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef SENSORS_H
#define SENSORS_H

#define CHANNELS 8

#include "Arduino.h"

class Sensors {
  public:
    Sensors();
    void init(uint8_t pin);
    void ledOnIR();
    void ledOffIR();
    
    void readAllChannels();
    uint8_t readChannel(uint8_t channel);
    void setChannel(uint8_t channel);
    uint8_t readRaw();
    void calibrate();
    bool validate();
    void calcRelative(uint8_t channel);

    int16_t getError();

    void polReset();
    bool getPolarity();


    uint8_t chnRaw[CHANNELS];   // Raw analog channel read
    uint8_t chnMax[CHANNELS];   // Maximum read value during calibration
    uint8_t chnMin[CHANNELS];   // Maximum read value during calibration
    uint8_t chnRange[CHANNELS]; // Channel range: max - min, precalculated
    uint8_t chnVal[CHANNELS];   // Calibrated analog value
    // Channel weights
    int8_t  chnWht[CHANNELS] = { -4, -3, -2, -1, 1, 2, 3, 4};

    bool polarity;              // Surface polarity (white/black, black/white)
    uint16_t polHst[16] = {0};  // Polarity histogram

  private:
    uint8_t pinIR;
};

#endif /* SENSORS_H */
