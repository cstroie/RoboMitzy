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
    void init();
    void readAllChannels();
    uint8_t readChannel(uint8_t channel);
    void readMinMax();

    uint8_t chnRaw[CHANNELS];   // Raw analog channel read
    uint8_t chnMax[CHANNELS];   // Maximum read value during calibration
    uint8_t chnMin[CHANNELS];   // Maximum read value during calibration
    uint8_t chnVal[CHANNELS];   // Calibrated analog value

  private:
};

#endif /* SENSORS_H */
