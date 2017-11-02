/**
  RoboMitzy

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>
*/

#define DEBUG

#include "Sensors.h"

Sensors SNS;

/**
  Calibrate the sensors
*/
void snsCalibrate() {
  uint32_t timeDelay, timeStop;
  uint16_t count = 0;

  /*
    // Read the sensors to allow them to stabilize
    timeDelay = 2000UL;
    timeStop = millis() + timeDelay;
    SNS.reset();
    while (millis() < timeStop) {
      SNS.calibrate();
      count++;
    }
    #ifdef DEBUG
    Serial.print(F("Warm-up    : "));
    Serial.print(1000 * timeDelay / count / CHANNELS, 4);
    Serial.println(F("us"));
    #endif
  */

  do {
    // Calibration
    count = 0;
    SNS.reset();
    timeDelay = 3000UL;
    timeStop = millis() + timeDelay;
    while (millis() < timeStop) {
      SNS.calibrate();
      count++;
    }
#ifdef DEBUG
    Serial.print(F("Calibration: "));
    Serial.print(1000 * timeDelay / count / CHANNELS, 4);
    Serial.println(F("us"));
#endif

    // Get the polarity
    SNS.getPolarity();

#ifdef DEBUG
    Serial.println(F("Channel Min/Max:"));
    // Show the results
    for (uint8_t c = 0; c < CHANNELS; c++) {
      Serial.print(F("Ch"));
      if (c < 10) Serial.print(" ");
      Serial.print(c);
      Serial.print(" ");
      Serial.print(SNS.chnMin[c]);
      Serial.print(",");
      Serial.print(SNS.chnMax[c]);
      Serial.println();
    }
    // Show the polarity
    Serial.print(F("Polarity: "));
    if (SNS.polarity) Serial.println("positive (black on white).");
    else              Serial.println("negative (white on black).");
    for (uint8_t c = 0; c < 16; c++) {
      Serial.print(F("Pol "));
      if (c < 10) Serial.print(" ");
      Serial.print(c);
      Serial.print(" ");
      Serial.println(SNS.polHst[c]);
    }
#endif
  } while (not SNS.validate());
}


void snsRead() {
  uint32_t timeDelay, timeStop;
  uint16_t count = 0;
  uint16_t error;

  // Read the sensors, calibrated
  timeDelay = 2000UL;
  timeStop = millis() + timeDelay;
  while (millis() < timeStop) {
    //SNS.readAllChannels();
    error = SNS.getError();
    count++;
  }
#ifdef DEBUG
  Serial.print(F("Calib. read: "));
  Serial.print(1000 * timeDelay / count / CHANNELS, 4);
  Serial.println(F("us"));
  Serial.print(F("PID error  : "));
  Serial.println(error);
#endif

#ifdef DEBUG
  Serial.println(F("Channel reading:"));
  // Show the calibrated readings
  for (uint8_t c = 0; c < CHANNELS; c++) {
    Serial.print(F("Ch"));
    if (c < 10) Serial.print(" ");
    Serial.print(c);
    Serial.print(" ");
    Serial.print(SNS.chnVal[c]);
    Serial.print(",");
    Serial.print(SNS.chnRng[c]);
    Serial.println();
  }
#endif
}

/**
  Main Arduino setup function
*/
void setup() {
  Serial.begin(115200);
  Serial.println("RoboMitzy");

  // Initialize the analog line sensors
  SNS.init(3);

  // Calibrate and validate the sensors
  //snsCalibrate();

  // HALT
  //while (true) snsCalibrate();
}

/**
  Main Arduino loop
*/
void loop() {
  //Serial.println(SNS.getError());
  SNS.readAllChannels();
  for (uint8_t c = 0; c < CHANNELS; c++) {
    //Serial.print(SNS.chnRaw[c]);
    Serial.print(SNS.chnVal[c]);
    Serial.print(",");
  }
  Serial.println();

  delay(100);
}
