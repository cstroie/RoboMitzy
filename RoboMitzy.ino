/**
  RoboMitzy

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>
*/


#include "Sensors.h"

Sensors SNS;

/**
  Calibrate the sensors
*/
void snsCalibrate() {
  // Calibrate the sensors for 5 seconds
  uint32_t timeout;
  uint16_t count = 0;
  // Read the sensors to allow them to stabilize
  timeout = millis() + 2000UL;
  while (millis() < timeout) {
    //SNS.readAllChannels();
    SNS.calibrate();
    count++;
  }
  Serial.println(2000.0 / count / CHANNELS, 4);

  count = 0;
  SNS.polReset();
  // Read again the sensors, keeping the minimum and maximum for each one
  timeout = millis() + 3000UL;
  while (millis() < timeout) {
    // FIXME Do motors
    SNS.calibrate();
    count++;
  }
  Serial.println(3000.0 / count / CHANNELS, 4);


  for (uint8_t c = 0; c < 16; c++) {
    Serial.print(c);
    Serial.print(" ");
    Serial.println(SNS.polHst[c]);
  }


  // Precalculate the channel span
  SNS.calcSpan();

  // Show the results
  for (uint8_t c = 0; c < CHANNELS; c++) {
    Serial.print(c);
    Serial.print(" ");
    Serial.print(SNS.chnMin[c]);
    Serial.print(",");
    Serial.print(SNS.chnMax[c]);
    Serial.println();
  }

  // Read again the sensors, calibrated
  timeout = millis() + 5000UL;
  while (millis() < timeout) {
    SNS.readAllChannels();
    count++;
  }
  Serial.println(5000.0 / count / CHANNELS, 4);

  // Show the calibrated readings
  for (uint8_t c = 0; c < CHANNELS; c++) {
    Serial.print(c);
    Serial.print(" ");
    Serial.print(SNS.chnVal[c]);
    Serial.print(",");
    Serial.print(SNS.chnSpn[c]);
    Serial.println();
  }



}

/**
  Main Arduino setup function
*/
void setup() {
  Serial.begin(115200);
  Serial.println("RoboMitzy");

  // Initialize the analog line sensors
  SNS.init(3);

  // Calibrate
  snsCalibrate();
  // HALT
  //while (true);
}

/**
  Main Arduino loop
*/
void loop() {
  //Serial.println(SNS.getError());
  SNS.readAllChannels();
  for (uint8_t c = 0; c < CHANNELS; c++) {
    Serial.print(SNS.chnRaw[c]);
    Serial.print(",");
  }
  Serial.println();

  delay(100);
}
