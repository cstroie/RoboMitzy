/**
  RoboMitzy

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>
*/

#define DEBUG

#include "Sensors.h"
#include "FastPID.h"

Sensors SNS;
FastPID PID;

void benchmark() {
  Serial.println(F("Benchmark "));
  uint16_t count = 10000;
  uint32_t timeStart = millis();
  while (count--) {
    SNS.calcRelative(0);
  }
  Serial.print(1000UL * (millis() - timeStart) / 10000);
  Serial.println(F("us"));
}

/**
  Calibrate the sensors
*/
void snsCalibrate() {
  SNS.reset();
  //while (not SNS.calibrate());
  SNS.calibrate();
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
}


void snsRead() {
  uint32_t timeDelay, timeStop;
  uint32_t count = 0;
  uint16_t error;

  // Read the sensors, calibrated
  timeDelay = 2000UL;
  timeStop = millis() + timeDelay;
  while (millis() < timeStop) {
    SNS.readAllChannels();
    //error = SNS.getError();
    //SNS.calcRelative(0);
    count++;
  }
#ifdef DEBUG
  Serial.print(F("Calib. read 8ch: "));
  Serial.print(1000UL * timeDelay / count);
  Serial.println(F("us"));
  Serial.println(count);
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
  snsCalibrate();

  // Benchmarking
  //benchmark();

  // HALT
  //while (true); // snsCalibrate();

  PID.configure(5, 3, 2, 0, 16, true);


  uint32_t timeDelay, timeStop;
  uint32_t count = 0;
  // Read the sensors, calibrated
  timeDelay = 2000UL;
  timeStop = millis() + timeDelay;
  int16_t pos;
  int16_t step;

  while (millis() < timeStop) {
    pos = SNS.getPosition();
    step = PID.step(pos, 0);
    count++;
  }
#ifdef DEBUG
  Serial.print(F("PID: "));
  Serial.print(1000UL * timeDelay / count);
  Serial.println(F("us"));
  Serial.println(count);
#endif
  while (true);

}

/**
  Main Arduino loop
*/
void loop() {
  int16_t pos = SNS.getPosition();
  int16_t step = PID.step(pos, 0);
  for (uint8_t c = 0; c < CHANNELS; c++) {
    //Serial.print(SNS.chnRaw[c]);
    Serial.print(SNS.chnVal[c]);
    Serial.print(",");
  }
  Serial.println(step);

  delay(100);
}
