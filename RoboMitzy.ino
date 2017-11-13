/**
  RoboMitzy

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>
*/

//#define DEBUG

#define BENCH_COUNT 10000

#include "Sensors.h"
#include "FastPID.h"
#include "Motors.h"


Sensors SNS;
FastPID PID;
Motors  M;

uint8_t lastRun;  // The last time the loop ran

int16_t benchmark() {
  int16_t result;
  uint16_t count = BENCH_COUNT;
  uint32_t start = millis();
  while (count--)
    result = SNS.getPosition();
  // Print the benchmark result
  Serial.print(F("Loop: "));
  Serial.print(1000UL * (millis() - start) / BENCH_COUNT);
  Serial.print(F("us, Result: "));
  Serial.println(result);
  return result;
}

/**
  Calibrate the sensors
*/
void snsCalibrate() {
#ifdef DEBUG
  Serial.println(F("Calibration"));
#endif
  while (not SNS.calibrate()) {
#ifdef DEBUG
    // Show partial results
    Serial.print(F("Clb "));
    for (uint8_t c = 0; c < CHANNELS; c++) {
      Serial.print(SNS.chnMin[c]);
      Serial.print("/");
      Serial.print(SNS.chnMax[c]);
      Serial.print(" ");
    }
    Serial.println();
#endif
  }
  //SNS.calibrate();
  // Get the polarity
  SNS.getPolarity();
#ifdef DEBUG
  Serial.println();
  Serial.println(F("Channel Min/Max Rng"));
  // Show the results
  for (uint8_t c = 0; c < CHANNELS; c++) {
    Serial.print(F("Ch"));
    if (c < 10) Serial.print(" ");
    Serial.print(c);
    Serial.print(" ");
    Serial.print(SNS.chnMin[c]);
    Serial.print("/");
    Serial.print(SNS.chnMax[c]);
    Serial.print(" ");
    Serial.print(SNS.chnRng[c]);
    Serial.println();
  }
  // Show the polarity
  Serial.println();
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
  SNS.init(4);
  // Calibrate and validate the sensors
  snsCalibrate();
  // Force a positive polarity
  SNS.polarity = true;

  // Configure the PID controller
  PID.configure(1.1, 0.2, 0.1, 16, true);

  // Initialize the motors
  M.init();
}

/**
  Main Arduino loop
*/
void loop() {
  // Run at most once per millisecond
  if ((uint8_t)(millis() & 0xFF) != lastRun) {
    // Get the line position
    int16_t pos = SNS.getPosition();
    // If the robot has been lifted, stop the motors
    if (SNS.onFloor()) {
      // Get the controller correction
      int16_t stp = PID.step(-pos, 0);
      // Adjust the motors
      M.drive(M.maxSpeed, stp >> 8);

      Serial.print(pos >> 8);
      Serial.print(",");
      Serial.println(stp >> 8);
    }
    else {
      M.stop();
    }

    /*
      for (uint8_t c = 0; c < CHANNELS; c++) {
        Serial.print(SNS.chnRaw[c]);
        //Serial.print(SNS.chnVal[c]);
        Serial.print(",");
      }
    */

    // Keep the last byte of millis()
    lastRun = millis() & 0xFF;
  }

  //delay(20);
}
