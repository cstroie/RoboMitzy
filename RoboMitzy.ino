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
  Serial.println(F("Channel Min/Max Rng Thr"));
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
    Serial.print(" ");
    Serial.print(SNS.chnThr[c]);
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

/**
  Try an auto-calibration
*/
bool autoCalibrate(uint32_t howlong) {
  bool valid = true;
  bool right = true;
  uint32_t now = millis();
  uint32_t stop1 = now + (howlong >> 2);
  uint32_t stop2 = stop1 + (howlong >> 1);
  uint32_t stop3 = now + howlong;

  while (true) {
    // Keep the time for compares
    now = millis();
    if (now > stop3) {
      // Time's up
      M.stop(true);
      break;
    }
    else if ((now <= stop1 or now > stop2) and right) {
      // Rotate left
      M.run(255, false, 255, true);
      right = false;
#ifdef DEBUG
      Serial.print(F(" left"));
#endif
    }
    else if ((now > stop1 and now <= stop2) and (not right)) {
      // Rotate right
      M.run(255, true, 255, false);
      right = true;
#ifdef DEBUG
      Serial.print(F(" right"));
#endif
    }
    // Calibrate
    valid = SNS.calibrate();
  }
#ifdef DEBUG
  if (valid) Serial.println(F(" done!"));
  else       Serial.println(F(" failed"));
#endif
  return valid;
}

/**
  Main Arduino setup function
*/
void setup() {
  Serial.begin(115200);
  Serial.println(F("RoboMitzy"));

  // Initialize the analog line sensors
  SNS.init(3);
  // Initialize the motors, setting the minimum and maximum speed
  M.init(0, 40);
  // Calibrate and validate the sensors for one second,
  // repeat four times if not valid
  for (uint8_t c = 1; c <= 4; c++) {
#ifdef DEBUG
    Serial.print(F("Calibration "));
    Serial.print(c);
#endif
    if (autoCalibrate(320)) break;
  }
#ifdef DEBUG
  Serial.println();
  Serial.println(F("Channel Min/Max Rng Thr"));
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
    Serial.print(" ");
    Serial.print(SNS.chnThr[c]);
    Serial.println();
  }
#endif
  // If not calibrated, halt
  if (not SNS.calibrated) while (true);

  // Force a positive polarity
  SNS.polarity = true;

  // Wait a bit
  delay(5000);

  // Configure the PID controller
  PID.configure(2.2, 0.04, 0.2, 16, true);

  //while (true) benchmark();
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
      int16_t stp = PID.step(pos, 0);
      // Adjust the motors
      M.drive(127, stp >> 8);

#ifdef DEBUG
      // Show the results
      for (uint8_t c = 0; c < CHANNELS; c++)
        Serial.print(SNS.chnVal[c]);
      Serial.print(" ");
      Serial.print(pos >> 8);
      Serial.print(",");
      Serial.println(stp >> 8);
#endif
    }
    else
      M.stop();

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

#ifdef DEBUG
  //delay(20);
#endif
}
