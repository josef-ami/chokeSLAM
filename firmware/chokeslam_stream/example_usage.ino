// Minimal example: how an existing sketch hands its values to
// chokeslam_stream.h. Replace the three placeholder functions with your
// sketch's own encoder / IMU code (they must follow the rules in the
// header: cumulative forward-positive count, RAW yaw, validity flag).
#include <Arduino.h>
#include "chokeslam_stream.h"

int32_t myEncoderCount()   { return 0; }     // e.g. -(int32_t)TIM5->CNT
float   myRawYawDeg()      { return 0.0f; }  // raw Game Rotation Vector yaw
bool    myYawIsFresh()     { return false; } // e.g. millis() - lastYawMs <= 100

void setup() {
  Serial.begin(115200);
  // ...your existing setup...
}

void loop() {
  // ...your existing loop (it must not print to Serial)...
  chokeslamStreamPoll(myEncoderCount(), myRawYawDeg(), myYawIsFresh());
}
