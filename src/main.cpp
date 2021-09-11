#include <Arduino.h>

#include "encoder.h"
#include "accelerometer.h"
#include "protocol_processor.h"
#include "stepper.h"

void setup() {}

void loop() {
    GetStepper().Poll();
    GetEncoder().Poll();
    GetAccelerometer().Poll();
    GetProtocolProcessor().Poll();
}
