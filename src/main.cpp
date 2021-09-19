#include <Arduino.h>
#include <Wire.h>

#include "encoder.h"
#include "accelerometer.h"
#include "protocol_processor.h"

ProtocolProcessor &protocol = GetProtocolProcessor();
Encoder &encoder = GetEncoder();

void setup() {}

void loop() {
    GetStepper().Poll();
    GetPoleEncoder().Poll();
    GetMotorEncoder().Poll();
    GetAccelerometer().Poll();
    GetProtocolProcessor().Poll();
}
