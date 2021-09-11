#include <Arduino.h>
#include <Wire.h>

#include "encoder.h"
#include "protocol_processor.h"

ProtocolProcessor &protocol = GetProtocolProcessor();
Encoder &encoder = GetEncoder();

void setup() {}

void loop() {
    GetStepper().Poll();
    GetEncoder().Poll();
    GetAccelerometer().Poll();
    GetProtocolProcessor().Poll();
}
