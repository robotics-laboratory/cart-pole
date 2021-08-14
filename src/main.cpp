#include <Arduino.h>

#include "protocol_processor.h"

ProtocolProcessor &P = GetProtocolProcessor();

void setup() {
}

void loop() { 
    P.Poll();
}