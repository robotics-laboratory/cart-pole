#include <Arduino.h>

#include "protocol_processor.h"
#include "stepper.h"

ProtocolProcessor &P = GetProtocolProcessor();
Stepper &S = GetStepper();

void setup() {}

void loop() {
    P.Poll();
    S.Poll();
}
