#include <Arduino.h>

#include "protocol_processor.h"
#include "stepper.h"

void setup() {}

void loop() {
    ProtocolProcessor &P = GetProtocolProcessor();
    Stepper &S = GetStepper();

    P.Poll();
    S.Poll();
}
