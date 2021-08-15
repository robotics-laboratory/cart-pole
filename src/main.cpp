#include <Arduino.h>

#include "encoder.h"
#include "protocol_processor.h"
#include "stepper.h"

void setup() {}

void loop() {
    ProtocolProcessor &P = GetProtocolProcessor();
    Encoder &E = GetEncoder();
    Stepper &S = GetStepper();

    P.Poll();
    E.Poll();
    S.Poll();
}
