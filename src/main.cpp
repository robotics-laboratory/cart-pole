#include <Arduino.h>

#include "accelerometer.h"
#include "protocol_processor.h"
#include "stepper.h"

void setup() {}

void loop() {
    ProtocolProcessor &P = GetProtocolProcessor();
    Accelerometer &A = GetAccelerometer();
    Stepper &S = GetStepper();

    P.Poll();
    S.Poll();
    A.Poll();
}
