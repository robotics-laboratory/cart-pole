#include <Arduino.h>

#include "protocol_processor.h"
#include "stepper.h"
#include "accelerometer.h"

void setup() {}

void loop() {
    ProtocolProcessor &P = GetProtocolProcessor();
    Accelerometer &A = GetAccelerometer();
    Stepper &S = GetStepper();

    P.Poll();
    S.Poll();
    A.Poll();
}
