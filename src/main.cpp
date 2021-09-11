#include <Arduino.h>

#include "encoder.h"
#include "accelerometer.h"
#include "protocol_processor.h"
#include "stepper.h"

void setup() {}

void loop() {
    ProtocolProcessor &P = GetProtocolProcessor();
    Encoder &E = GetEncoder();
    Accelerometer &A = GetAccelerometer();
    Stepper &S = GetStepper();

    P.Poll();
    E.Poll();
    S.Poll();
    A.Poll();
}
