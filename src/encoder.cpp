#include "encoder.h"

#include <Arduino.h>

#include <cmath>
#include <iomanip>
#include <sstream>

#include "globals.h"
#include "protocol_processor.h"

namespace {
    const int ENCODER_MAX_VALUE = 4096;  // 12 bit
    const unsigned long REFRESH_INTERVAL_MILLIS = 100;
    const float ZERO_ANGLE = 1090;
    const bool REVERSE = false;
}

Encoder::Encoder() : as5600(), prevAngle(0), prevTime(0) {
    ProtocolProcessor &P = GetProtocolProcessor();
    P.Log("Encoder init...");
    // TODO: Check if magnet is detected?
}

void Encoder::Poll() {
    unsigned long currTime = millis();
    if (currTime - prevTime < REFRESH_INTERVAL_MILLIS) return;
    float rawAngle = as5600.getRawAngle();
    float currAngle = (rawAngle - ZERO_ANGLE) / ENCODER_MAX_VALUE * 2 * PI;
    if (REVERSE) currAngle = 2 * PI - currAngle;
    
    float deltaAngle = currAngle - prevAngle;
    unsigned long deltaTime = currTime - prevTime;    
    Globals &G = GetGlobals();
    G.pole_v = deltaAngle / deltaTime * 1000;
    G.pole_x = prevAngle = currAngle;
}

Encoder &GetEncoder() {
    static Encoder encoder{};
    return encoder;
}
