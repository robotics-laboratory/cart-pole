#pragma once
#include "globals.h"
#include "AS5600.h"
#include <deque>

class Encoder {
    AS5600 as5600;
    float prevAngle;
    unsigned long prevTime;

public:
    Encoder();

    void Poll();
};

Encoder &GetEncoder();
