#pragma once
#include "globals.h"
#include "AS5600.h"
#include <deque>

class Encoder {
    AS5600 as5600;
    unsigned long prevTime;
    float prevAngle;

public:
    Encoder();

    void Poll();
};

Encoder &GetEncoder();
