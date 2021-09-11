#include "accelerometer.h"

#include <Wire.h>

#include <sstream>

#include "protocol_processor.h"
#include "globals.h"

namespace {
    int SENSETIVITY = MPU6050_ACCEL_FS_2;  // 16384 LSB / g
    float G_ACCELERATION = 9.81509;

    float convertToG(int16_t lsb, int sensetivity = SENSETIVITY) {
        switch (sensetivity) {
        case MPU6050_ACCEL_FS_2: return static_cast<float>(lsb) / 16384;
        case MPU6050_ACCEL_FS_4: return static_cast<float>(lsb) / 8192;
        case MPU6050_ACCEL_FS_8: return static_cast<float>(lsb) / 4096;
        case MPU6050_ACCEL_FS_16: return static_cast<float>(lsb) / 2048;
        }
        throw std::runtime_error{"Unknown sensetivity in lsb to G conversion"};
    }
}

Accelerometer::Accelerometer() : mpu() {
    ProtocolProcessor &P = GetProtocolProcessor();

    Wire.begin();
    mpu.initialize();

    if (mpu.testConnection()) {
        P.Log("Accelerometer connection established");
    } else {
        P.Error("Failed to initialize accelerometer");
        throw std::runtime_error{"Failed to initialize accelerometer"};
    }
    mpu.setFullScaleAccelRange(SENSETIVITY);
}

void Accelerometer::Poll() {
    Globals &G = GetGlobals();

    int16_t y = mpu.getAccelerationY();
    G.curr_a = convertToG(y) / G_ACCELERATION;
}

Accelerometer &GetAccelerometer() {
    static Accelerometer A{};
    return A;
}