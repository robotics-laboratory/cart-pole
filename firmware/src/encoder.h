#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <cmath>
#include <deque>

#include "proto/protocol.pb.h"
#include "helpers.h"

namespace cartpole {

typedef uint32_t time_us;  // microseconds

class Encoder {
private:
    static const int I2C_PORT_NUM = 0;
    static const int I2C_SDA_PIN = -1;
    static const int I2C_SCL_PIN = -1;
    static const int I2C_SPEED = 1000000;
    static const uint32_t I2C_TIMEOUT = 1000;

    static const int AS5600_ADDRESS = 0x36;
    static const uint8_t RAWANGLE_ADDRESS_LSB = 0x0D;
    static const uint8_t RAWANGLE_ADDRESS_MSB = 0x0C;
    static const int ENCODER_MAX_VALUE = 4096;
    static const bool REVERSE = true;

    static const uint32_t POLLING_DELAY = 1000;
    static const uint32_t VELOCITY_TIMEDELTA = 20 * 1000;
    static constexpr float VELOCITY_SMOOTHING = 0.1;
    static constexpr float MAX_VELOCITY = 5 * 2 * PI;
    static constexpr float ROTATION_THRESHOLD = 1.8 * PI;

private:
    float zero;
    float angle;
    float velocity;
    int rotations;
    uint32_t time;
    std::deque<std::pair<uint32_t, float>> history;
    TwoWire wire;
    HardwareError error;

public:
    Encoder() : wire(I2C_PORT_NUM), error(HardwareError_NO_ERRORS) {}

    void init() {
        wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED);
        wire.setTimeOut(I2C_TIMEOUT);
        reset();
        CREATE_TASK("encoder", _pollingTask, this);
    }

    void reset() {
        error = HardwareError_NO_ERRORS;
        _update();
        if (error) return;
        zero -= angle;
        angle = 0;
        velocity = 0;
        rotations = 0;
        history.clear();
    }

    float getAngle() { return 2 * PI * rotations + angle; }

    float getVelocity() { return velocity; }

    HardwareError getErrors() { return error; }

private:
    static void _pollingTask(void *params) {
        Encoder *encoder = (Encoder *)params;
        while (true) {
            if (!encoder->getErrors()) encoder->_update();
            vTaskDelay(POLLING_DELAY / portTICK_PERIOD_MS / 1000);
        }
    }

    void _update() {
        if (error) return;
        uint32_t newTime = micros();
        float rawAngle = _getRawAngle();
        if (error) return;
        float newAngle = rawAngle / ENCODER_MAX_VALUE * 2 * PI - zero;
        if (newAngle < 0) newAngle += 2 * PI;
        if (REVERSE) newAngle = 2 * PI - newAngle;

        int newRotations = rotations;
        float momentaryDeltaAngle = newAngle - angle;
        float momentaryDeltaTime = static_cast<float>(newTime - time) / 1000000;
        if (std::abs(momentaryDeltaAngle) > ROTATION_THRESHOLD) {
            // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
            int rotationDelta = (momentaryDeltaAngle > 0) ? -1 : 1;
            newRotations += rotationDelta;
            momentaryDeltaAngle += 2 * PI * rotationDelta;
        }
        float momentaryVelocity = momentaryDeltaAngle / momentaryDeltaTime;
        if (std::abs(momentaryVelocity) > MAX_VELOCITY && !history.empty()) {
            // Spike detected!
            return;
        }

        uint32_t prevHistoryTime = newTime - 1;
        float prevHistoryAngle = newAngle;
        while (!history.empty()) {
            auto item = history.front();
            prevHistoryTime = item.first;
            prevHistoryAngle = item.second;
            if (newTime - item.first <= VELOCITY_TIMEDELTA) break;
            history.pop_front();
        }

        float deltaAngle = newAngle - prevHistoryAngle;
        float deltaTime = static_cast<float>(newTime - prevHistoryTime) / 1000000;
        if (std::abs(deltaAngle) > ROTATION_THRESHOLD) {
            // Fix angle delta if we're transitioning between 0 -> 2PI or 2PI -> 0
            deltaAngle -= (deltaAngle > 0) ? 2 * PI : -2 * PI;
        }
        float newVelocity =
            VELOCITY_SMOOTHING * (deltaAngle / deltaTime) + (1 - VELOCITY_SMOOTHING) * velocity;

        time = newTime;
        angle = newAngle;
        velocity = newVelocity;
        rotations = newRotations;
        history.emplace_back(newTime, newAngle);
    }

    uint16_t _getRawAngle() {
        uint16_t hi = _getRegister(RAWANGLE_ADDRESS_MSB);
        uint16_t lo = _getRegister(RAWANGLE_ADDRESS_LSB);
        return (hi << 8) | (lo);
    }

    uint8_t _getRegister(uint8_t reg) {
        wire.beginTransmission(AS5600_ADDRESS);
        wire.write(reg);
        if (wire.endTransmission()) {
            error = HardwareError_ENCODER_COMM_ERROR;
            return 0;
        }
        if (!wire.requestFrom(AS5600_ADDRESS, 1)) {
            error = HardwareError_ENCODER_COMM_ERROR;
            return 0;
        }
        return wire.read();
    }
};
}  // namespace cartpole
