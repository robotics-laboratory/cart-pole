#pragma once
#include <FastAccelStepper.h>
#include <TMCStepper.h>

#include "freertos/task.h"
#include "helpers.h"
#include "proto/protocol.pb.h"

namespace cartpole {

class Stepper {
private:
    static const int TMC_EN = 25;
    static const int TMC_STEP = 33;
    static const int TMC_DIR = 32;
    static const int TMC_STALLGUARD = 39;
    static const int TMC_SERIAL_PORT_NUM = 2;

    static const int LEFT_ENDSTOP_PIN = 35;
    static const int RIGHT_ENDSTOP_PIN = 34;
    static const bool ENDSTOPS_ACTIVE_LOW = false;

    static const int STEPPER_CURRENT_MA = 2000;
    static constexpr float HOLD_CURRENT_RATIO = 0.8;
    static const int SERIAL_SPEED = 115200;
    static const int ADDRESS = 0b00;
    static constexpr float R_SENSE = 0.11;
    static const int TOFF_VALUE = 2;

    static const int MICROSTEPS = 16;
    static const bool REVERSE_STEPPER = true;
    static const int FULL_STEPS_PER_METER = 2000;
    static constexpr float HOMING_SPEED = 0.4;
    static constexpr float HOMING_ACCELERATION = 1.0;

    static const int METERS_TO_STEPS_MULTIPLIER = MICROSTEPS * FULL_STEPS_PER_METER;
    static constexpr float LIMITS_EPS = 1e-3;

private:
    HardwareSerial tmcSerial;
    TMC2209Stepper tmcDriver;
    FastAccelStepperEngine fasEngine;
    FastAccelStepper *fasStepper;
    HardwareError error;

    uint32_t fullRangeInSteps;
    bool enabled;
    bool homing;

public:
    std::function<void()> homingCallback = nullptr;

    Stepper()
        : tmcSerial(TMC_SERIAL_PORT_NUM),
          tmcDriver(&tmcSerial, R_SENSE, ADDRESS),
          error(HardwareError_NO_ERRORS),
          enabled(false),
          homing(false){};

    void init() {
        pinMode(TMC_EN, OUTPUT);
        pinMode(TMC_STEP, OUTPUT);
        pinMode(TMC_DIR, OUTPUT);
        // pinMode(TMC_STALLGUARD, INPUT);
        pinMode(LEFT_ENDSTOP_PIN, INPUT);
        pinMode(RIGHT_ENDSTOP_PIN, INPUT);

        int mode = ENDSTOPS_ACTIVE_LOW ? FALLING : RISING;
        attachInterruptArg(LEFT_ENDSTOP_PIN, _endstopISR, this, mode);
        attachInterruptArg(RIGHT_ENDSTOP_PIN, _endstopISR, this, mode);

        // TODO: Check tmc comm
        digitalWrite(TMC_EN, LOW);
        tmcSerial.begin(SERIAL_SPEED);
        tmcDriver.begin();
        tmcDriver.toff(TOFF_VALUE);
        tmcDriver.blank_time(24);
        tmcDriver.hysteresis_start(1);
        tmcDriver.hysteresis_end(12);
        tmcDriver.rms_current(STEPPER_CURRENT_MA, HOLD_CURRENT_RATIO);
        tmcDriver.seimin(1);
        tmcDriver.semin(15);
        tmcDriver.semax(15);
        tmcDriver.sedn(4);
        tmcDriver.seup(2);
        tmcDriver.iholddelay(3);
        tmcDriver.TPWMTHRS(0);
        tmcDriver.TCOOLTHRS(0);
        tmcDriver.pwm_autoscale(true);
        tmcDriver.en_spreadCycle(true);
        tmcDriver.microsteps(MICROSTEPS == 1 ? 0 : MICROSTEPS);
        tmcDriver.intpol(true);

        fasEngine.init();
        fasStepper = fasEngine.stepperConnectToPin(TMC_STEP);
        if (!fasStepper) {
            error = HardwareError_STEPPER_FAS_ENGINE_ERROR;
            return;
        }
        fasStepper->setDirectionPin(TMC_DIR, REVERSE_STEPPER);
    }

    bool isEnabled() { return enabled; }

    bool isHoming() { return homing; }

    float getFullRange() { return _stepsToMeters(fullRangeInSteps); }

    float getPosition() { return _stepsToMeters(fasStepper->getCurrentPosition()); }

    float getVelocity() { return _stepsToMeters(fasStepper->getCurrentSpeedInMilliHz() / 1000.0f); }

    float getAcceleration() { return _stepsToMeters(fasStepper->getCurrentAcceleration()); }

    void setMaxSpeed(float value) { fasStepper->setSpeedInHz(_metersToSteps(value)); }

    void setMaxAccel(float value) { fasStepper->setAcceleration(_metersToSteps(value)); }

    // void setAccel(float value) { fasStepper->setAcceleration(_metersToSteps(value)); }

    // void setSpeed(float value) {
    //     fasStepper->setSpeedInHz(std::abs(_metersToSteps(value)));
    //     (value > 0) ? fasStepper->runForward() : fasStepper->runBackward();
    // }

    void setPosition(float value) {
        if (!enabled) return;
        fasStepper->moveTo(_metersToSteps(value));
    }

    HardwareError getErrors() { return error; }

    void reset() {
        if (homing) return;
        homing = true;
        enabled = false;
        fasStepper->forceStopAndNewPosition(0);
        CREATE_TASK("homing", _homingTask, this);
    }

    void disable() {
        if (homing) return;
        enabled = false;
        fasStepper->forceStopAndNewPosition(0);
        // TODO: Maybe disable TMC?
    }

private:
    static void ARDUINO_ISR_ATTR _endstopISR(void *params) {
        Stepper *stepper = (Stepper *)params;
        if (stepper->enabled) {
            stepper->disable();
            stepper->error = HardwareError_STEPPER_ENDSTOP_HIT;
        }
    }

    static void _homingTask(void *params) {
        Stepper *stepper = (Stepper *)params;
        stepper->error = HardwareError_NO_ERRORS;
        stepper->_linearHoming();
        stepper->enabled = stepper->error == 0;
        stepper->homing = false;
        if (stepper->homingCallback) stepper->homingCallback();
        vTaskDelete(nullptr);
        while (true);
    }

    void _linearHoming() {
        // TODO: Decouple from stepper class
        setMaxAccel(HOMING_ACCELERATION);
        setMaxSpeed(HOMING_SPEED);
        fasStepper->forceStopAndNewPosition(0);

        fasStepper->runBackward();
        while (!_leftEndstop())
            RTOS_YIELD;
        fasStepper->forceStopAndNewPosition(0);

        fasStepper->runForward();
        while (!_rightEndstop())
            RTOS_YIELD;
        fullRangeInSteps = fasStepper->getCurrentPosition();
        fasStepper->forceStopAndNewPosition(fullRangeInSteps);

        fasStepper->moveTo(fullRangeInSteps / 2);
        while (fasStepper->isRunning()) {
            if (_leftEndstop()) {
                fasStepper->forceStopAndNewPosition(0);
                error = HardwareError_HOMING_FAILED;
                return;
            }
            RTOS_YIELD;
        }
        fasStepper->setCurrentPosition(0);
    }

    bool _leftEndstop() { return ENDSTOPS_ACTIVE_LOW ^ digitalRead(LEFT_ENDSTOP_PIN); }

    bool _rightEndstop() { return ENDSTOPS_ACTIVE_LOW ^ digitalRead(RIGHT_ENDSTOP_PIN); }

    int _metersToSteps(float meters) { return meters * METERS_TO_STEPS_MULTIPLIER; }

    float _stepsToMeters(int steps) { return (float)steps / METERS_TO_STEPS_MULTIPLIER; }
};

}  // namespace cartpole
