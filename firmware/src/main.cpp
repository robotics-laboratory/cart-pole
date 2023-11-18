#include <Arduino.h>

#include "encoder.h"
#include "helpers.h"
#include "protocol.h"
#include "stepper.h"

using namespace cartpole;

// GLOBALS

State state = getDefaultState();
Config config = getDefaultConfig();

Encoder encoder;
Stepper stepper;
Protocol protocol;

// PROTOCOL CALLBACKS

State reset() {
    if (!stepper.isHoming()) {
        state.error = Error_NEED_RESET;
        stepper.reset();
    }
    return state;
}

State setTarget(Target target) {
    if (state.error) return state;
    state.error = validateTarget(target, config);
    if (state.error) {
        stepper.disable();
        return state;
    }

    // TODO: Prettify
    float velocity = target.has_velocity ? target.velocity : config.max_cart_velocity;
    float accel = target.has_acceleration ? target.acceleration : config.max_cart_acceleration;

    if (target.has_position) {
        stepper.setMaxAccel(accel);
        stepper.setMaxSpeed(velocity);
        stepper.setPosition(target.position);
    }
    // TODO: Velocity control, accel control
    return state;
}

Config setConfig(Config newConfig) {
    // ...
    return config;
}

// HELPER FUNCTIONS

void homingCallback() {
    if (!stepper.getErrors()) {
        config.max_cart_position = stepper.getFullRange() / 2;
        state.hardware_errors = HardwareError_NO_ERRORS;
        state.error = Error_NO_ERROR;
    }
}

void updateState() {
    state.cart_position = stepper.getPosition();
    state.cart_velocity = stepper.getVelocity();
    state.cart_acceleration = stepper.getAcceleration();
    state.pole_angle = encoder.getAngle();
    state.pole_angular_velocity = encoder.getVelocity();
    state.hardware_errors |= encoder.getErrors();
    state.hardware_errors |= stepper.getErrors();
    state.hardware_errors |= protocol.getErrors();
}

void checkErrors() {
    if (state.hardware_errors && state.error == Error_NO_ERROR) {
        state.error = Error_HARDWARE;
    } else if (std::abs(state.cart_position) > config.max_cart_position) {
        state.error = Error_CART_POSITION_OVERFLOW;
    } else if (std::abs(state.cart_velocity) > config.max_cart_velocity) {
        state.error = Error_CART_VELOCITY_OVERFLOW;
    } else if (std::abs(state.cart_acceleration) > config.max_cart_acceleration) {
        state.error = Error_CART_ACCELERATION_OVERFLOW;
    }
    if (state.error && state.error != Error_NEED_RESET) stepper.disable();
}

// ENTRYPOINT

void setup() {
    protocol.resetCallback = reset;
    protocol.targetCallback = setTarget;
    protocol.configCallback = setConfig;
    stepper.homingCallback = homingCallback;

    encoder.init();
    stepper.init();
    protocol.init();
}

void loop() {
    updateState();
    if (!stepper.isHoming()) checkErrors();
}
