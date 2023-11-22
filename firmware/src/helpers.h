#pragma once
#include "freertos/task.h"
#include "proto/protocol.pb.h"

#define RTOS_YIELD vTaskDelay(0);
#define DEFAULT_STACK_DEPTH 4096
#define DEFAULT_TASK_PRIORITY 1
#define CREATE_TASK(name, callback, params)                                                     \
    xTaskCreatePinnedToCore(callback, name, DEFAULT_STACK_DEPTH, params, DEFAULT_TASK_PRIORITY, \
                            nullptr, APP_CPU_NUM);

State getDefaultState() {
    State state = State_init_zero;
    state.error = Error_NEED_RESET;

    state.has_cart_position = true;
    state.has_cart_velocity = true;
    state.has_cart_acceleration = true;
    state.has_pole_angle = true;
    state.has_pole_angular_velocity = true;
    state.has_error = true;
    state.has_hardware_errors = true;

    return state;
}

Config getDefaultConfig() {
    Config config = Config_init_zero;
    config.max_cart_position = 0;
    config.max_cart_velocity = 10.0;
    config.max_cart_acceleration = 20.0;

    config.has_max_cart_position = true;
    config.has_max_cart_velocity = true;
    config.has_max_cart_acceleration = true;

    return config;
}

Error validateTarget(const Target &target, const Config &config) {
    if (target.has_position && std::abs(target.position) > config.max_cart_position) {
        return Error_CART_POSITION_OVERFLOW;
    }
    if (target.has_velocity && std::abs(target.velocity) > config.max_cart_velocity) {
        return Error_CART_VELOCITY_OVERFLOW;
    }
    if (target.has_acceleration && std::abs(target.acceleration) > config.max_cart_acceleration) {
        return Error_CART_ACCELERATION_OVERFLOW;
    }
    return Error_NO_ERROR;
}
