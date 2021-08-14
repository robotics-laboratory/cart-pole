#pragma once
#include <string>
#include <sstream>

#include <Arduino.h>


class ProtocolProcessor {
    HardwareSerial serial_port;
    
public:
    static constexpr int SERIAL_SPEED = 115200;
    static constexpr int REVERSE_STEPPER = true;
    static constexpr int FULL_STEPS_PER_METER = 5000;
    static constexpr float HOMING_SPEED = 0.1;
    static constexpr float HOMING_ACCELERATION = 0.5;
    static constexpr float MIN_STEPPER_CURRENT = 0.1;
    static constexpr float MAX_STEPPER_CURRENT = 2.0;

    ProtocolProcessor();

    void Poll();

    void Success(const std::string &text);
    void Log(const std::string &text);
    void Error(const std::string &text);
    void KeepAlive();

private:
    void handleCommand(std::stringstream &stream);

    std::string get(const std::string &group, std::stringstream &stream);
    std::string set(const std::string &group, std::stringstream &stream);
    std::string reset();
};

ProtocolProcessor &GetProtocolProcessor();
