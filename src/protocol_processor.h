#pragma once
#include <Arduino.h>

#include <sstream>
#include <string>

class ProtocolProcessor {
    HardwareSerial serial_port;

public:
    ProtocolProcessor();

    void Poll();

    void Success(const std::string &text);
    void Log(const std::string &text);
    void Error(const std::string &text);
    void KeepAlive();

private:
    void handleCommand(const std::string &line);

    std::string get(const std::string &group, std::stringstream &stream);
    std::string set(const std::string &group, std::stringstream &stream);
    std::string reset();
};

ProtocolProcessor &GetProtocolProcessor();
