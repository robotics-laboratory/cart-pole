#include "protocol_processor.h"

#include <vector>

#include "globals.h"
#include "stepper.h"

namespace {
const HardwareSerial PROTOCOL_SERIAL_PORT = Serial;
const int SERIAL_SPEED = 115200;

using ErrorEnum = Error;
}  // namespace

ProtocolProcessor::ProtocolProcessor() : serial_port(PROTOCOL_SERIAL_PORT) {
    serial_port.begin(SERIAL_SPEED);
    while (!serial_port) {
    }  // Wait for init
    Log("CARTPOLE CONTROLLER STARTED");
}

void ProtocolProcessor::Poll() {
    static std::string buffer;
    while (serial_port.available()) {
        char c = serial_port.read();
        if (c == '\n') {
            handleCommand(buffer);
            buffer.clear();
            break;
        }
        buffer += std::tolower(c);
    }
}

void ProtocolProcessor::handleCommand(const std::string &line) {
    std::stringstream stream{line};

    std::string command;
    stream >> command;

    std::string result;

    try {
        if (command == "get") {
            std::string group;
            stream >> group;
            result = get(group, stream);
        } else if (command == "set") {
            std::string group;
            stream >> group;
            result = set(group, stream);
        } else if (command == "reset") {
            result = reset();
        } else {
            throw std::runtime_error{"Unknown command: " + command};
        }
    } catch (std::exception &e) {
        Error(e.what());
        return;
    }

    Success(result);
}

void ProtocolProcessor::Success(const std::string &text) {
    serial_port.printf("+ %s\n", text.c_str());
}

void ProtocolProcessor::Log(const std::string &text) { serial_port.printf("# %s\n", text.c_str()); }

void ProtocolProcessor::Error(const std::string &text) {
    serial_port.printf("! %s\n", text.c_str());
}

void ProtocolProcessor::KeepAlive() { serial_port.printf("~\n"); }

std::string ProtocolProcessor::get(const std::string &group, std::stringstream &stream) {
    Globals &G = GetGlobals();
    std::stringstream result;

    std::string key;
    bool first = true;
    while (stream >> key) {
        if (first) {
            first = false;
        } else {
            result << ' ';
        }
        result << key << "=" << G.Get(group, key);
    }
    if (first) {
        auto res = G.Get(group);
        for (auto &&kv : res) {
            if (first) {
                first = false;
            } else {
                result << ' ';
            }
            result << kv.first << '=' << kv.second;
        }
    }

    return result.str();
}

std::string ProtocolProcessor::set(const std::string &group, std::stringstream &stream) {
    Globals &G = GetGlobals();
    if (G.errcode != ErrorEnum::NO_ERROR) {
        throw std::runtime_error{"Global error code is set"};
    }

    std::stringstream result;

    std::vector<std::string> request_keys;
    std::string kv;

    while (stream >> kv) {
        size_t pos = kv.find('=');
        if (pos == std::string::npos) {
            throw std::runtime_error{"Incorrect key-value pair format"};
        }
        std::string key = kv.substr(0, pos);
        std::string value = kv.substr(pos + 1);
        G.Prepare(group, key, value);
        request_keys.emplace_back(std::move(key));
    }

    bool first = true;
    for (const auto &key : request_keys) {
        G.Commit(group, key);
        if (first) {
            first = false;
        } else {
            result << ' ';
        }
        result << key << '=' << G.Get(group, key);
    }

    return result.str();
}

std::string ProtocolProcessor::reset() {
    Stepper &S = GetStepper();
    Globals &G = GetGlobals();

    S.AsyncHoming();
    while (!S.IsDoneHoming()) {
        KeepAlive();
        delay(100);
    }
    KeepAlive();
    delay(100);

    float hw_max_x = G.hw_max_x;
    G.Reset();
    G.errcode = ErrorEnum::NO_ERROR;
    G.hw_max_x = hw_max_x;
    G.max_x = G.hw_max_x;

    return "";
}

ProtocolProcessor &GetProtocolProcessor() {
    static ProtocolProcessor processor{};
    return processor;
}
