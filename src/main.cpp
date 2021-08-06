#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#undef min
#undef max

#include <math.h>
#include <cstdio>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using std::string;

/* ===== GLOBALS ===== */

namespace config {
    float safe_margin = 0.01;   // [m] Reserved distance to real edges of workspace
    float max_x = 0;            // [m] Absolute max cart position
    float max_v = 0.5;          // [m/s] Absolute max cart velocity
    float max_a = 1.0;          // [m/s^2] Absolute max cart acceleration
    float hw_max_v = 10;        // [m/s] Absolute max hardware-allowed velocity
    float hw_max_a = 10;        // [m/s^2] Absolute max hardware-allowed acceleration
    bool clamp_x = false;       // Clamp X to allowed range instead of raising error
    bool clamp_v = false;       // Clamp V to allowed range instead of raising error
    bool clamp_a = false;       // Clamp A to allowed range instead of raising error
    float stepper_current = 1;  // [amps] Stepper RMS current (controlled by TMC)
};

namespace state {
    enum class ERR : int {
        E0_NO_ERROR = 0,        // This is fine
        E1_NEED_INIT = 1,       // Homing procedure is required ("reset" command)
        E2_X_OVERFLOW = 2,      // X overshoot detected, motion is disabled
        E3_V_OVERFLOW = 3,      // V overshoot detected, motion is disabled
        E4_A_OVERFLOW = 4,      // A overshoot detected, motion is disabled
        E5_STALL_DETECTED = 5,  // TMC StallGuard is triggered (stepper missed steps)
    };

    enum class ACTION : int {
        A1_X = 1,  // Current action is position control (X)
        A2_V = 2,  // Current action is velocity control (V)
        A3_A = 3,  // Current action is acceleration control (A)
    };

    float curr_x = 0;                 // [m] Current cart position
    float trgt_x = 0;                 // [m] Target cart position
    float curr_v = 0;                 // [m/s] Current cart velocity
    float trgt_v = 0;                 // [m/s] Target cart velocity
    float curr_a = 0;                 // [m/s^2] Current cart acceleration
    float trgt_a = 0;                 // [m/s^2] Target cart acceleration
    float pole_ang = 0;               // [rad] Current pole angle
    float pole_vel = 0;               // [rad/s] Current pole angular velocity
    float timestamp = 0;              // [s] Current internal timestamp
    ERR errcode = ERR::E1_NEED_INIT;  // Current error code
    ACTION action = ACTION::A1_X;     // Current action
};

namespace pins {
    const int tmc_en = 25;
    const int tmc_step = 26;
    const int tmc_dir = 27;
    const int endstop_left = 18;
    const int endstop_right = 19;
};

namespace tmc {
    auto serial_port = Serial2;
    const int serial_speed = 115200;
    const int address = 0b00;
    const float r_sense = 0.11;
    const int toff_value = 5;
    const int microsteps = 16;
};

namespace misc {
    auto cmd_serial_port = Serial;
    const int cmd_serial_speed = 115200;
    const bool reverse_stepper = true;
    const int full_steps_per_m = 5000;
    const float homing_speed = 0.1;
    const float homing_accel = 0.5;
    const float min_stepper_current = 0.1;
    const float max_stepper_current = 2.0;
    float full_length_m = 0;
};

/* ===== STRING-RELATED HELPERS ===== */

template <typename... Args>
string string_format(const string &format, Args... args) {
    // https://stackoverflow.com/a/26221725/6152172
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
    if (size_s <= 0) throw std::runtime_error("Error during formatting");
    auto size = static_cast<size_t>(size_s);
    auto buf = std::make_unique<char[]>(size);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return string(buf.get(), buf.get() + size - 1);
}

/* ===== STEPPER LOGIC ===== */

namespace stepper {
    TMC2209Stepper tmc_driver(&tmc::serial_port, tmc::r_sense, tmc::address);
    FastAccelStepperEngine fas_engine = FastAccelStepperEngine();
    FastAccelStepper *fas_stepper = nullptr;

    void init() {
        // Init pins
        pinMode(pins::tmc_en, OUTPUT);
        pinMode(pins::tmc_step, OUTPUT);
        pinMode(pins::tmc_dir, OUTPUT);
        pinMode(pins::endstop_left, INPUT);
        pinMode(pins::endstop_right, INPUT);

        // Init TMC driver
        digitalWrite(pins::tmc_en, LOW);
        delay(10);
        tmc::serial_port.begin(tmc::serial_speed);
        tmc_driver.begin();
        tmc_driver.rms_current(config::stepper_current * 1000);
        tmc_driver.microsteps(tmc::microsteps);
        tmc_driver.toff(0);

        // Init FastAccelStepper
        fas_engine.init();
        fas_stepper = fas_engine.stepperConnectToPin(pins::tmc_step);
        assert(fas_stepper != NULL);
        fas_stepper->setDirectionPin(pins::tmc_dir, misc::reverse_stepper);
    };

    void poll() {
        // ???
    }

    void enable() {
        Serial.println("# Stepper enabled");
        tmc_driver.toff(tmc::toff_value);
    }

    void disable() {
        Serial.println("# Stepper disabled");
        tmc_driver.toff(0);
    }

    void set_speed(float value) {
        uint32_t speed_hz = value * misc::full_steps_per_m * tmc::microsteps;
        Serial.printf("# Set stepper speed: %.5f m/s, %d steps/s\n", value, speed_hz);
        fas_stepper->setSpeedInHz(speed_hz);
    }

    void set_accel(float value) {
        uint32_t steps_per_ss = value * misc::full_steps_per_m * tmc::microsteps;
        Serial.printf("# Set stepper accel: %.5f m/s^2, %d steps/s^2\n", value,
                      steps_per_ss);
        fas_stepper->setAcceleration(steps_per_ss);
    }

    void set_current(float value) { tmc_driver.rms_current(value * 1000); }

    void force_stop() {
        misc::cmd_serial_port.println("# Stepper force stop");
        fas_stepper->forceStopAndNewPosition(fas_stepper->getCurrentPosition());
    }

    float get_current_pos() {
        int pos_steps = fas_stepper->getCurrentPosition();
        return (float)pos_steps / tmc::microsteps / misc::full_steps_per_m;
    }

    void set_target_pos(float value) {
        int pos_steps = value * misc::full_steps_per_m * tmc::microsteps;
        fas_stepper->moveTo(pos_steps);
    }

    void homing() {
        force_stop();
        enable();
        set_speed(misc::homing_speed);
        set_accel(misc::homing_accel);

        // RUN LEFT
        fas_stepper->runBackward();
        while (!digitalRead(pins::endstop_left)) {
        };
        force_stop();
        fas_stepper->setCurrentPosition(0);
        delay(50);

        // RUN RIGHT
        fas_stepper->runForward();
        while (!digitalRead(pins::endstop_right)) {
        };
        force_stop();
        int delta_steps = fas_stepper->getCurrentPosition();
        fas_stepper->setCurrentPosition(delta_steps);
        delay(50);

        // GOTO CENTER
        fas_stepper->moveTo(delta_steps / 2);
        while (fas_stepper->isRunning()) {
        };

        // UPDATE CONFIG
        misc::full_length_m =
            (float)delta_steps / tmc::microsteps / misc::full_steps_per_m;
        config::max_x = misc::full_length_m / 2 - config::safe_margin;
        assert(config::max_x > 0);
        // TODO: Reset state?
        state::errcode = state::ERR::E0_NO_ERROR;
        misc::cmd_serial_port.printf("# Full length: %d steps, %.3f meters\n",
                                     delta_steps, misc::full_length_m);
        misc::cmd_serial_port.printf("# Valid X range: %.3f ... %.3f\n", -config::max_x,
                                     config::max_x);
    }
};

/* ===== DYNAMIC FIELDS ===== */

enum class FID {  // Field ID
    safe_margin,
    max_x,
    max_v,
    max_a,
    hw_max_v,
    hw_max_a,
    clamp_x,
    clamp_v,
    clamp_a,
    stepper_current,
    curr_x,
    trgt_x,
    curr_v,
    trgt_v,
    curr_a,
    trgt_a,
    pole_ang,
    pole_vel,
    timestamp,
    errcode,
    action,
};

class FieldBase {
public:
    virtual ~FieldBase() = default;
    virtual string format();            // Current value -> string representation
    virtual void parse(string &input);  // Parse input string, save pending value
    virtual void validate();            // Validate pending value
    virtual void commit();              // Set current value = pending value
    virtual void update();              // Call field-specific callbacks
    virtual void reset();               // Reset field to default value, then update()
};

/* ===== FIELD DEFINITION ===== */

template <typename T>
string format_field(T value) {
    throw std::runtime_error("Not implemented");
}

template <typename T>
T parse_field(string &input) {
    throw std::runtime_error("Not implemented");
}

template <typename T, FID V>
T validate_field(T value) {
    return value;
}

template <typename T, FID V>
void update_field(T value) {
    // Do nothing
}

template <typename T, FID V>
class Field : public FieldBase {
public:
    T &value;
    T pending_value;
    const T default_value;
    const bool readonly;

    Field(T &value, bool readonly)
        : value(value), pending_value(value), default_value(value), readonly(readonly) {}

    string format() { return format_field<T>(value); }

    void parse(string &input) {
        if (readonly) throw std::runtime_error("This field is readonly");
        pending_value = parse_field<T>(input);
    };

    void validate() { pending_value = validate_field<T, V>(pending_value); }

    void commit() { value = pending_value; }

    void update() { update_field<T, V>(value); }

    void reset() {
        value = default_value;
        update();
    }
};

/* ===== FORMAT IMPLEMENTATIONS ===== */

template <>
string format_field<float>(float value) {
    return string_format("%.5f", value);
};

template <>
string format_field<bool>(bool value) {
    return value ? "true" : "false";
};

template <>
string format_field<state::ERR>(state::ERR value) {
    return string_format("%d", (int)value);
};

template <>
string format_field<state::ACTION>(state::ACTION value) {
    return string_format("%d", (int)value);
};

/* ===== PARSE IMPLEMENTATIONS ===== */

template <>
float parse_field<float>(string &input) {
    std::istringstream kek(input);
    float value;
    kek >> value;
    if (kek.fail()) throw std::runtime_error("Failed to parse float");
    if (std::isinf(value)) throw std::runtime_error("Infinite values not allowed");
    if (std::isnan(value)) throw std::runtime_error("NaN not allowed");
    return value;
};

template <>
bool parse_field<bool>(string &input) {
    if (input == "true") return true;
    if (input == "false") return false;
    throw std::runtime_error("Failed to parse bool");
};

/* ===== VALIDATE IMPLEMENTATIONS ===== */

float validate_min_max(float value, float min, float max, bool clamp = false) {
    if (value <= min) {
        if (clamp) return min;
        throw std::runtime_error(string_format("Out of range: %.5f < %.5f", value, min));
    }
    if (value >= max) {
        if (clamp) return max;
        throw std::runtime_error(string_format("Out of range: %.5f > %.5f", value, max));
    }
    return value;
}

template <>
float validate_field<float, FID::safe_margin>(float value) {
    if (state::errcode != state::ERR::E0_NO_ERROR)
        throw std::runtime_error("Homing is needed");
    return validate_min_max(value, 0, config::max_x / 2);
}

template <>
float validate_field<float, FID::max_v>(float value) {
    return validate_min_max(value, 0, config::hw_max_v);
}

template <>
float validate_field<float, FID::max_a>(float value) {
    return validate_min_max(value, 0, config::hw_max_a);
}

template <>
float validate_field<float, FID::stepper_current>(float value) {
    return validate_min_max(value, misc::min_stepper_current, misc::max_stepper_current);
}

template <>
float validate_field<float, FID::trgt_x>(float value) {
    try {
        return validate_min_max(value, -config::max_x, config::max_x, config::clamp_x);
    } catch (std::runtime_error &e) {
        state::errcode = state::ERR::E2_X_OVERFLOW;
        throw;
    }
}

template <>
float validate_field<float, FID::trgt_v>(float value) {
    try {
        return validate_min_max(value, -config::max_v, config::max_v, config::clamp_v);
    } catch (std::runtime_error &e) {
        state::errcode = state::ERR::E3_V_OVERFLOW;
        throw;
    }
}

template <>
float validate_field<float, FID::trgt_a>(float value) {
    try {
        return validate_min_max(value, -config::max_a, config::max_a, config::clamp_a);
    } catch (std::runtime_error &e) {
        state::errcode = state::ERR::E4_A_OVERFLOW;
        throw;
    }
}

/* ===== FIELD UPDATE CALLBACKS ===== */

template <>
void update_field<float, FID::safe_margin>(float value) {
    config::max_x = misc::full_length_m / 2 - config::safe_margin;
}

template <>
void update_field<float, FID::max_v>(float value) {
    stepper::set_speed(value);
}

template <>
void update_field<float, FID::max_a>(float value) {
    stepper::set_accel(value);
}

template <>
void update_field<float, FID::stepper_current>(float value) {
    stepper::set_current(value);
}

template <>
void update_field<float, FID::trgt_x>(float value) {
    stepper::set_target_pos(value);
    state::action = state::ACTION::A1_X;
}

template <>
void update_field<float, FID::trgt_v>(float value) {
    // TODO
    state::action = state::ACTION::A2_V;
}

template <>
void update_field<float, FID::trgt_a>(float value) {
    // TODO
    state::action = state::ACTION::A3_A;
}

/* ===== FIELD REGISTRY ===== */

// clang-format off
#define MAKE_FIELD(namespace, key, type, args...) \
{#key, std::unique_ptr<FieldBase>(new Field <type, FID::key>(namespace::key, args))}
// clang-format on

using fields_map_t = std::map<string, std::unique_ptr<FieldBase>>;

fields_map_t CONFIG_FIELDS{
    // MAKE_FIELD(NAMESPACE, KEY, TYPE, READONLY)
    MAKE_FIELD(config, safe_margin, float, false),
    MAKE_FIELD(config, max_x, float, true),
    MAKE_FIELD(config, max_v, float, false),
    MAKE_FIELD(config, max_a, float, false),
    MAKE_FIELD(config, hw_max_v, float, true),
    MAKE_FIELD(config, hw_max_a, float, true),
    MAKE_FIELD(config, clamp_x, bool, false),
    MAKE_FIELD(config, clamp_v, bool, false),
    MAKE_FIELD(config, clamp_a, bool, false),
    MAKE_FIELD(config, stepper_current, float, false),
};

fields_map_t STATE_FIELDS{
    // MAKE_FIELD(NAMESPACE, KEY, TYPE, READONLY)
    MAKE_FIELD(state, curr_x, float, true),
    MAKE_FIELD(state, trgt_x, float, false),
    MAKE_FIELD(state, curr_v, float, true),
    MAKE_FIELD(state, trgt_v, float, false),
    MAKE_FIELD(state, curr_a, float, true),
    MAKE_FIELD(state, trgt_a, float, false),
    MAKE_FIELD(state, pole_ang, float, true),
    MAKE_FIELD(state, pole_vel, float, true),
    MAKE_FIELD(state, timestamp, float, true),
    MAKE_FIELD(state, errcode, state::ERR, true),
    MAKE_FIELD(state, action, state::ACTION, true),
};

/* ===== PROTOCOL PARSER LOGIC ===== */

namespace protocol {
    using iss = std::istringstream;
    using oss = std::ostringstream;

    void init() {
        misc::cmd_serial_port.begin(misc::cmd_serial_speed);
        while (!misc::cmd_serial_port) {
        };  // Wait for init
        misc::cmd_serial_port.println("# CARTPOLE CONTROLLER");
    }

    void parse_keys(iss &in, fields_map_t &fields, std::vector<string> &keys) {
        while (!in.eof()) {
            string key;
            in >> key;
            if (fields.find(key) == std::end(fields))
                throw std::runtime_error("Unknown key: " + key);
            keys.push_back(key);
        }
    }

    void print_keys(oss &out, fields_map_t &fields, std::vector<string> &keys) {
        for (auto &&key : keys) {
            FieldBase *field = fields[key].get();
            out << key << "=" << field->format() << " ";
        }
        out.seekp(-1, std::ios_base::end);
        out << '\0';
    }

    void set_keys(iss &in, fields_map_t &fields, std::vector<string> &keys) {
        while (!in.eof()) {
            string key_value;
            in >> key_value;
            string::size_type split = key_value.find('=');
            if (split == string::npos)
                throw std::runtime_error("Failed to parse key=value pair: " + key_value);
            string key = key_value.substr(0, split);
            string value = key_value.substr(split, string::npos);
            if (fields.find(key) == std::end(fields))
                throw std::runtime_error("Unknown key: " + key);
            FieldBase *field = fields[key].get();
            field->parse(value);
            field->validate();
            keys.push_back(key);
        }
        for (auto &&key : keys) {
            FieldBase *field = fields[key].get();
            field->update();
        }
    }

    void reset_keys(fields_map_t &fields, std::vector<string> &keys) {
        for (auto &&key : keys) {
            FieldBase *field = fields[key].get();
            field->reset();
        }
    }

    void cmd_config_group(iss &in, oss &out) {
        string cmd;
        in >> cmd;
        std::vector<string> keys;
        if (cmd == "get") {
            parse_keys(in, CONFIG_FIELDS, keys);
            print_keys(out, CONFIG_FIELDS, keys);
        } else if (cmd == "set") {
            set_keys(in, CONFIG_FIELDS, keys);
            print_keys(out, CONFIG_FIELDS, keys);
        } else if (cmd == "reset") {
            parse_keys(in, CONFIG_FIELDS, keys);
            reset_keys(CONFIG_FIELDS, keys);
            print_keys(out, CONFIG_FIELDS, keys);
        } else {
            throw std::runtime_error("Invalid subcommand: " + cmd);
        }
    }

    void cmd_state_group(iss &in, oss &out) {
        string cmd;
        in >> cmd;
        std::vector<string> keys;
        if (cmd == "get") {
            parse_keys(in, STATE_FIELDS, keys);
            print_keys(out, STATE_FIELDS, keys);
        } else if (cmd == "set") {
            set_keys(in, STATE_FIELDS, keys);
            print_keys(out, STATE_FIELDS, keys);
        } else if (cmd == "reset") {
            parse_keys(in, STATE_FIELDS, keys);
            reset_keys(STATE_FIELDS, keys);
            print_keys(out, STATE_FIELDS, keys);
        } else if (cmd == "echo") {
            throw std::runtime_error("Not implemented (yet)");
        } else {
            throw std::runtime_error("Invalid subcommand: " + cmd);
        }
    }

    void cmd_reset(iss &in, oss &out) {
        if (!out.eof()) throw std::runtime_error("Reset takes no arguments");
        stepper::homing();
        out << "ok";
    }

    void handle_command(iss &in, oss &out) {
        try {
            string cmd;
            in >> cmd;
            if (cmd == "reset") {
                return cmd_reset(in, out);
            } else if (cmd == "config") {
                return cmd_config_group(in, out);
            } else if (cmd == "state") {
                return cmd_state_group(in, out);
            } else {
                throw std::runtime_error("Unknown command: " + cmd);
            }
        } catch (std::runtime_error e) {
            out.clear();
            out << "! ERR: " << e.what();
        }
    }

    void poll() {
        static string buff = "";
        while (misc::cmd_serial_port.available()) {
            char c = misc::cmd_serial_port.read();
            if (c == '\n') {
                iss in(buff);
                oss out;
                handle_command(in, out);
                misc::cmd_serial_port.println(out.str().c_str());
                buff = "";
                break;
            } else {
                buff += std::tolower(c);
            }
        }
    }
}

/* ===== ENTRYPOINT ===== */

void setup() {
    stepper::init();
    protocol::init();
};

void loop() {
    stepper::poll();
    protocol::poll();
};
