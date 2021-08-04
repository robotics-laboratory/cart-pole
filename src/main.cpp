#include <Arduino.h>
#include "FastAccelStepper.h"
#include "TMCStepper.h"
#undef min
#undef max
#include <math.h>
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
    const float hw_max_v = 10;  // [m/s] Absolute max hardware-allowed velocity
    const float hw_max_a = 10;  // [m/s^2] Absolute max hardware-allowed acceleration
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

    float curr_x = 0;                 // [m] Current cart position
    float trgt_x = NAN;               // [m] Target cart position
    float curr_v = 0;                 // [m/s] Current cart velocity
    float trgt_v = NAN;               // [m/s] Target cart velocity
    float curr_a = 0;                 // [m/s^2] Current cart acceleration
    float trgt_a = NAN;               // [m/s^2] Target cart acceleration
    float pole_ang = 0;               // [rad] Current pole angle
    float pole_vel = 0;               // [rad/s] Current pole angular velocity
    float timestamp = 0;              // [s] Current internal timestamp
    ERR errcode = ERR::E1_NEED_INIT;  // Current error code
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
};

/* ===== DYNAMIC GET/SET ===== */

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
};

class FieldBase {
public:
    virtual ~FieldBase() = default;
    virtual string get();             // Return string representation of current value
    virtual void set(string &input);  // Parse input string, update field value
    virtual void reset();             // Reset to default
};

// Pre define
template <typename T, FID V>
class Field;

// Format field value to string
template <typename T, FID V>
struct format_field {
    string operator()(const Field<T, V> &field) {
        throw std::runtime_error("Not implemented");
    };
};

// Parse input string, validate, return new value
template <typename T, FID V>
struct parse_field {
    T operator()(const Field<T, V> &field, const string &input) {
        throw std::runtime_error("Not implemented");
    };
};

// Called when field value is about to update
// Can implement additional validation and/or field-specific logic
template <typename T, FID V>
void set_callback(Field<T, V> &field, T &new_value){
    // Default impl: do nothing
};

/* ===== FIELD DEFINITION ===== */

template <typename T, FID V>
class Field : public FieldBase {
public:
    T &value;
    const T default_value;
    const bool readonly;
    const bool allow_nan;

    Field(T &value, bool readonly, bool allow_nan)
        : value(value), readonly(readonly), allow_nan(allow_nan) {}

    string get() { return format_field<T, V>{}(this->value); }

    void set(string &input) {
        if (readonly) throw std::runtime_error("This field is readonly");
        T new_value = parse_field<T, V>{}(*this, input);
        set_callback(*this, new_value);
        this->value = value;
    }
};

// clang-format off
#define MAKE_FIELD(namespace, key, type, args...) \
{#key, std::unique_ptr<FieldBase>(new Field <type, FID::key>(namespace::key, args))}
// clang-format on

std::map<string, std::unique_ptr<FieldBase>> CONFIG_FIELDS{
    // MAKE_FIELD(NAMESPACE, KEY, TYPE, READONLY, ALLOW_NAN)
    MAKE_FIELD(config, max_v, float, false, false),
    MAKE_FIELD(config, clamp_x, bool, false, false),
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

/* ===== FORMAT IMPLEMENTATIONS ===== */

template <FID V>
struct format_field<float, V> {
    string operator()(const Field<float, V> &field) {
        return string_format("%.5f", field.value);
    };
};

template <FID V>
struct format_field<bool, V> {
    string operator()(const Field<bool, V> &field) {
        return field.value ? "true" : "false";
    };
};

template <FID V>
struct format_field<state::ERR, V> {
    string operator()(const Field<state::ERR, V> &field) {
        return string_format("%d", field.value);
    };
};

/* ===== PARSE IMPLEMENTATIONS ===== */

template <FID V>
struct parse_field<float, V> {
    float operator()(const Field<float, V> &field, const string &input) {
        try {
            float value = std::stof(input);
            if (!std::isfinite(value))
                throw std::runtime_error("Non-finite values not allowed");
            if (std::isnan(value))
                // TODO: Implement parse_field for trgt_x, trgt_v, trgt_a (allow nan)
                throw std::runtime_error("NaN not allowed");
            return value;
        } catch (std::logic_error) {
            throw std::runtime_error("Failed to parse float");
        }
    };
};

template <FID V>
struct parse_field<bool, V> {
    bool operator()(const Field<bool, V> &field, const string &input) {
        if (input == "true") return true;
        if (input == "false") return false;
        throw std::runtime_error("Failed to parse bool");
    };
};

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

    void force_stop() {
        misc::cmd_serial_port.println("# Stepper force stop");
        fas_stepper->forceStopAndNewPosition(fas_stepper->getCurrentPosition());
        delay(50);
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

        // RUN RIGHT
        fas_stepper->runForward();
        while (!digitalRead(pins::endstop_right)) {
        };
        force_stop();
        int delta_steps = fas_stepper->getCurrentPosition();
        fas_stepper->setCurrentPosition(delta_steps);

        // GOTO CENTER
        fas_stepper->moveTo(delta_steps / 2);
        while (fas_stepper->isRunning()) {
        };

        // UPDATE CONFIG
        float full_length = (float)delta_steps / tmc::microsteps / misc::full_steps_per_m;
        config::max_x = full_length / 2 - config::safe_margin;
        assert(config::max_x > 0);
        // TODO: Reset state?
        state::errcode = state::ERR::E0_NO_ERROR;
        misc::cmd_serial_port.printf("# Full length: %d steps, %.3f meters\n",
                                     delta_steps, full_length);
        misc::cmd_serial_port.printf("# Valid X range: %.3f ... %.3f\n", -config::max_x,
                                     config::max_x);
    }
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

    void get_keys(iss &in, oss &out,
                  std::map<string, std::unique_ptr<FieldBase>> &fields) {
        while (!in.eof()) {
            string key;
            in >> key;
            if (fields.find(key) == std::end(fields))
                throw std::runtime_error("Unknown key: " + key);
            FieldBase *field = fields[key].get();
            out << key << "=" << field->get() << " ";
        }
        out.seekp(-1, std::ios_base::end);
        out << '\0';
    }

    void set_keys(iss &in, std::map<string, std::unique_ptr<FieldBase>> &fields) {
        std::vector<std::pair<string, string>> kv_pairs;
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
            try {
                field->set(value);  // Not good
            } catch (std::runtime_error e) {
                throw std::runtime_error(e.what() +
                                         string_format(" [at %s=%s]", key, value));
            }
        }
    }

    void reset_keys(iss &in, std::map<string, std::unique_ptr<FieldBase>> &fields) {
        // TODO
    }

    void cmd_reset(iss &in, oss &out) {
        if (!out.eof()) throw std::runtime_error("Reset takes no arguments");
        stepper::homing();
        out << "ok";
    }

    void cmd_config_group(iss &in, oss &out) {
        string cmd;
        in >> cmd;
        if (cmd == "get") {
            get_keys(in, out, CONFIG_FIELDS);
        } else if (cmd == "set") {
            // TODO
        } else if (cmd == "reset") {
            // TODO
        } else {
            throw std::runtime_error("Invalid subcommand: " + cmd);
        }
    }

    void cmd_state_group(iss &in, oss &out) {}

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
