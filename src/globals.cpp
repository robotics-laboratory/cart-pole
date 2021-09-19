#include "globals.h"
#include "stepper.h"
#include "encoder.h"

#include <iomanip>
#include <sstream>
#include <unordered_map>


namespace {
    const int DEBUG_LED_PIN = 2;

std::ostream &operator<<(std::ostream &out, Error error) { return out << static_cast<int>(error); }

    std::istream &operator >>(std::istream &in, Error &error) {
        int err = 0;
        in >> err;
        error = static_cast<Error>(err);
        return in;
    }

enum class FieldID {
    max_x,
    max_v,
    max_a,
    hw_max_x,
    hw_max_v,
    hw_max_a,
    clamp_x,
    clamp_v,
    clamp_a,
    curr_x,
    trgt_x,
    curr_v,
    trgt_v,
    curr_a,
    trgt_a,
    pole_x,
    pole_v,
    errcode,
    imu_a,
    motor_x,
    motor_v,
    debug_led,
};

struct FieldBase {
    virtual ~FieldBase() = default;
    virtual std::string Format() const;
    virtual void Prepare(const std::string &text);
    virtual void Commit();
    virtual void Reset();
};

template <typename T>
std::string formatField(T value) {
    std::stringstream stream;
    stream << value;
    return stream.str();
}

    enum class FieldID {
        max_x,
        max_v,
        max_a,
        hw_max_x,
        hw_max_v,
        hw_max_a,
        clamp_x,
        clamp_v,
        clamp_a,
        curr_x,
        trgt_x,
        curr_v,
        trgt_v,
        curr_a,
        trgt_a,
        pole_x,
        pole_v,
        errcode,
    };

    struct FieldBase {
        virtual ~FieldBase() = default;
        virtual std::string Format() const;
        virtual void Prepare(const std::string &text);
        virtual void Commit();
        virtual void Reset();
    };

    template <typename T>
    std::string formatField(T value) {
        std::stringstream stream;
        stream << value;
        return stream.str();
    }

    template <>
    std::string formatField<>(float value) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(5) << value;
        return stream.str();
    }

    template <>
    std::string formatField<>(bool value) {
        return value ? "true" : "false";
    }

    template <typename T>
    T parseField(const std::string &text) {
        std::stringstream stream(text);
        T value;
        stream >> value;
        if (stream.fail()) {
            throw std::runtime_error{"Failed to parse value"};
        }
        return value;
    }

    template <>
    bool parseField<>(const std::string &text) {
        if (text == "true") {
            return true;
        } else if (text == "false") {
            return false;
        }
        throw std::runtime_error("Failed to parse bool");
    }

    template <typename T, FieldID F>
    T validateField(T value) {
        return value;
    }

    float validateFloatRange(float value, float min, float max, bool clamp = false) {
        if (std::isinf(value)) {
            throw std::runtime_error{"Infinite values are not allowed"};
        }
        if (std::isnan(value)) {
            throw std::runtime_error{"NaN is not allowed"};
        }
        if (value <= min) {
            if (clamp) {
                return min;
            }
            std::stringstream stream;
            stream << std::fixed << std::setprecision(5) << "Out of range: " << value << " < " << min;
            throw std::runtime_error{stream.str()};
        }
        if (max <= value) {
            if (clamp) {
                return max;
            }
            std::stringstream stream;
            stream << std::fixed << std::setprecision(5) << "Out of range: " << value << " > " << max;
            throw std::runtime_error{stream.str()};
        }
        return value;
    }

    template <>
    float validateField<float, FieldID::max_x>(float value) {
        return validateFloatRange(value, 0, G.hw_max_x);
    }

    template <>
    float validateField<float, FieldID::max_v>(float value) {
        return validateFloatRange(value, 0, G.hw_max_v);
    }

    template <>
    float validateField<float, FieldID::max_a>(float value) {
        return validateFloatRange(value, 0, G.hw_max_a);
    }

    template <>
    float validateField<float, FieldID::trgt_x>(float value) {
        try {
            return validateFloatRange(value, -G.max_x, G.max_x, G.clamp_x);
        } catch (std::runtime_error &e) {
            G.errcode = Error::X_OVERFLOW;
            throw;
        }
    }

    template <>
    float validateField<float, FieldID::trgt_v>(float value) {
        try {
            return validateFloatRange(value, -G.max_v, G.max_v, G.clamp_v);
        } catch (std::runtime_error &e) {
            G.errcode = Error::V_OVERFLOW;
            throw;
        }
    }

    template <>
    float validateField<float, FieldID::trgt_a>(float value) {
        try {
            return validateFloatRange(value, -G.max_a, G.max_a, G.clamp_a);
        } catch (std::runtime_error &e) {
            G.errcode = Error::A_OVERFLOW;
            throw;
        }
    }

    template <typename T, FieldID F>
    void updateField(T value) {
        // do nothing
    }

    template <>
    void updateField<float, FieldID::max_v>(float value) {
        S.SetSpeed(value);
    }

    template <>
    void updateField<float, FieldID::max_a>(float value) {
        S.SetAcceleration(value);
    }

    template <>
    void updateField<float, FieldID::trgt_x>(float value) {
        S.SetTargetPosition(value);
    }

template <>
void updateField<bool, FieldID::debug_led>(bool value) {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    digitalWrite(DEBUG_LED_PIN, value);
}

template <>
void updateField<bool, FieldID::debug_led>(bool value) {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    digitalWrite(DEBUG_LED_PIN, value);
}

template <typename T, FieldID F, bool readonly = false>
class Field : public FieldBase {
    T &global_value;
    T pending_value;
    const T default_value;

    template <>
    void updateField<float, FieldID::trgt_a>(float value) {
        // S.SetTargetAcceleration(value);
    }

    template <typename T, FieldID F>
    class Field : public FieldBase {
        T &global_value;
        T pending_value;
        const T default_value;
        const bool readonly;

        void update() {
            updateField<T, F>(global_value);
        }

    public:
        Field(T &global_value, bool readonly)
            : global_value(global_value),
            pending_value(),
            default_value(global_value),
            readonly(readonly) {}

        std::string Format() const override {
            return formatField<T>(global_value);
        }

        void Prepare(const std::string &text) override {
            if (readonly) {
                throw std::runtime_error{"Readonly violation"};
            }
            pending_value = validateField<T, F>(parseField<T>(text));
        }

        void Commit() override {
            global_value = pending_value;
            update();
        }

FieldMap constructConfigFieldMap() {
    Globals &G = GetGlobals();
    FieldMap map;
    map["max_x"] = makeField<float, FieldID::max_x, false>(G.max_x);
    map["max_v"] = makeField<float, FieldID::max_v, false>(G.max_v);
    map["max_a"] = makeField<float, FieldID::max_a, false>(G.max_a);
    map["hw_max_x"] = makeField<float, FieldID::hw_max_x, true>(G.hw_max_x);
    map["hw_max_v"] = makeField<float, FieldID::hw_max_v, true>(G.hw_max_v);
    map["hw_max_a"] = makeField<float, FieldID::hw_max_a, true>(G.hw_max_a);
    map["clamp_x"] = makeField<bool, FieldID::clamp_x, false>(G.clamp_x);
    map["clamp_v"] = makeField<bool, FieldID::clamp_v, false>(G.clamp_v);
    map["clamp_a"] = makeField<bool, FieldID::clamp_a, false>(G.clamp_a);
    map["debug_led"] = makeField<bool, FieldID::debug_led, false>(G.clamp_a);
    return map;
}

FieldMap constructStateFieldMap() {
    Globals &G = GetGlobals();
    FieldMap map;
    map["x"] = makeField<float, FieldID::curr_x, true>(G.curr_x);
    map["v"] = makeField<float, FieldID::curr_v, true>(G.curr_v);
    map["a"] = makeField<float, FieldID::curr_a, true>(G.curr_a);
    map["pole_x"] = makeField<float, FieldID::pole_x, true>(G.pole_x);
    map["pole_v"] = makeField<float, FieldID::pole_v, true>(G.pole_v);
    map["errcode"] = makeField<Error, FieldID::errcode, true>(G.errcode);
    map["imu_a"] = makeField<float, FieldID::imu_a, true>(G.imu_a);
    map["motor_x"] = makeField<float, FieldID::motor_x, true>(G.motor_x);
    map["motor_v"] = makeField<float, FieldID::motor_v, true>(G.motor_v);
    return map;
}

    FieldMap constructStateFieldMap() {
        FieldMap map;
        MAKE_FIELD(x, curr_x, float, true)
        MAKE_FIELD(v, curr_v, float, true)
        MAKE_FIELD(a, curr_a, float, true)
        MAKE_FIELD(pole_x, pole_x, float, true)
        MAKE_FIELD(pole_v, pole_v, float, true)
        MAKE_FIELD(errcode, errcode, Error, true)
        return map;
    }

    FieldMap constructTargetFieldMap() {
        FieldMap map;
        MAKE_FIELD(x, trgt_x, float, false)
        MAKE_FIELD(v, trgt_v, float, false)
        MAKE_FIELD(a, trgt_a, float, false)
        return map;
    }

    FieldMap CONFIG_FIELDS = constructConfigFieldMap();
    FieldMap STATE_FIELDS = constructStateFieldMap();
    FieldMap TARGET_FIELDS = constructTargetFieldMap();

    FieldMap &lookupFieldMap(const std::string &group) {
        if (group == "config") {
            return CONFIG_FIELDS;
        } else if (group == "state") {
            return STATE_FIELDS;
        } else if (group == "target") {
            return TARGET_FIELDS;
        }
        throw std::runtime_error{"Unknown group"};
    }

    void resetFieldMap(FieldMap &map) {
        for (auto &&kv : map) {
            FieldBase *field = kv.second.get();
            field->Reset();
        }
    }
    return it->second.get();
}

std::string Globals::Get(const std::string &group, const std::string &key) const {
    const FieldMap &map = lookupFieldMap(group);
    return map.at(key)->Format();
}

std::vector<std::pair<std::string, std::string>> Globals::Get(const std::string &group) const {
    const FieldMap &map = lookupFieldMap(group);
    std::vector<std::pair<std::string, std::string>> res;
    for (auto &&kv : map) {
        res.emplace_back(kv.first, kv.second->Format());
    }
    return res;
}

void Globals::Prepare(const std::string &group, const std::string &key, const std::string &value) {
    FieldMap &map = lookupFieldMap(group);
    FieldBase *field = map.at(key).get();
    field->Prepare(value);
}

void Globals::Commit(const std::string &group, const std::string &key) {
    FieldMap &map = lookupFieldMap(group);
    FieldBase *field = map.at(key).get();
    field->Commit();
}

void Globals::Reset() {
    resetFieldMap(CONFIG_FIELDS);
    resetFieldMap(STATE_FIELDS);
    resetFieldMap(TARGET_FIELDS);
}

Globals &GetGlobals() {
    static Globals globals{
        /* CONFIG */
        0.0,                // [m] Absolute max cart position
        0.5,                // [m/s] Absolute max cart velocity
        1.0,                // [m/s^2] Absolute max cart acceleration
        0.0,                // [m] Absolute max hardware-allowed position
        10,                 // fixme  // [m/s] Absolute max hardware-allowed velocity
        10,                 // fixme  // [m/s^2] Absolute max hardware-allowed acceleration
        false,              // Clamp X to allowed range instead of raising error
        false,              // Clamp V to allowed range instead of raising error
        false,              // Clamp A to allowed range instead of raising error
        false,
        /* STATE */
        0.0,                // [m] Current cart position
        0.0,                // [m/s] Current cart velocity
        0.0,                // [m/s^2] Current cart acceleration
        0.0,                // [rad] Current pole angle
        0.0,                // [rad/s] Current pole angular velocity
        Error::NEED_RESET,  // Current error code
        0.0,
        0.0,
        0.0,
        /* TARGET */
        0.0,                // [m] Target cart position
        0.0,                // [m/s] Target cart velocity
        0.0,                // [m/s^2] Target cart acceleration
        /* MISC */
        0.0,        // [m] Total Length as determined during homing
    };
    return globals;
}
