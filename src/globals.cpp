#include "globals.h"

#include "stepper.h"

#include <iomanip>
#include <sstream>
#include <unordered_map>


namespace {
    std::ostream &operator <<(std::ostream &out, Error error) {
        return out << static_cast<int>(error);
    }

    std::istream &operator >>(std::istream &in, Error &error) {
        int err = 0;
        in >> err;
        error = static_cast<Error>(err);
        return in;
    }

    Globals &G = GetGlobals();
    Stepper &S = GetStepper();


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
        S.SetSpeed(value);  // fixme
    }

    template <>
    void updateField<float, FieldID::max_a>(float value) {
        S.SetAcceleration(value);  // fixme
    }

    template <>
    void updateField<float, FieldID::trgt_x>(float value) {
        S.SetTargetPosition(value);
    }

    template <>
    void updateField<float, FieldID::trgt_v>(float value) {
        // S.SetTargetVelocity(value);
    }

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

        void Reset() override {
            global_value = default_value;
            update();
        }
    };

    // clang-format off
    #define MAKE_FIELD(name, key, type, args...) \
    map[#name] = std::move(std::unique_ptr<Field<type, FieldID::key>>(new Field<type, FieldID::key>(G.key, args)));
    // clang-format on

    using FieldMap = std::unordered_map<std::string, std::unique_ptr<FieldBase>>;

    FieldMap constructConfigFieldMap() {
        FieldMap map;
        MAKE_FIELD(max_x, max_x, float, false)
        MAKE_FIELD(max_v, max_v, float, false)
        MAKE_FIELD(max_a, max_a, float, false)
        MAKE_FIELD(hw_max_x, hw_max_x, float, true)
        MAKE_FIELD(hw_max_v, hw_max_v, float, true)
        MAKE_FIELD(hw_max_a, hw_max_a, float, true)
        MAKE_FIELD(clamp_x, clamp_x, bool, false)
        MAKE_FIELD(clamp_v, clamp_v, bool, false)
        MAKE_FIELD(clamp_a, clamp_a, bool, false)
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
        0.0,  // [m] Absolute max cart position
        0.5,  // [m/s] Absolute max cart velocity
        1.0,  // [m/s^2] Absolute max cart acceleration
        0.0,  // [m] Absolute max hardware-allowed position
        10,  // [m/s] Absolute max hardware-allowed velocity [FIXME!]
        10,  // [m/s^2] Absolute max hardware-allowed acceleration [FIXME!]
        false,  // Clamp X to allowed range instead of raising error
        false,  // Clamp V to allowed range instead of raising error
        false,  // Clamp A to allowed range instead of raising error
        0.0,  // [m] Current cart position
        0.0,  // [m] Target cart position
        0.0,  // [m/s] Current cart velocity
        0.0,  // [m/s] Target cart velocity
        0.0,  // [m/s^2] Current cart acceleration
        0.0,  // [m/s^2] Target cart acceleration
        0.0,  // [rad] Current pole angle
        0.0,  // [rad/s] Current pole angular velocity
        Error::NEED_RESET,  // Current error code
        0.0,  // [m] Total Length as determined during homing
    };
    return globals;
}