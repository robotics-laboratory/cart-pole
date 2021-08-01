#include <Arduino.h>
#include "FastAccelStepper.h"
#undef min
#undef max
#include "TMCStepper.h"
#include <string>
#include <set>
#include <tuple>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std;

/**** PIN CONNECTIONS ****/
#define PIN_TMC_EN 25
#define PIN_TMC_STEP 26
#define PIN_TMC_DIR 27
#define PIN_ENDSTOP_LEFT 18
#define PIN_ENDSTOP_RIGHT 19

/**** STEPPER DRIVER CONFIG ****/
#define TMC_SERIAL_PORT Serial2
#define TMC_ADDRESS 0b00
#define TMC_R_SENSE 0.11f
#define TMC_TOFF_VALUE 5
#define TMC_MICROSTEPS 16

/**** MISCELLANEOUS ****/
#define REVERSE_STEPPER true
#define FULL_STEPS_PER_M 5000    // How many *full* steps to move by 1 meter
#define SERIAL_SPEED_CMD 115200
#define SERIAL_SPEED_TMC 115200
#define HOMING_SPEED 0.1
#define HOMING_ACCEL 0.5
#define ERR_MSG_KEY_IS_READONLY "! This key is readonly: "

/**** CONFIG/STATE DEFAULTS ****/
#define DEFAULT_SAFE_MARGIN 0.01
#define DEFAULT_MAX_V 0.2
#define DEFAULT_MAX_A 1.0
#define DEFAULT_CLAMP_X false
#define DEFAULT_CLAMP_V false
#define DEFAULT_CLAMP_A false
#define DEFAULT_STEPPER_CURRENT 1500

/**** INTERNAL GLOBALS ****/
float full_length = 0;
string input_buffer = "";
char output_buffer[255];

/**** PUBLIC CONFIG GLOBALS ****/
float max_x = 0;
float max_v = DEFAULT_MAX_V;
float max_a = DEFAULT_MAX_A;
float safe_margin = DEFAULT_SAFE_MARGIN;
const float hw_max_v = 1000;
const float hw_max_a = 1000;
bool clamp_x = DEFAULT_CLAMP_X;
bool clamp_v = DEFAULT_CLAMP_V;
bool clamp_a = DEFAULT_CLAMP_A;
int stepper_current = DEFAULT_STEPPER_CURRENT;

const auto config_keys = vector<string> {
  "max_x",
  "max_v",
  "max_a",
  "safe_margin",
  "hw_max_v",
  "hw_max_a",
  "clamp_x",
  "clamp_v",
  "clamp_a",
  "stepper_current",
  "trgt_x", // TODO: REMOVE!!! ONLY FOR TESTING
};

/**** ERROR CODES ****/
enum class ERRCODE: int {
  E00_NO_ERROR = 0,
  E01_NEED_INIT = 1,
  E02_X_OVERFLOW = 2,
  E03_V_OVERFLOW = 3,
  E04_A_OVERFLOW = 4
};

/**** PUBLIC STATE GLOBALS ****/
float curr_x = 0;
float trgt_x = 0;
float curr_v = 0;
float trgt_v = 0;
float curr_a = 0;
float trgt_a = 0;
float pole_ang = 0;
float pole_vel = 0;
float timestamp = 0;
ERRCODE errcode = ERRCODE::E01_NEED_INIT;

const auto state_keys = vector<string> {
  "curr_x",
  "trgt_x",
  "curr_v",
  "trgt_v",
  "curr_a",
  "trgt_a",
  "pole_ang",
  "pole_vel",
  "timestamp",
  "errcode",
};

/**** GLOBAL SINGLETONES ****/
TMC2209Stepper driver(&TMC_SERIAL_PORT, TMC_R_SENSE, TMC_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

/**** INITIALIZATION ****/

void init_pins() {
  pinMode(PIN_TMC_EN, OUTPUT);
  pinMode(PIN_TMC_STEP, OUTPUT);
  pinMode(PIN_TMC_DIR, OUTPUT);
  pinMode(PIN_ENDSTOP_LEFT, INPUT);
  pinMode(PIN_ENDSTOP_RIGHT, INPUT);
}

void init_tmc() {
  digitalWrite(PIN_TMC_EN, LOW);
  delay(100);  // Small warmup delay
  TMC_SERIAL_PORT.begin(SERIAL_SPEED_TMC);
  driver.begin();
  driver.rms_current(DEFAULT_STEPPER_CURRENT);
  driver.microsteps(TMC_MICROSTEPS);
  driver.toff(0);
}

void init_fas() {
  engine.init();
  stepper = engine.stepperConnectToPin(PIN_TMC_STEP);
  stepper->setDirectionPin(PIN_TMC_DIR, REVERSE_STEPPER);
}

void init_encoder() {
  // TODO
}

void setup() {
  Serial.begin(SERIAL_SPEED_CMD);
  while (!Serial) {};
  init_pins();
  init_tmc();
  init_fas();
  init_encoder();
  Serial.println("# CARTPOLE CONTROLLER");
}

/**** STEPPER CONTROL ****/

void stepper_set_speed(float value) {
  uint32_t speed_hz = value * FULL_STEPS_PER_M * TMC_MICROSTEPS;
  Serial.printf("# Set stepper speed to %d steps/s\n", speed_hz);
  stepper->setSpeedInHz(speed_hz);
}

void stepper_set_accel(float value) {
  uint32_t steps_per_ss = value * FULL_STEPS_PER_M * TMC_MICROSTEPS;
  Serial.printf("# Set stepper accel to %d steps/s^2\n", steps_per_ss);
  stepper->setAcceleration(steps_per_ss);
}

void stepper_enable() {
  Serial.println("# Stepper enabled");
  driver.toff(TMC_TOFF_VALUE);
}

void stepper_disable() {
  Serial.println("# Stepper disabled");
  driver.toff(0);
}

void stepper_force_stop() {
  Serial.println("# Stepper force stop");
  stepper->forceStopAndNewPosition(stepper->getCurrentPosition());
  delay(100);
}

void stepper_goto_pos(float pos) {
  uint32_t pos_steps = (pos + full_length / 2) * FULL_STEPS_PER_M * TMC_MICROSTEPS;
  Serial.printf("# New target: %.3f (%d steps)", pos, pos_steps);
  stepper->moveTo(pos_steps);
}

/**** INPUT PARSING/FORMATTING HELPERS ****/

string format_float(float value, int precision = 5) {
  char tmp[255];
  sprintf(tmp, "%.*f", precision, value);
  return tmp;
}

string format_bool(bool value) {
  return value ? "true" : "false";
}

string format_int(int value) {
  char tmp[255];
  sprintf(tmp, "%d", value);
  return tmp;
}

string format_param(string key) {
  if (key == "max_x") { return format_float(max_x); }
  else if (key == "max_v") { return format_float(max_v); }
  else if (key == "max_a") { return format_float(max_a); }
  else if (key == "safe_margin") { return format_float(safe_margin); }
  else if (key == "hw_max_v") { return format_float(hw_max_v); }
  else if (key == "hw_max_a") { return format_float(hw_max_a); }
  else if (key == "clamp_x") { return format_bool(clamp_x); }
  else if (key == "clamp_v") { return format_bool(clamp_v); }
  else if (key == "clamp_a") { return format_bool(clamp_a); }
  else if (key == "stepper_current") { return format_int(stepper_current); }
  else if (key == "curr_x") { return format_float(curr_x); }
  else if (key == "trgt_x") { return format_float(trgt_x); }
  else if (key == "curr_v") { return format_float(curr_v); }
  else if (key == "trgt_v") { return format_float(trgt_v); }
  else if (key == "curr_a") { return format_float(curr_a); }
  else if (key == "trgt_a") { return format_float(trgt_a); }
  else if (key == "pole_ang") { return format_float(pole_ang); }
  else if (key == "pole_vel") { return format_float(pole_vel); }
  else if (key == "timestamp") { return format_float(timestamp); }
  else if (key == "errcode") { return format_int(static_cast<int>(errcode)); }
  else { throw runtime_error("! Unknown key passed to format_value"); }
}

tuple<bool, float, string> parse_float(string input, float min, float max, bool allow_nan = false) {
  float value;
  int retcode = sscanf(input.c_str(), "%f", &value);
  if (retcode != 1 || !isfinite(value)) return make_tuple(false, 0, "! Failed to parse float: " + input);
  if (allow_nan && !isnan(value)) return make_tuple(false, 0, "! NaN not allowed: " + input);
  if (value < min) return make_tuple(false, 0, "! Validation failed: " + format_float(value) + " < " + format_float(min));
  if (value > max) return make_tuple(false, 0, "! Validation failed: " + format_float(value) + " > " + format_float(max));
  return make_tuple(true, value, "");
}

string parse_param(string key, string input) {
  bool ok;
  string msg;
  float f_value;
  if (key == "max_x") { return "! This key is readonly: " + key; }
  else if (key == "max_v") {
    tie(ok, f_value, msg) = parse_float(input, 0, hw_max_v);
    if (!ok) return msg;
    max_v = f_value;
    stepper_set_speed(max_v);
  } else if (key == "max_a") {
    tie(ok, f_value, msg) = parse_float(input, 0, hw_max_a);
    if (!ok) return msg;
    max_a = f_value;
    stepper_set_accel(max_a);
  } else if (key == "safe_margin") {
    tie(ok, f_value, msg) = parse_float(input, 0, 1);
    if (!ok) return msg;
    safe_margin = f_value;
    errcode = ERRCODE::E01_NEED_INIT; // Maybe go to center, then update max_x, then reset state
  } else if (key == "hw_max_v") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "hw_max_a") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "clamp_x") { return "! NOT IMPLEMENTED"; }
  else if (key == "clamp_v") { return "! NOT IMPLEMENTED"; }
  else if (key == "clamp_a") { return "! NOT IMPLEMENTED"; }
  else if (key == "stepper_current") { return "! NOT IMPLEMENTED"; }
  else if (key == "curr_x") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "trgt_x") {
    tie(ok, f_value, msg) = parse_float(input, -max_x, max_x);
    if (!ok) return msg;
    trgt_x = f_value;
    stepper_goto_pos(trgt_x);
  } else if (key == "curr_v") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "trgt_v") { return "! NOT IMPLEMENTED"; }
  else if (key == "curr_a") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "trgt_a") { return "! NOT IMPLEMENTED"; }
  else if (key == "pole_ang") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "pole_vel") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "timestamp") { return ERR_MSG_KEY_IS_READONLY + key; }
  else if (key == "errcode") { return ERR_MSG_KEY_IS_READONLY + key; }
  else { throw runtime_error("! Unknown key passed to parse_param"); }
  return "";
}

tuple<bool, vector<pair<string, string>>, string>
parse_kv_pairs(string input, vector<string> allowed_keys) {
  istringstream ss{input};
  string token;
  vector<pair<string, string>> pairs;
  while (getline(ss, token, ' ')) {
    if (token.empty()) continue;
    int split = token.find('=');
    if (split == string::npos) {
      return make_tuple(false, pairs, "! Failed to parse key=value pair: " + token);
    }
    string key = token.substr(0, split);
    string value = token.substr(split + 1, string::npos);
    if (find(allowed_keys.begin(), allowed_keys.end(), key) == allowed_keys.end()) {
      return make_tuple(false, pairs, "! No such key: " + key);
    }
    pairs.push_back(make_pair(key, value));
  }
  return make_tuple(true, pairs, "");
}

tuple<bool, vector<string>, string>
parse_keys(string input, vector<string> allowed_keys) {
  istringstream ss{input};
  string token;
  vector<string> keys;
  while (getline(ss, token, ' ')) {
    if (token.empty()) continue;
    if (find(allowed_keys.begin(), allowed_keys.end(), token) == allowed_keys.end()) {
      return make_tuple(false, keys, "! No such key: " + token);
    }
    keys.push_back(token);
  }
  return make_tuple(true, keys, "");
}

string format_kv_pairs(vector<string> keys) {
  ostringstream ss;
  string sep = "";
  for (auto&& key : keys) {
    ss << sep;
    sep = " ";
    ss << key << "=" << format_param(key);
  }
  return ss.str();
}

/**** COMMANDS IMPLEMENTATION ****/

string cmd_config_get(string args) {
  if (args == "") {
    return format_kv_pairs(config_keys);  
  } else {
    bool ok; vector<string> keys; string msg;
    tie(ok, keys, msg) = parse_keys(args, config_keys);
    if (!ok) return msg;
    return format_kv_pairs(keys);
  }
}

string cmd_config_set(string args) {
  bool ok; vector<pair<string, string>> kv_pairs; string msg;
  tie(ok, kv_pairs, msg) = parse_kv_pairs(args, config_keys);
  if (!ok) return msg;
  vector<string> keys;
  for (auto&& pair : kv_pairs) {
    msg = parse_param(pair.first, pair.second);
    if (!msg.empty()) return msg;
    keys.push_back(pair.first);
  }
  return format_kv_pairs(keys);
}

string cmd_reset(string args) {
  if (!args.empty()) return "! Reset takes no arguments";
  stepper_force_stop();
  stepper_enable();
  stepper_set_speed(HOMING_SPEED);
  stepper_set_accel(HOMING_ACCEL);
  
  // RUN LEFT
  stepper->runBackward();
  while (!digitalRead(PIN_ENDSTOP_LEFT)) {};
  stepper_force_stop();
  stepper->setCurrentPosition(0);
  
  // RUN RIGHT
  stepper->runForward();
  while (!digitalRead(PIN_ENDSTOP_RIGHT)) {};
  stepper_force_stop();
  int delta_steps = stepper->getCurrentPosition();
  stepper->setCurrentPosition(delta_steps);

  // GOTO CENTER
  stepper->moveTo(delta_steps / 2);
  while (stepper->isRunning()) {};

  // UPDATE CONFIG
  full_length = 1.0f * delta_steps / TMC_MICROSTEPS / FULL_STEPS_PER_M;
  max_x = full_length / 2 - safe_margin;
  // TODO: Reset state?
  errcode = ERRCODE::E00_NO_ERROR;
  Serial.printf("# Full length: %d steps, %.3f meters\n", delta_steps, full_length);
  Serial.printf("# Valid X range: %.3f ... %.3f\n", -max_x, max_x);
  
  return "OK";
}

string handle_command(string line) {
  int split = line.find(' ');
  string cmd, args;
  if (split == string::npos) {  // TODO: Helper function
    cmd = line;
    args = "";
  } else {
    cmd = line.substr(0, split);
    args = line.substr(split + 1, string::npos); 
  }
  if (cmd == "reset") {
    return cmd_reset(args);
  } else if (cmd == "config-get") {  // TODO: Split one more time
    return cmd_config_get(args);
  } else if (cmd == "config-set") {
    return cmd_config_set(args);
  // } else if (errcode != ERRCODE::E00_NO_ERROR) {
  //   sprintf(output_buffer, "! Errcode: %d", static_cast<int>(errcode));
  //   return output_buffer;
  } else {
    return "! Unknown command: " + cmd;
  }
}

/**** MAIN HANDLERS ****/

void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      string response = handle_command(input_buffer);
      Serial.println(response.c_str());
      input_buffer = "";
      break;
    } else {
      input_buffer += c;
    }
  };
}

void handle_guards() {
  // TODO
}

void handle_encoder() {
  // TODO
}

/**** MAIN LOOP ****/

void loop() {
  handle_serial();
  handle_guards();
  handle_encoder();
}
