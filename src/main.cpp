#include <Arduino.h>
#include "TMCStepper.h"
#include "FastAccelStepper.h"

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

/**** CONFIG/STATE DEFAULTS ****/
#define DEFAULT_SAFE_MARGIN 0.01
#define DEFAULT_MAX_V 0.2
#define DEFAULT_MAX_A 1.0
#define DEFAULT_CLAMP_X false
#define DEFAULT_CLAMP_V false
#define DEFAULT_CLAMP_A false
#define DEFAULT_STEPPER_CURRENT 1000

/**** INTERNAL GLOBALS ****/
int full_length = 0;
String input_buffer = "";
String output_buffer(255);

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

/**** PUBLIC STATE GLOBALS ****/
// TODO

/**** GLOBAL SINGLETONES ****/
TMC2209Stepper driver(&TMC_SERIAL_PORT, TMC_R_SENSE, TMC_ADDRESS);
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

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
  // stepper->setSpeedInHz(1000 * TMC_MICROSTEPS);  // TODO
  // stepper->setAcceleration(5000 * TMC_MICROSTEPS);  // TODO
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

void set_speed(float value) {
  uint32_t speed_hz = value * FULL_STEPS_PER_M * TMC_MICROSTEPS;
  stepper->setSpeedInHz(speed_hz);
}

void set_accel(float value) {
  uint32_t steps_per_ss = value * FULL_STEPS_PER_M * TMC_MICROSTEPS;
  stepper->setAcceleration(steps_per_ss);
}

String auto_home() {
  driver.toff(TMC_TOFF_VALUE);
  set_speed(HOMING_SPEED);
  set_accel(HOMING_ACCEL);
  
  // RUN LEFT
  stepper->runBackward();
  while (!digitalRead(PIN_ENDSTOP_LEFT)) {};
  stepper->setCurrentPosition(0);
  stepper->forceStopAndNewPosition(0);
  delay(100);
  
  // RUN RIGHT
  stepper->runForward();
  while (!digitalRead(PIN_ENDSTOP_RIGHT)) {};
  full_length = stepper->getCurrentPosition();
  stepper->forceStopAndNewPosition(full_length);
  delay(100);

  Serial.printf(
    "# Full length: %d steps, %.3f meters\n", 
    full_length, 
    1.0f * full_length / TMC_MICROSTEPS / FULL_STEPS_PER_M
  );

  // GOTO CENTER
  stepper->moveTo(full_length / 2);
  while (stepper->isRunning()) {};
  stepper->setCurrentPosition(0);
  
  return "OK";
}

String handle_command(String line) {
  char char_array[line.length() + 1];
  line.toCharArray(char_array, line.length() + 1);
  String cmd = strtok(char_array, " ");
  if (cmd == "reset") {
    return auto_home();
  } else {
    return "! Unknown command: " + cmd;
  }
}

void handle_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      String response = handle_command(input_buffer);
      Serial.println(response);
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

void loop() {
  handle_serial();
  handle_guards();
  handle_encoder();
}
