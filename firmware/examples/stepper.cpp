#include <Arduino.h>
#include <TMCStepper.h>

// https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2209_Datasheet_V103.pdf
// https://github.com/bigtreetech/BIGTREETECH-TMC2209-V1.2/blob/master/manual/TMC2209-V1.2-manual.pdf

#define DEBUG_LED_PIN 2
#define TMC_EN_PIN 25
#define TMC_STEP_PIN 33
#define TMC_DIR_PIN 32
#define TMC_UART_PORT Serial2
#define TMC_UART_SPEED 115200
#define TMC_ADDRESS 0b00
#define TMC_R_SENSE_OHMS 0.11f

#define MICROSTEPS 32
#define STEPS_PER_REV 200 * MICROSTEPS
#define TURN_COUNT 2
#define STEP_DELAY_US 100
#define CYCLE_DELAY_MS 3000

TMC2209Stepper TMC_DRIVER(&TMC_UART_PORT, TMC_R_SENSE_OHMS, TMC_ADDRESS);

void setup() {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    pinMode(TMC_EN_PIN, OUTPUT);
    pinMode(TMC_STEP_PIN, OUTPUT);
    pinMode(TMC_DIR_PIN, OUTPUT);

    TMC_UART_PORT.begin(TMC_UART_SPEED);
    TMC_DRIVER.begin();
    TMC_DRIVER.microsteps(MICROSTEPS);
    TMC_DRIVER.I_scale_analog(0);
    TMC_DRIVER.vsense(0);
    TMC_DRIVER.irun(20);
    TMC_DRIVER.ihold(20);

    digitalWrite(TMC_EN_PIN, false);
}

bool dir = false;

void loop() {
    digitalWrite(DEBUG_LED_PIN, false);
    for (int i = 0; i < STEPS_PER_REV * TURN_COUNT; ++i) {
        digitalWrite(TMC_STEP_PIN, true);
        delayMicroseconds(STEP_DELAY_US);
        digitalWrite(TMC_STEP_PIN, false);
        delayMicroseconds(STEP_DELAY_US);
    }
    digitalWrite(DEBUG_LED_PIN, true);
    delay(CYCLE_DELAY_MS);
    digitalWrite(TMC_DIR_PIN, dir = !dir);
}
