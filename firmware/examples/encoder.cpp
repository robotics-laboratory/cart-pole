#include <Arduino.h>
#include <Wire.h>

// https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdf
// https://github.com/kanestoboi/AS5600/blob/master/AS5600.cpp

#define DEBUG_LED_PIN 2
#define CYCLE_DELAY_MS 500
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_SPEED 1000000
#define DEBUG_UART_PORT Serial
#define DEBUG_UART_SPEED 115200
#define AS5600_ADDRESS 0x36
#define RAW_ANGLE_MSB_REG 0x0C
#define RAW_ANGLE_LSB_REG 0x0D
#define STATUS_REG 0x0B

TwoWire I2C_PORT(Wire);

uint8_t get_one_register(byte reg) {
    I2C_PORT.beginTransmission(AS5600_ADDRESS);
    I2C_PORT.write(reg);
    I2C_PORT.endTransmission();
    I2C_PORT.requestFrom(AS5600_ADDRESS, 1);
    while (I2C_PORT.available() == 0);
    return I2C_PORT.read();
}

uint16_t get_two_registers(byte regMSB, byte regLSB) {
    uint16_t _hi = get_one_register(regMSB);
    uint16_t _lo = get_one_register(regLSB);
    return (_hi << 8) | (_lo);
}

void setup() {
    pinMode(DEBUG_LED_PIN, OUTPUT);
    I2C_PORT.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_SPEED);
    DEBUG_UART_PORT.begin(DEBUG_UART_SPEED);

    int status = get_one_register(STATUS_REG);
    bool magnet_too_strong = status & (1 << 3);
    bool magnet_too_weak = status & (1 << 4);
    bool magnet_detected = status & (1 << 5);
    DEBUG_UART_PORT.printf("Magnet detected: %d\n", magnet_detected);
    DEBUG_UART_PORT.printf("Magnet too weak: %d\n", magnet_too_weak);
    DEBUG_UART_PORT.printf("Magnet too strong: %d\n", magnet_too_strong);
    delay(3000);
}

void loop() {
    int raw_angle = get_two_registers(RAW_ANGLE_MSB_REG, RAW_ANGLE_LSB_REG);
    DEBUG_UART_PORT.println(raw_angle);
    delay(50);
}
