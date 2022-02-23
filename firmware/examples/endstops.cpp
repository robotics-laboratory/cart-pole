#include <Arduino.h>

#define L_ENDSTOP_PIN 34
#define R_ENDSTOP_PIN 35
#define DEBUG_UART_PORT Serial
#define DEBUG_UART_SPEED 115200

void setup() {
    pinMode(L_ENDSTOP_PIN, INPUT);
    pinMode(R_ENDSTOP_PIN, INPUT);
    DEBUG_UART_PORT.begin(DEBUG_UART_SPEED);
}

void loop() {
    bool l_endstop = digitalRead(L_ENDSTOP_PIN);
    bool r_endstop = digitalRead(R_ENDSTOP_PIN);
    DEBUG_UART_PORT.printf("LEFT: %d | RIGHT: %d\n", l_endstop, r_endstop);
    delay(50);
}
