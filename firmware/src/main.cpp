#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "CRC.h"
#include "CRC8.h"
#include "cobs.h"
#include "proto/protocol.pb.h"

const uint32_t SERIAL_SPEED = 3000000;
const uint8_t FRAME_DELIMITER = 0x00;

uint8_t rx_buff[256];
CRC8 crc;

void setup() {
    Serial.begin(SERIAL_SPEED);
    Serial.print("start");
}

void loop() {
    size_t length = Serial.readBytesUntil(FRAME_DELIMITER, rx_buff, sizeof(rx_buff));
    if (!length) return;
    cobs::decode(rx_buff, length);
    RequestType type = static_cast<RequestType>(rx_buff[0]);
    size_t payload_len = rx_buff[1];
    if (type == RequestType_RESET) {
        uint8_t tx_buff[256];
        tx_buff[0] = 0;                  // Reserved for COBS overhead byte
        tx_buff[1] = RequestType_RESET;  // Request type
        tx_buff[2] = 0;                  // Data length
        crc.restart();
        crc.add(tx_buff + 1, 2);         // CRC8 for bytes 1-2
        tx_buff[3] = crc.getCRC();
        tx_buff[4] = FRAME_DELIMITER;    // End of frame
        size_t tx_size = cobs::encode(tx_buff, 4);
        Serial.write(tx_buff, 5);
    } else {
        Serial.print("error");
        Serial.write(FRAME_DELIMITER);
    }
}
