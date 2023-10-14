#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

#include "CRC.h"
#include "CRC8.h"
#include "cobs.h"
#include "proto/protocol.pb.h"

const uint32_t SERIAL_SPEED = 1000000;
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
    // uint8_t type = rx_buff[1];
    uint8_t type = 2;
    // size_t payload_len = rx_buff[1];
    if (type == 0) {
        uint8_t tx_buff[256];
        tx_buff[0] = 0;                  // Reserved for COBS overhead byte
        tx_buff[1] = type;               // Request type
        tx_buff[2] = 0;                  // Data length
        crc.restart();
        crc.add(tx_buff + 1, 2);         
        tx_buff[3] = crc.getCRC();       // CRC8 for bytes 1-2
        tx_buff[4] = FRAME_DELIMITER;    // End of frame
        size_t tx_size = cobs::encode(tx_buff, 4);
        Serial.write(tx_buff, 5);
    } else if (type == 2) {
        uint8_t tx_buff[256];
        tx_buff[0] = 0;
        tx_buff[1] = type;
        tx_buff[2] = 0;

        // Data region starts at byte 3
        // 5 extra bytes reserved for COBS, TYPE, LEN, CRC8, EOF
        pb_ostream_t ostream = pb_ostream_from_buffer(tx_buff + 3, sizeof(tx_buff) - 5);
        State state = State_init_zero;
        state.cart_position = 123.0;
        state.has_cart_position = true;
        bool ok = pb_encode(&ostream, &State_msg, &state);

        // Serial.printf("size: %d ", ostream.bytes_written);
        // Serial.printf("ok: %d ", ok);
        // Serial.write(FRAME_DELIMITER);
        // return;

        tx_buff[2] = ostream.bytes_written;
        crc.restart();
        crc.add(tx_buff + 1, 2 + ostream.bytes_written);
        tx_buff[3 + ostream.bytes_written] = crc.getCRC();
        tx_buff[4] = FRAME_DELIMITER;
        size_t tx_size = cobs::encode(tx_buff, 4 + ostream.bytes_written);
        Serial.write(tx_buff, 5 + ostream.bytes_written);
    } else {
        Serial.print("error");
        Serial.write(FRAME_DELIMITER);
    }
}
