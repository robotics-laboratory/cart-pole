from typing import Tuple

import crc
from betterproto import Message
from cobs import cobs

from cartpole.device import proto

FRAME_DELIMITER = b"\x00"
BYTEORDER = "little"
CRC_CALCULATOR = crc.Calculator(crc.Crc8.CCITT)


def encode(type: proto.RequestType, payload: Message = None) -> bytes:
    # Frame: [TYPE] [LEN+1] [DATA] [CRC] [EOF]
    payload_encoded = bytes(payload) if payload is not None else b""
    frame = bytearray()
    frame.extend(type.to_bytes(length=1, byteorder=BYTEORDER))
    frame.extend(len(payload_encoded).to_bytes(length=1, byteorder=BYTEORDER))
    frame.extend(payload_encoded)
    checksum = CRC_CALCULATOR.checksum(frame)
    frame.extend(checksum.to_bytes(length=1, byteorder=BYTEORDER))
    return cobs.encode(frame) + FRAME_DELIMITER


def decode(data: bytes, payload_type: Message = None) -> Tuple[proto.RequestType, Message]:
    # Frame: [TYPE] [LEN+1] [DATA] [CRC] [EOF]
    data = data.rstrip(FRAME_DELIMITER)
    data = cobs.decode(data)
    assert len(data) >= 3, f"Incorrect frame length ({len(data)}): {data}"
    checksum_ok = CRC_CALCULATOR.verify(data[:-1], data[-1])
    assert checksum_ok, "CRC checksum mismatch"
    type = proto.RequestType(data[0])
    payload_length = data[1]
    assert payload_length >= 0, "Incorrect payload length"
    payload = None
    if payload_type is not None:
        payload = payload_type.FromString(data[2 : 2 + payload_length])
    return type, payload


if __name__ == "__main__":
    # TODO: Move to tests
    request_type = proto.RequestType.TARGET
    request_payload = proto.Target(position=1, velocity=2, acceleration=3)
    data = encode(request_type, request_payload)
    print("Encoded frame:", data)
    response_type, response_payload = decode(data, proto.Target)
    assert request_type == response_type
    assert request_payload == response_payload
    print("All checks passed")   
