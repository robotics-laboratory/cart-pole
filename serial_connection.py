import logging
import serial

LOGGER = logging.getLogger(__name__)


class SerialConnection:
    PROCESSING_MESSAGE = 'echo:busy:processing'

    def __init__(self, 
        port: str = '/dev/ttyS0', 
        baud_rate: int = 115200, 
        read_timeout: float = 1.0, 
        write_timeout: float = 1.0
    ) -> None:
        self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=read_timeout, write_timeout=write_timeout, exclusive=True)
        LOGGER.debug(f'Opened serial connection to {self.serial.name}')

    def request(self, text: str):
        LOGGER.debug(f'Request to serial connection "{text}"')
        self.serial.write(text)

        while True:
            line = self.serial.readline().strip()
            
            if line == SerialConnection.PROCESSING_MESSAGE:
                LOGGER.debug(f'Received processing message during "{text}" request')
                continue
            if line.startswith('!'):
                LOGGER.error(f'Received error response during "{text}" request: {line}')
                raise ValueError(f'Received error response: {line}')

            LOGGER.debug(f'Responding to request "{text}" with "{line}"')
            return line
