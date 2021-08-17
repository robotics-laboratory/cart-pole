import logging
import serial
from typing import Union, Type

from cart_pole.common.interface import DeviceVariableGroup


LOGGER = logging.getLogger(__name__)
RAW_COMMANDS_LOGGER = logging.getLogger('raw_commands')


class WireInterface:
    def __init__(self, 
        port: str = '/dev/ttyS0', 
        baud_rate: int = 115200, 
        read_timeout: float = 1.0, 
        write_timeout: float = 1.0,
    ) -> None:
        self.serial = serial.Serial(port=port, baudrate=baud_rate, timeout=read_timeout, write_timeout=write_timeout, exclusive=True)
        LOGGER.debug(f'Opened serial connection to {self.serial.name}')

    def request(self, text: str = '') -> str:
        LOGGER.debug(f'Request to serial connection "{text}"')
        RAW_COMMANDS_LOGGER.debug(text)
        self.serial.write(text)

        while True:
            line = self.serial.readline().strip()
            RAW_COMMANDS_LOGGER.debug(text)
            
            if line.startswith('~'):
                LOGGER.debug(f'Received processing message during "{text}" request')
                continue

            stripped = line[1:].strip()
            if line.startswith('#'):
                LOGGER.debug(f'Received log message during "{text}" request: {stripped}')
                continue
            elif line.startswith('!'):
                LOGGER.error(f'Received error response during "{text}" request: {stripped}')
                raise RuntimeError(f'Received error response: {stripped}')
            elif line.startswith('+'):
                LOGGER.debug(f'Responding to request "{text}" with "{line}"')
                return stripped
            else:
                LOGGER.debug(f'Received unknown response line: {line}')
                raise RuntimeError(f'Received unknown response line: {line}')

    def _command(self, command: str, group: str = '', args: str = '') -> str:
        return self.serial_connection.request(f'{command} {group} {args}')

    def set(self, params: DeviceVariableGroup) -> DeviceVariableGroup:
        response = self._command('set', params.PROTOCOL_GROUP_NAME, params.to_dict_format())
        return params.from_dict_format(response)

    def get(self, params: Union[DeviceVariableGroup, Type[DeviceVariableGroup]]) -> DeviceVariableGroup:
        if isinstance(params, type):
            params = params.full()
        response = self._command('get', params.PROTOCOL_GROUP_NAME, params.to_list_format())
        return params.from_dict_format(response)

    def reset(self) -> None:
        _ = self._command('reset')
