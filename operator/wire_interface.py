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

    def close(self):
        self.serial.close()

    def request(self, command: str = '') -> str:
        command = command.strip()

        LOGGER.debug(f'Request to serial connection "{command}"')
        RAW_COMMANDS_LOGGER.debug(command)
        self.serial.write((command + '\n').encode('utf-8'))

        while True:
            received = self.serial.readline().decode('utf-8').strip()
            if received == '':
                LOGGER.warning(f'Serial read timeout during "{command}" request')
                continue
        
            RAW_COMMANDS_LOGGER.debug(received)

            if received.startswith('~'):
                LOGGER.debug(f'Received processing message during "{command}" request')
                continue

            stripped = received[1:].strip()
            if received.startswith('#'):
                LOGGER.debug(f'Received log message during "{command}" request: {stripped}')
                continue
            elif received.startswith('!'):
                LOGGER.error(f'Received error response during "{command}" request: {stripped}')
                raise RuntimeError(f'Received error response: {stripped}')
            elif received.startswith('+'):
                LOGGER.debug(f'Responding to request "{command}" with "{received}"')
                return stripped
            else:
                LOGGER.debug(f'Received unknown response line: {received}')

    def _command(self, command: str, group: str = '', args: str = '') -> str:
        return self.request(f'{command} {group} {args}')

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
