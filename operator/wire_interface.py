from typing import Union, Type

from cart_pole.common.interface import DeviceVariableGroup
from cart_pole.operator.serial_connection import SerialConnection


class WireInterface:
    def __init__(self, serial_connection: SerialConnection) -> None:
        self.serial_connection = serial_connection

    def _request(self, command: str, group: str = '', args: str = '') -> str:
        return self.serial_connection.request(f'{command} {group} {args}')

    def set(self, params: DeviceVariableGroup) -> DeviceVariableGroup:
        response = self._request('set', params.PROTOCOL_GROUP_NAME, params.to_dict_format())
        return params.from_dict_format(response)

    def get(self, params: Union[DeviceVariableGroup, Type[DeviceVariableGroup]]) -> DeviceVariableGroup:
        if isinstance(params, type):
            params = params.full()
        response = self._request('get', params.PROTOCOL_GROUP_NAME, params.to_list_format())
        return params.from_dict_format(response)

    def reset(self) -> None:
        _ = self._request('reset')
