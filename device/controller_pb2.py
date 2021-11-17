# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: controller.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import device.nanopb_pb2 as nanopb__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='controller.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x10\x63ontroller.proto\x1a\x0cnanopb.proto\"\x7f\n\x07Request\x12\x1a\n\x04type\x18\x01 \x01(\x0e\x32\x0c.RequestType\x12\x19\n\x06\x63onfig\x18\x02 \x01(\x0b\x32\x07.ConfigH\x00\x12\x17\n\x05state\x18\x03 \x01(\x0b\x32\x06.StateH\x00\x12\x19\n\x06target\x18\x04 \x01(\x0b\x32\x07.TargetH\x00\x42\t\n\x07payload\"\x9e\x01\n\x08Response\x12\x1f\n\x06status\x18\x01 \x01(\x0e\x32\x0f.ResponseStatus\x12\x17\n\x07message\x18\x02 \x01(\tB\x06\x92?\x03\x08\x80\x01\x12\x19\n\x06\x63onfig\x18\x03 \x01(\x0b\x32\x07.ConfigH\x00\x12\x17\n\x05state\x18\x04 \x01(\x0b\x32\x06.StateH\x00\x12\x19\n\x06target\x18\x05 \x01(\x0b\x32\x07.TargetH\x00\x42\t\n\x07payload\"\xda\x02\n\x06\x43onfig\x12\x12\n\x05max_x\x18\x01 \x01(\x02H\x00\x88\x01\x01\x12\x12\n\x05max_v\x18\x02 \x01(\x02H\x01\x88\x01\x01\x12\x12\n\x05max_a\x18\x03 \x01(\x02H\x02\x88\x01\x01\x12\x15\n\x08hw_max_x\x18\x04 \x01(\x02H\x03\x88\x01\x01\x12\x15\n\x08hw_max_v\x18\x05 \x01(\x02H\x04\x88\x01\x01\x12\x15\n\x08hw_max_a\x18\x06 \x01(\x02H\x05\x88\x01\x01\x12\x14\n\x07\x63lamp_x\x18\x07 \x01(\x08H\x06\x88\x01\x01\x12\x14\n\x07\x63lamp_v\x18\x08 \x01(\x08H\x07\x88\x01\x01\x12\x14\n\x07\x63lamp_a\x18\t \x01(\x08H\x08\x88\x01\x01\x12\x16\n\tdebug_led\x18\n \x01(\x08H\t\x88\x01\x01\x42\x08\n\x06_max_xB\x08\n\x06_max_vB\x08\n\x06_max_aB\x0b\n\t_hw_max_xB\x0b\n\t_hw_max_vB\x0b\n\t_hw_max_aB\n\n\x08_clamp_xB\n\n\x08_clamp_vB\n\n\x08_clamp_aB\x0c\n\n_debug_led\"\xab\x02\n\x05State\x12\x13\n\x06\x63urr_x\x18\x01 \x01(\x02H\x00\x88\x01\x01\x12\x13\n\x06\x63urr_v\x18\x02 \x01(\x02H\x01\x88\x01\x01\x12\x13\n\x06\x63urr_a\x18\x03 \x01(\x02H\x02\x88\x01\x01\x12\x13\n\x06pole_x\x18\x04 \x01(\x02H\x03\x88\x01\x01\x12\x13\n\x06pole_v\x18\x05 \x01(\x02H\x04\x88\x01\x01\x12\x14\n\x07\x65rrcode\x18\x06 \x01(\x05H\x05\x88\x01\x01\x12\x12\n\x05imu_a\x18\x07 \x01(\x02H\x06\x88\x01\x01\x12\x14\n\x07motor_x\x18\x08 \x01(\x02H\x07\x88\x01\x01\x12\x14\n\x07motor_v\x18\t \x01(\x02H\x08\x88\x01\x01\x42\t\n\x07_curr_xB\t\n\x07_curr_vB\t\n\x07_curr_aB\t\n\x07_pole_xB\t\n\x07_pole_vB\n\n\x08_errcodeB\x08\n\x06_imu_aB\n\n\x08_motor_xB\n\n\x08_motor_v\"h\n\x06Target\x12\x13\n\x06trgt_x\x18\x01 \x01(\x02H\x00\x88\x01\x01\x12\x13\n\x06trgt_v\x18\x02 \x01(\x02H\x01\x88\x01\x01\x12\x13\n\x06trgt_a\x18\x03 \x01(\x02H\x02\x88\x01\x01\x42\t\n\x07_trgt_xB\t\n\x07_trgt_vB\t\n\x07_trgt_a*g\n\x0bRequestType\x12\r\n\tGET_STATE\x10\x00\x12\x0e\n\nSET_TARGET\x10\x01\x12\x0e\n\nSET_CONFIG\x10\x02\x12\x0e\n\nGET_TARGET\x10\x03\x12\x0e\n\nGET_CONFIG\x10\x04\x12\t\n\x05RESET\x10\x05*>\n\x0eResponseStatus\x12\x06\n\x02OK\x10\x00\x12\t\n\x05\x45RROR\x10\x01\x12\x0e\n\nPROCESSING\x10\x02\x12\t\n\x05\x44\x45\x42UG\x10\x03\x62\x06proto3'
  ,
  dependencies=[nanopb__pb2.DESCRIPTOR,])

_REQUESTTYPE = _descriptor.EnumDescriptor(
  name='RequestType',
  full_name='RequestType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='GET_STATE', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SET_TARGET', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SET_CONFIG', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GET_TARGET', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='GET_CONFIG', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RESET', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1081,
  serialized_end=1184,
)
_sym_db.RegisterEnumDescriptor(_REQUESTTYPE)

RequestType = enum_type_wrapper.EnumTypeWrapper(_REQUESTTYPE)
_RESPONSESTATUS = _descriptor.EnumDescriptor(
  name='ResponseStatus',
  full_name='ResponseStatus',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OK', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PROCESSING', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DEBUG', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1186,
  serialized_end=1248,
)
_sym_db.RegisterEnumDescriptor(_RESPONSESTATUS)

ResponseStatus = enum_type_wrapper.EnumTypeWrapper(_RESPONSESTATUS)
GET_STATE = 0
SET_TARGET = 1
SET_CONFIG = 2
GET_TARGET = 3
GET_CONFIG = 4
RESET = 5
OK = 0
ERROR = 1
PROCESSING = 2
DEBUG = 3



_REQUEST = _descriptor.Descriptor(
  name='Request',
  full_name='Request',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='Request.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='config', full_name='Request.config', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='state', full_name='Request.state', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='target', full_name='Request.target', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='payload', full_name='Request.payload',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=34,
  serialized_end=161,
)


_RESPONSE = _descriptor.Descriptor(
  name='Response',
  full_name='Response',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='Response.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='message', full_name='Response.message', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=b'\222?\003\010\200\001', file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='config', full_name='Response.config', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='state', full_name='Response.state', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='target', full_name='Response.target', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='payload', full_name='Response.payload',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=164,
  serialized_end=322,
)


_CONFIG = _descriptor.Descriptor(
  name='Config',
  full_name='Config',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='max_x', full_name='Config.max_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_v', full_name='Config.max_v', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='max_a', full_name='Config.max_a', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hw_max_x', full_name='Config.hw_max_x', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hw_max_v', full_name='Config.hw_max_v', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hw_max_a', full_name='Config.hw_max_a', index=5,
      number=6, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='clamp_x', full_name='Config.clamp_x', index=6,
      number=7, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='clamp_v', full_name='Config.clamp_v', index=7,
      number=8, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='clamp_a', full_name='Config.clamp_a', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='debug_led', full_name='Config.debug_led', index=9,
      number=10, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='_max_x', full_name='Config._max_x',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_max_v', full_name='Config._max_v',
      index=1, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_max_a', full_name='Config._max_a',
      index=2, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_hw_max_x', full_name='Config._hw_max_x',
      index=3, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_hw_max_v', full_name='Config._hw_max_v',
      index=4, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_hw_max_a', full_name='Config._hw_max_a',
      index=5, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_clamp_x', full_name='Config._clamp_x',
      index=6, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_clamp_v', full_name='Config._clamp_v',
      index=7, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_clamp_a', full_name='Config._clamp_a',
      index=8, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_debug_led', full_name='Config._debug_led',
      index=9, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=325,
  serialized_end=671,
)


_STATE = _descriptor.Descriptor(
  name='State',
  full_name='State',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='curr_x', full_name='State.curr_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='curr_v', full_name='State.curr_v', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='curr_a', full_name='State.curr_a', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pole_x', full_name='State.pole_x', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='pole_v', full_name='State.pole_v', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='errcode', full_name='State.errcode', index=5,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='imu_a', full_name='State.imu_a', index=6,
      number=7, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='motor_x', full_name='State.motor_x', index=7,
      number=8, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='motor_v', full_name='State.motor_v', index=8,
      number=9, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='_curr_x', full_name='State._curr_x',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_curr_v', full_name='State._curr_v',
      index=1, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_curr_a', full_name='State._curr_a',
      index=2, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_pole_x', full_name='State._pole_x',
      index=3, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_pole_v', full_name='State._pole_v',
      index=4, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_errcode', full_name='State._errcode',
      index=5, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_imu_a', full_name='State._imu_a',
      index=6, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_motor_x', full_name='State._motor_x',
      index=7, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_motor_v', full_name='State._motor_v',
      index=8, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=674,
  serialized_end=973,
)


_TARGET = _descriptor.Descriptor(
  name='Target',
  full_name='Target',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='trgt_x', full_name='Target.trgt_x', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trgt_v', full_name='Target.trgt_v', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='trgt_a', full_name='Target.trgt_a', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
    _descriptor.OneofDescriptor(
      name='_trgt_x', full_name='Target._trgt_x',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_trgt_v', full_name='Target._trgt_v',
      index=1, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_trgt_a', full_name='Target._trgt_a',
      index=2, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=975,
  serialized_end=1079,
)

_REQUEST.fields_by_name['type'].enum_type = _REQUESTTYPE
_REQUEST.fields_by_name['config'].message_type = _CONFIG
_REQUEST.fields_by_name['state'].message_type = _STATE
_REQUEST.fields_by_name['target'].message_type = _TARGET
_REQUEST.oneofs_by_name['payload'].fields.append(
  _REQUEST.fields_by_name['config'])
_REQUEST.fields_by_name['config'].containing_oneof = _REQUEST.oneofs_by_name['payload']
_REQUEST.oneofs_by_name['payload'].fields.append(
  _REQUEST.fields_by_name['state'])
_REQUEST.fields_by_name['state'].containing_oneof = _REQUEST.oneofs_by_name['payload']
_REQUEST.oneofs_by_name['payload'].fields.append(
  _REQUEST.fields_by_name['target'])
_REQUEST.fields_by_name['target'].containing_oneof = _REQUEST.oneofs_by_name['payload']
_RESPONSE.fields_by_name['status'].enum_type = _RESPONSESTATUS
_RESPONSE.fields_by_name['config'].message_type = _CONFIG
_RESPONSE.fields_by_name['state'].message_type = _STATE
_RESPONSE.fields_by_name['target'].message_type = _TARGET
_RESPONSE.oneofs_by_name['payload'].fields.append(
  _RESPONSE.fields_by_name['config'])
_RESPONSE.fields_by_name['config'].containing_oneof = _RESPONSE.oneofs_by_name['payload']
_RESPONSE.oneofs_by_name['payload'].fields.append(
  _RESPONSE.fields_by_name['state'])
_RESPONSE.fields_by_name['state'].containing_oneof = _RESPONSE.oneofs_by_name['payload']
_RESPONSE.oneofs_by_name['payload'].fields.append(
  _RESPONSE.fields_by_name['target'])
_RESPONSE.fields_by_name['target'].containing_oneof = _RESPONSE.oneofs_by_name['payload']
_CONFIG.oneofs_by_name['_max_x'].fields.append(
  _CONFIG.fields_by_name['max_x'])
_CONFIG.fields_by_name['max_x'].containing_oneof = _CONFIG.oneofs_by_name['_max_x']
_CONFIG.oneofs_by_name['_max_v'].fields.append(
  _CONFIG.fields_by_name['max_v'])
_CONFIG.fields_by_name['max_v'].containing_oneof = _CONFIG.oneofs_by_name['_max_v']
_CONFIG.oneofs_by_name['_max_a'].fields.append(
  _CONFIG.fields_by_name['max_a'])
_CONFIG.fields_by_name['max_a'].containing_oneof = _CONFIG.oneofs_by_name['_max_a']
_CONFIG.oneofs_by_name['_hw_max_x'].fields.append(
  _CONFIG.fields_by_name['hw_max_x'])
_CONFIG.fields_by_name['hw_max_x'].containing_oneof = _CONFIG.oneofs_by_name['_hw_max_x']
_CONFIG.oneofs_by_name['_hw_max_v'].fields.append(
  _CONFIG.fields_by_name['hw_max_v'])
_CONFIG.fields_by_name['hw_max_v'].containing_oneof = _CONFIG.oneofs_by_name['_hw_max_v']
_CONFIG.oneofs_by_name['_hw_max_a'].fields.append(
  _CONFIG.fields_by_name['hw_max_a'])
_CONFIG.fields_by_name['hw_max_a'].containing_oneof = _CONFIG.oneofs_by_name['_hw_max_a']
_CONFIG.oneofs_by_name['_clamp_x'].fields.append(
  _CONFIG.fields_by_name['clamp_x'])
_CONFIG.fields_by_name['clamp_x'].containing_oneof = _CONFIG.oneofs_by_name['_clamp_x']
_CONFIG.oneofs_by_name['_clamp_v'].fields.append(
  _CONFIG.fields_by_name['clamp_v'])
_CONFIG.fields_by_name['clamp_v'].containing_oneof = _CONFIG.oneofs_by_name['_clamp_v']
_CONFIG.oneofs_by_name['_clamp_a'].fields.append(
  _CONFIG.fields_by_name['clamp_a'])
_CONFIG.fields_by_name['clamp_a'].containing_oneof = _CONFIG.oneofs_by_name['_clamp_a']
_CONFIG.oneofs_by_name['_debug_led'].fields.append(
  _CONFIG.fields_by_name['debug_led'])
_CONFIG.fields_by_name['debug_led'].containing_oneof = _CONFIG.oneofs_by_name['_debug_led']
_STATE.oneofs_by_name['_curr_x'].fields.append(
  _STATE.fields_by_name['curr_x'])
_STATE.fields_by_name['curr_x'].containing_oneof = _STATE.oneofs_by_name['_curr_x']
_STATE.oneofs_by_name['_curr_v'].fields.append(
  _STATE.fields_by_name['curr_v'])
_STATE.fields_by_name['curr_v'].containing_oneof = _STATE.oneofs_by_name['_curr_v']
_STATE.oneofs_by_name['_curr_a'].fields.append(
  _STATE.fields_by_name['curr_a'])
_STATE.fields_by_name['curr_a'].containing_oneof = _STATE.oneofs_by_name['_curr_a']
_STATE.oneofs_by_name['_pole_x'].fields.append(
  _STATE.fields_by_name['pole_x'])
_STATE.fields_by_name['pole_x'].containing_oneof = _STATE.oneofs_by_name['_pole_x']
_STATE.oneofs_by_name['_pole_v'].fields.append(
  _STATE.fields_by_name['pole_v'])
_STATE.fields_by_name['pole_v'].containing_oneof = _STATE.oneofs_by_name['_pole_v']
_STATE.oneofs_by_name['_errcode'].fields.append(
  _STATE.fields_by_name['errcode'])
_STATE.fields_by_name['errcode'].containing_oneof = _STATE.oneofs_by_name['_errcode']
_STATE.oneofs_by_name['_imu_a'].fields.append(
  _STATE.fields_by_name['imu_a'])
_STATE.fields_by_name['imu_a'].containing_oneof = _STATE.oneofs_by_name['_imu_a']
_STATE.oneofs_by_name['_motor_x'].fields.append(
  _STATE.fields_by_name['motor_x'])
_STATE.fields_by_name['motor_x'].containing_oneof = _STATE.oneofs_by_name['_motor_x']
_STATE.oneofs_by_name['_motor_v'].fields.append(
  _STATE.fields_by_name['motor_v'])
_STATE.fields_by_name['motor_v'].containing_oneof = _STATE.oneofs_by_name['_motor_v']
_TARGET.oneofs_by_name['_trgt_x'].fields.append(
  _TARGET.fields_by_name['trgt_x'])
_TARGET.fields_by_name['trgt_x'].containing_oneof = _TARGET.oneofs_by_name['_trgt_x']
_TARGET.oneofs_by_name['_trgt_v'].fields.append(
  _TARGET.fields_by_name['trgt_v'])
_TARGET.fields_by_name['trgt_v'].containing_oneof = _TARGET.oneofs_by_name['_trgt_v']
_TARGET.oneofs_by_name['_trgt_a'].fields.append(
  _TARGET.fields_by_name['trgt_a'])
_TARGET.fields_by_name['trgt_a'].containing_oneof = _TARGET.oneofs_by_name['_trgt_a']
DESCRIPTOR.message_types_by_name['Request'] = _REQUEST
DESCRIPTOR.message_types_by_name['Response'] = _RESPONSE
DESCRIPTOR.message_types_by_name['Config'] = _CONFIG
DESCRIPTOR.message_types_by_name['State'] = _STATE
DESCRIPTOR.message_types_by_name['Target'] = _TARGET
DESCRIPTOR.enum_types_by_name['RequestType'] = _REQUESTTYPE
DESCRIPTOR.enum_types_by_name['ResponseStatus'] = _RESPONSESTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Request = _reflection.GeneratedProtocolMessageType('Request', (_message.Message,), {
  'DESCRIPTOR' : _REQUEST,
  '__module__' : 'controller_pb2'
  # @@protoc_insertion_point(class_scope:Request)
  })
_sym_db.RegisterMessage(Request)

Response = _reflection.GeneratedProtocolMessageType('Response', (_message.Message,), {
  'DESCRIPTOR' : _RESPONSE,
  '__module__' : 'controller_pb2'
  # @@protoc_insertion_point(class_scope:Response)
  })
_sym_db.RegisterMessage(Response)

Config = _reflection.GeneratedProtocolMessageType('Config', (_message.Message,), {
  'DESCRIPTOR' : _CONFIG,
  '__module__' : 'controller_pb2'
  # @@protoc_insertion_point(class_scope:Config)
  })
_sym_db.RegisterMessage(Config)

State = _reflection.GeneratedProtocolMessageType('State', (_message.Message,), {
  'DESCRIPTOR' : _STATE,
  '__module__' : 'controller_pb2'
  # @@protoc_insertion_point(class_scope:State)
  })
_sym_db.RegisterMessage(State)

Target = _reflection.GeneratedProtocolMessageType('Target', (_message.Message,), {
  'DESCRIPTOR' : _TARGET,
  '__module__' : 'controller_pb2'
  # @@protoc_insertion_point(class_scope:Target)
  })
_sym_db.RegisterMessage(Target)


_RESPONSE.fields_by_name['message']._options = None
# @@protoc_insertion_point(module_scope)