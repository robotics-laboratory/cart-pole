# CartPole Controller Protocol

- [Overview](#overview)
- [Variables](#variables)
  - [Config group](#config)
  - [State group](#state)
  - [Target group](#target)
  - [Error codes](#error-codes-enum)
- [Commands](#commands)
  - [Get variable](#get-group-key1-key2-)
  - [Set variable](#set-group-key1val1-key2val2-)
  - [Reset variable](#reset-group-key1-key2-)
  - [Homing](#homing)

## Overview

This document defines communication protocol between higher-level SBC ("brain") 
and lower-level MCU ("controller"). Brain operates with high-level commands like 
"go to x=0.1m", "accelerate to v=0.5m/s", "get current pole angle". Controller 
is responsible for translating these commands to underlying hardware (stepper 
motor, encoder, endstop switches etc.). Controller also implements safety checks 
to prevent incorrect brain commands from damaging the device (e.g. when the brain 
asks for x=10m when max allowed x is 1m).

Communication is performed through serial connection, using plain-text messages 
with ASCII encoding. This allows easy (de)serialization and ability to manually 
debug the device. A message is the smallest protocol entity that can be sent or 
received. Messages are separated by UNIX-style "\n" delimiter. Protocol follows 
a "request-reply" strategy, where the brain only sends requests, and the controller 
only sends replies to those requests. Request-reply pairs (also called “commands”) 
are described in a separate section below. There are several types of replies, 
which can be determined by message prefix:

- `+ <data>` - Successful request. <data> is a response payload.
- `! <data>` - Request failed. <data> is an error description.
- `# <data>` - Debug message. Could be ignored and/or logged.
- `~` - Request in progress. This message is sent to prevent timeouts.

The main rule of a command is that it must eventually return either successful (`+`) or unsuccessful (`!`) response.
If it takes a long time to process a command (e.g. homing), the controller should send multiple `~` messages to 
prevent a timeout. The brain can consider a fatal communication error in two cases:

- After sending a request, there’s no `+`, `!` or `~` replies for longer than 1 second.
- After sending a request, there’s multiple `~` replies, but no `+` or `!` reply for longer than 30 seconds.

If a fatal error is detected, the best thing the brain can do is to hard-reset the controller.
Hopefully, that can be done by re-opening the serial connection. In order for this to work, 
the serial adapter must have DTR/RTS output connected to MCU reset pin.

## Variables
  
### Config

| KEY      | TYPE         | DEFAULT | RANGE         | COMMENT                                  |
|----------|--------------|---------|---------------|------------------------------------------|
| MAX_X    | m, float     | 0       | [0, HW_MAX_X] | Max cart position (X=0 is center)        |
| MAX_V    | m/s, float   | 0.5     | [0, HW_MAX_V] | Max cart velocity                        |
| MAX_A    | m/s^2, float | 1.0     | [0, HW_MAX_A] | Max cart acceleration                    |
| HW_MAX_X | m, float     | 0       | -             | [READONLY] Max allowed cart position     |
| HW_MAX_V | m/s, float   | TBD     | -             | [READONLY] Max allowed cart velocity     |
| HW_MAX_A | m/s^2, float | TBD     | -             | [READONLY] Max allowed cart acceleration |
| CLAMP_X  | flag, bool   | FALSE   | true/false    | Round X to allowed range without errors  |
| CLAMP_V  | flag, bool   | FALSE   | true/false    | Round V to allowed range without errors  |
| CLAMP_A  | flag, bool   | FALSE   | true/false    | Round A to allowed range without errors  |
  
Notes about `HW_*` variables:
- `HW_MAX_X` is updated after the homing procedure.
- `HW_MAX_V` and `HW_MAX_A` are hardcoded values, determined by stress-tests. It’s recommended to 
always set `MAX_V` and `MAX_A` slightly lower than `HW_MAX_V` and `HW_MAX_A`. Otherwise, the device 
operation may be unstable (probably, `MOTOR_STALLED` error will be raised at some point).

### State

| KEY     | TYPE         | DEFAULT | RANGE | COMMENT                                        |
|---------|--------------|---------|-------|------------------------------------------------|
| X       | m, float     | 0       | -     | [READONLY] Current cart position               |
| V       | m/s, float   | 0       | -     | [READONLY] Current cart velocity               |
| A       | m/s^2, float | 0       | -     | [READONLY] Current cart acceleration           |
| POLE_X  | rad, float   | 0       | -     | [READONLY] Current pole angle                  |
| POLE_V  | rad/s, float | 0       | -     | [READONLY] Current pole angular velocity       |
| ERRCODE | enum, int    | 1       | -     | [READONLY] Current error code (see enum below) |


### Target

| KEY | TYPE         | DEFAULT | RANGE           | COMMENT                  |
|-----|--------------|---------|-----------------|--------------------------|
| X   | m, float     | 0       | [-MAX_A, MAX_X] | Target cart position     |
| V   | m/s, float   | 0       | [-MAX_V, MAX_V] | Target cart velocity     |
| A   | m/s^2, float | 0       | [-MAX_A, MAX_A] | Target cart acceleration |
  
Note: validation logic of TRGT_* variables depends on CLAMP_* flags. If set, invalid 
values are silently clamped to allowed range. Otherwise, corresponding *_OVERFLOW error is raised.

### Error codes (enum):

| KEY           | INT | COMMENT                                                               |
|---------------|-----|-----------------------------------------------------------------------|
| NO_ERROR      | 0   | This is fine (no errors, motion is allowed)                           |
| NEED_HOMING   | 1   | Device needs homing (see homing command)                              |
| X_OVERFLOW    | 2   | Current or target position was out of allowed range                   |
| V_OVERFLOW    | 3   | Current or target velocity was out of allowed range                   |
| A_OVERFLOW    | 4   | Current or target acceleration was out of allowed range               |
| MOTOR_STALLED | 5   | Stepper driver detected motor failure (TMC’s StallGuard is triggered) |
| ENDSTOP_HIT   | 6   | One of endstops is triggered during movement                          |
  
## Commands
  
### `get <group> [key1 key2 ...]`
Returns values for given keys in the form of space-separated “key=value” pairs. If no keys are specified, returns all keys from gived group.
```
>>> get config max_x
<<< + max_x=123.456
>>> get state non_existent_key
<<< ! No such key: non_existent_key
>>> get target
<<< + x=123 v=456 a=789
```

### `set <group> [key1=val1 key2=val2 ...]`
Updates values of given keys and returns them back (format matches “get” command).
```
>>> set state x=123
<<< ! This key is readonly
>>> set config max_v=1.0 max_a=2.0
<<< + max_v=1.0 max_a=2.0
>>> config set max_v=1000
<<< ! Value out of range: 1000 > 10 [at max_v=1000]
```

### `reset <group> [key1 key2 ...]`
Resets given keys to default values and returns them back (format matches “get” command). If no keys are specified, resets all keys from gived group.
```
>>> reset config max_v max_a
<<< + max_v=0.5 max_a=1.0
>>> reset target
<<< + x=0 v=0 a=0
```

### `homing`
Performs homing procedure. This command may take a long time to complete, so it will periodically send keepalive “~” messages. After completion, returns the word “ok”.
```
>>> homing
<<< ~
<<< ~
<<< ~
<<< ~
<<< + ok
```
