# UART Protocol

## Frame Format
- Header: `0xAA 0x55`
- Length: 1 byte (data payload length)
- Command: 1 byte
- Data: variable (`Length` bytes)
- CRC8: 1 byte (poly 0x07, init 0x00, covers Length + Command + Data)
- Footer: `0x0D 0x0A`

## Commands
- `0x01` CMD_MOTION_CONTROL: payload `<float speedL><float speedR>` (little-endian)
- `0x02` CMD_HEARTBEAT
- `0x03` CMD_HEARTBEAT_ACK
- `0x04` CMD_STATUS_FEEDBACK: payload `STAT` (see below)
- `0x05` CMD_ERROR_REPORT: payload `uint8 error_code`
- `0x06` CMD_REQUEST_STATUS
- `0x07` CMD_SERVO_CONTROL: payload `<uint8 id><int16 angle>`
- `0x08` CMD_REQUEST_MOTOR_SPEED
- `0x09` CMD_MOTOR_SPEED_FEEDBACK: payload `<float speedL><float speedR>`
- `0x0A` CMD_LIGHT_CONTROL: payload `<uint8 enable><uint8 brightness>`
  - `enable`: 0=off, 1=on, 0xFF=toggle
  - `brightness`: 0-100 (%); values >100 are ignored
- `0x0B` CMD_MOTOR_ENABLE: payload `<uint8 enable>` (0=disable/stop, 1=enable/start, 0xFF=toggle)

## STAT Payload (CMD_STATUS_FEEDBACK)
Packed, little-endian, total 71 bytes.

| Field | Type | Bytes |
| --- | --- | --- |
| Ready | bool | 1 |
| motorEN | bool | 1 |
| servoEN | bool | 1 |
| onlineservoid[10] | uint8[10] | 10 |
| req_servo[10] | uint8[10] | 10 |
| servo[10] | uint8[10] | 10 |
| speedLimit | uint8 | 1 |
| req_speedL | float | 4 |
| req_speedR | float | 4 |
| speedL | float | 4 |
| speedR | float | 4 |
| batteryVoltage | float | 4 |
| motorCurrentL | float | 4 |
| motorCurrentR | float | 4 |
| motorCurrent | float | 4 |
| CPU | uint8 | 1 |
| RAM | uint8 | 1 |
| error_flags | uint8 | 1 |
| lightEnabled | bool | 1 |
| lightBrightness | uint8 | 1 |

Sum: 3 + 30 + 1 + 32 + 3 + 2 = 71 bytes.

## Light Control Examples
- Toggle light: `enable=0xFF`, `brightness=0xFF` (brightness ignored)
- Set brightness to 80% and turn on: `enable=1`, `brightness=80`
- Turn off: `enable=0`, `brightness` can be 0
