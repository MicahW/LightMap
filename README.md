### Light Map ###
A two-wheeled Arduino robot controlled by a Raspberry PI over Bluetooth that generates a heat map of light.

#### Message Protocols ####
Each message will contain a 1 byte message ID followed by a multi-byte
payload consisting of a number of bytes determined by the particular ID.

```
Message      ::= MotorReq | ServoReq | UltraSonicReq | UltraSonicResp |
                 PingReq | PingResp | MoveStraightReq | StopReq | RotateReq

MotorReq         ::= '0x01' RightMotor LeftMotor
RightMotor       ::= MotorByte
LeftMotor        ::= MotorByte
MotorByte        ::= DirectionBit Speed7Bits
DirectionBit     ::= '0'   # Forward
                     | '1' # Backward
Speed7Bits       ::= '0x00' thru '0x7F'

ServoReq         ::= '0x02' ServoAngle
ServoAngle       ::= 8BitSigned # -80 thru 80

UltraSonicReq    ::= '0x03'
UltraSonicResp   ::= '0x04' UltraSonicValue
UltraSonicValue  ::= '0x00000000' thru '0xFFFFFFFF'

PingReq          ::= '0x05'
PingResp         ::= '0x06'

MoveStraightReq  ::= '0x07'

StopReq          ::= '0x08'

RotateReq        ::= '0x09' RotateDirection StepCount
RotateDirection  ::= '0x00'   # Clockwise
                     | '0x01' # Counter Clockwise
StepCount        ::= '0x00' thru '0xFF'

```
