### Light Map ###
A two-wheeled Arduino robot controlled by a Raspberry PI over Bluetooth that generates a heat map of light.

#### Message Protocols ####
Each message will contain a 1 byte message ID followed by a multi-byte
payload consisting of a number of bytes determined by the particular ID.

```
Message      ::= Motor

Motor        ::= '0x01' RightMotor LeftMotor
RightMotor   ::= MotorByte
LeftMotor    ::= MotorByte
MotorByte    ::= DirectionBit Speed7Bits
DirectionBit ::= ForwardBit | BackwardBit
ForwardBit   ::= '0'
BackwardBit  ::= '1'
Speed7Bits   ::= '0x00' thru '0x7F'
```
