### Light Map ###
A two-wheeled Arduino robot controlled by a Raspberry PI over Bluetooth that generates a heat map of light.

#### Message Protocols ####
Each message will contain a 1 byte message ID followed by the payload

- Raw motor control message [ ID = 1 | right motor control byte | left motor control byte]
  -   motor control byte = {1 bit <0 = foward, 1 = backward> 7 bits speed in range (0 - 0x7F)}