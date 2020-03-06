#include <stdint.h>
#include <SoftwareSerial.h>

#define MOTOR_STATE_COUNT 8

// A state the motor bridges can be in
struct MotorState {
  uint8_t bridge_count : 1;  // 0 for bridge_0 only, 1 for both bridges
  uint8_t bridge_0 : 2;  // pin offset for the first bridge
  uint8_t bridge_1 : 2;  // pin offset for the second bridge
};

// Each motor bridge state to cycle between in order to make the motor spin using half steps
const MotorState motor_states[MOTOR_STATE_COUNT] {
  MotorState {.bridge_count = 1, .bridge_0 = 0,},
  MotorState {.bridge_count = 2, .bridge_0 = 0, .bridge_1 = 2},
  MotorState {.bridge_count = 1, .bridge_0 = 2,},
  MotorState {.bridge_count = 2, .bridge_0 = 1, .bridge_1 = 2},
  MotorState {.bridge_count = 1, .bridge_0 = 1,},
  MotorState {.bridge_count = 2, .bridge_0 = 1, .bridge_1 = 3},
  MotorState {.bridge_count = 1, .bridge_0 = 3,},
  MotorState {.bridge_count = 2, .bridge_0 = 0, .bridge_1 = 3}
};

// Oopen the Bluetooth serial connection
SoftwareSerial BTserial(2, 3); // RX | TX
 
void setup() 
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  BTserial.begin(9600);  // Init bluetooth serial connection
}

// Read a length encoded serial message over bluetooth
// buf: message buffer to read into
// return: num bytes read
int readBT(char* buf) {
  if (!BTserial.available()) {
    return 0;
  }
  char message_size = BTserial.read();
  for (char pos = 0; pos < message_size; pos++) {
    buf[pos] = BTserial.read();    
  }
  buf[message_size] = '\0';
  return message_size;
}

// Read a length encoded serial message from bluetooth
// buf: message buffer to read from
// size: message size
void writeBT(char* buf, int size) {
  BTserial.write((char) size);
  for (int i = 0; i < size; i++) {
    BTserial.write(buf[i]);
  }
}
 
void loop()
{
  char message_buf[256];
  readBT(message_buf);
}
