#include <stdint.h>
#include <SoftwareSerial.h>

#define MIN_STEP_DELAY 2000  // Minimum delay in miliseconds of the motor (highest speed)
#define MAX_STEP_DELAY 20000 // Maximum delay (lowest speed)

#define MOTOR_STATE_COUNT 8  // Number of motor states in a cycle

// Motor control takes up pins 2-9
#define MOTOR_PIN_COUNT 8  // Number of pins to contorl motor
#define RIGHT_MOTOR_START_NUMBER  2  // Start index of the left motor
#define LEFT_MOTOR_START_NUMBER  6  // Start index of the right motor

// Motor control mask information
#define MOTOR_CONTROL_DIRECTION_MASK 0x80
#define MOTOR_CONTROL_DIRECTION_INDEX 7
#define MOTOR_CONTROL_VALUE_MASK 0x7F

// A state the motor bridge pins can be in
struct MotorBridgeState {
  uint8_t bridge_count : 1;  // 0 for bridge_0 only, 1 for both bridges
  uint8_t bridge_0 : 2;  // pin offset for the first bridge
  uint8_t bridge_1 : 2;  // pin offset for the second bridge
};

// Current state of one motor
struct MotorState {
  // Motor specific data
  const uint8_t reversed;  // 1 if this motor has a reversed step (one will, one wont)
  const uint8_t pin_0_index;  // Index of pin zero, the rest of the pins will be in sequence

  // State information
  unsigned long last_step_time;  // Time of last motor step in miliseconds
  unsigned long step_delay;  // Delay in miliseconds between each step (0 indicates no step should be taken)
  uint8_t direction;  // <0 = foward, 1 = backward>
  uint8_t bridge_state_index;  // Current index within the motor_bridge_states
};

// Each motor bridge state to cycle between in order to make the motor spin using half steps
const MotorBridgeState motor_bridge_states[MOTOR_STATE_COUNT] {
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 0,},
  MotorBridgeState {.bridge_count = 2, .bridge_0 = 0, .bridge_1 = 2},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 2,},
  MotorBridgeState {.bridge_count = 2, .bridge_0 = 1, .bridge_1 = 2},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 1,},
  MotorBridgeState {.bridge_count = 2, .bridge_0 = 1, .bridge_1 = 3},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 3,},
  MotorBridgeState {.bridge_count = 2, .bridge_0 = 0, .bridge_1 = 3}
};

// Define the bluetooth serial connection
const SoftwareSerial BTserial(2, 3); // RX | TX
 
// States for each motor, starts as set for no movement
MotorState motor_states[2] {
  // Right Motor
  MotorState {  .reversed = 0,
                .pin_0_index = RIGHT_MOTOR_START_NUMBER,
                .last_step_time = 0,
                .step_delay = 0,
                .direction = 0,
                .bridge_state_index = 0
             },
  // Left Motor
  MotorState {  .reversed = 1,
                .pin_0_index = LEFT_MOTOR_START_NUMBER,
                .last_step_time = 0,
                .step_delay = 0,
                .direction = 0,
                .bridge_state_index = 0
             }
};

// Set the motor delay from a control value provided in a motor control message
//
// control_value: The one byte control value from a control message
// motor_state_index: The index of the motor_state the delay is being set for
void setMotorDelayAndDirection(uint8_t control_value, uint8_t motor_state_index) {
  // Get the direction and speed value from the control value
  uint8_t direction = (control_value & MOTOR_CONTROL_DIRECTION_MASK) >> MOTOR_CONTROL_DIRECTION_INDEX;
  unsigned long value = (unsigned long) control_value & MOTOR_CONTROL_VALUE_MASK; 

  // Calculate the delay, if the speed value is 0 then delay should also be zero
  unsigned long delay = 0;
  if (value > 0) {
    delay = MAX_STEP_DELAY - ((value * (MAX_STEP_DELAY - MIN_STEP_DELAY)) / MOTOR_CONTROL_VALUE_MASK);
  }

  // Set the motor states
  motor_states[motor_state_index].step_delay = delay;
  motor_states[motor_state_index].direction = direction;
}

void setup() 
{
#ifdef DEBUG
  Serial.begin(9600);
#endif
  // Set the motor pins to output
  for(int pin = RIGHT_MOTOR_START_NUMBER; pin < (RIGHT_MOTOR_START_NUMBER + MOTOR_PIN_COUNT); pin++ ) {
    pinMode(pin, OUTPUT);
  }
  // Init bluetooth serial connection
  BTserial.begin(9600);
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
