#include <stdint.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define MIN_STEP_DELAY 2000  // Minimum delay in microseconds of the motor (highest speed)
#define MAX_STEP_DELAY 20000 // Maximum delay (lowest speed)

// Pin mappings
#define BLUETOOTH_RX_PIN 12
#define BLUETOOTH_TX_PIN 13
#define SERVO_CONTROL_PIN 11

// Servo motor positions
#define SERVO_MIN_DEGREES 20
#define SERVO_MAX_DEGREES 180

#define MOTOR_STATE_COUNT 8  // Number of motor states in a cycle

// Motor control takes up pins 2-9
#define MOTOR_COUNT 2 // Number of motors
#define MOTOR_PIN_COUNT 4  // Number of pins to control one motor
#define RIGHT_MOTOR_START_NUMBER  2  // Start index of the left motor
#define LEFT_MOTOR_START_NUMBER  6  // Start index of the right motor
#define RIGHT_MOTOR_INDEX 0  // Index of right motor in motor_states
#define LEFT_MOTOR_INDEX 1  // Index of left motor

// Motor control mask information
#define MOTOR_CONTROL_DIRECTION_MASK 0x80
#define MOTOR_CONTROL_DIRECTION_INDEX 7
#define MOTOR_CONTROL_VALUE_MASK 0x7F

// Receive buffer length for incoming bluetooth messages
#define RECEIVE_BUFFER_LENGTH 16

// Message IDs and sizes
#define MOTOR_CONTROL_ID 1
#define MOTOR_CONTROL_MESSAGE_SIZE 2
#define SERVERO_CONTROL_ID 2
#define SERVERO_CONTROL_MESSAGE_SIZE 1

// A state the motor bridge pins can be in
struct MotorBridgeState {
  uint8_t bridge_count;  // 0 for bridge_0 only, 1 for both bridges
  uint8_t bridge_0;  // pin offset for the first bridge
  uint8_t bridge_1;  // pin offset for the second bridge
};

// Current state of one motor
struct MotorState {
  // Motor specific data
  const uint8_t reversed;  // 1 if this motor has a reversed step
  const uint8_t pin_0_index;  // Index of pin zero, the rest of the pins will be in sequence

  // State information
  unsigned long last_step_time;  // Time of last motor step in microseconds
  unsigned long step_delay;  // Delay in microseconds between each step (0 indicates no step should be taken)
  uint8_t direction;  // <0 = foward, 1 = backward>
  int8_t bridge_state_index;  // Current index within the motor_bridge_states
};

// Each motor bridge state to cycle between in order to make the motor spin using half steps
const MotorBridgeState motor_bridge_states[MOTOR_STATE_COUNT] {
  MotorBridgeState {.bridge_count = 0, .bridge_0 = 0,},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 0, .bridge_1 = 2},
  MotorBridgeState {.bridge_count = 0, .bridge_0 = 2,},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 1, .bridge_1 = 2},
  MotorBridgeState {.bridge_count = 0, .bridge_0 = 1,},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 1, .bridge_1 = 3},
  MotorBridgeState {.bridge_count = 0, .bridge_0 = 3,},
  MotorBridgeState {.bridge_count = 1, .bridge_0 = 0, .bridge_1 = 3}
};

// Define the bluetooth serial connection
const SoftwareSerial BTserial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN);

// Create the servo motor object
const Servo servo_motor;
 
// States for each motor, starts as set for no movement
MotorState motor_states[MOTOR_COUNT] {
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

// Buffer for receiving bluetooth messages
uint8_t receive_buffer[RECEIVE_BUFFER_LENGTH];

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

// Set the motor bridge states to step the motor if it is time to do so
void stepMotorsIfTime() {
  unsigned long current_time = micros();
  for (uint8_t index = 0; index < MOTOR_COUNT; index++) {
    MotorState *motor_state = &motor_states[index];

    // If delay is set to zero then the motor should not be stepped
    if (motor_state->step_delay == 0) {
      continue;
    }

    // If enough time has not yet passed then the motor should not be stepped
    if (current_time < (motor_state->last_step_time + motor_state->step_delay)) {
      continue;
    }

    // Set the bridge state index to the next index in the sequence
    int8_t incr = (motor_state->reversed == motor_state->direction) ? 1 : -1;
    motor_state->bridge_state_index += incr;
    if (motor_state->bridge_state_index < 0) {
      motor_state->bridge_state_index = MOTOR_STATE_COUNT - 1;
    }
    motor_state->bridge_state_index %= MOTOR_STATE_COUNT;

    // Turn all motor pins off
    for (uint8_t pin_offset = 0; pin_offset < (MOTOR_PIN_COUNT); pin_offset++) {
      digitalWrite(pin_offset + motor_state->pin_0_index, LOW);
    }

    // Turn the next motor pins on
    MotorBridgeState *bridge_state = &motor_bridge_states[motor_state->bridge_state_index];
    digitalWrite(bridge_state->bridge_0 + motor_state->pin_0_index, HIGH);
    if (bridge_state->bridge_count == 1) {
      digitalWrite(bridge_state->bridge_1 + motor_state->pin_0_index, HIGH);
    }

    // Set the new motor last read time
    motor_state->last_step_time = current_time;
  }
}

/**
 * Sets the Servo motor position from a motor control message byte
 * degrees_byte: The servo control byte
 */
void setServoPos(uint8_t degrees_byte) {
  int8_t servo_degrees = static_cast<int8_t>(degrees_byte);
  int8_t servo_center = (SERVO_MAX_DEGREES + SERVO_MIN_DEGREES) / 2;
  int8_t translated_degrees = servo_center + servo_degrees;

  // Set the motor position if the position is valid
  if ((translated_degrees < SERVO_MIN_DEGREES) || (translated_degrees > SERVO_MAX_DEGREES)) {
    return;
  }
  servo_motor.write(translated_degrees);
}

// Read a fixed number of bytes into the receive buffer
// num_bytes: The number of bytes to read into the buffer
void readBytesIntoReceiveBuffer(uint8_t num_bytes) {
  for (uint8_t index = 0; index < num_bytes; index++) {
    while(!BTserial.available()) {}
    receive_buffer[index] = BTserial.read();
  }
}

// Process a bluetooth message if one is available
void processBluetoothMessageIfAvailable() {
    // Check if a messages is available
    if (!BTserial.available()) {
      return;
    }

    // Read the message ID
    uint8_t message_id = BTserial.read();

    // If a Motor Control Message
    if (message_id == MOTOR_CONTROL_ID) {
      readBytesIntoReceiveBuffer(MOTOR_CONTROL_MESSAGE_SIZE);
      // Set motor speed from control bytes
      setMotorDelayAndDirection(receive_buffer[0], RIGHT_MOTOR_INDEX);
      setMotorDelayAndDirection(receive_buffer[1], LEFT_MOTOR_INDEX);
    }

    // If a servo control message
    if (message_id == SERVERO_CONTROL_ID) {
      readBytesIntoReceiveBuffer(SERVERO_CONTROL_MESSAGE_SIZE);
      // set servo position
      setServoPos(receive_buffer[0]);
    }
}

// Arduino one time setup
void setup()
{
  // Set the motor pins to output
  for(int pin = RIGHT_MOTOR_START_NUMBER; pin < (LEFT_MOTOR_START_NUMBER + MOTOR_PIN_COUNT); pin++ ) {
    pinMode(pin, OUTPUT);
  }
  // Init bluetooth serial connection
  BTserial.begin(9600);
  BTserial.listen();

  // Attach the servo motor and center it
  servo_motor.attach(SERVO_CONTROL_PIN);
  servo_motor.write((SERVO_MAX_DEGREES + SERVO_MIN_DEGREES) / 2);
}

// Control loop, processes messages and runs motors
void loop()
{
  stepMotorsIfTime();
  processBluetoothMessageIfAvailable();
}
