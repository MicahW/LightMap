#include <stdint.h>
#include <Servo.h>
#include <SoftwareSerial.h>

static const uint32_t kMinStepDelay = 2000;  // Minimum delay in microseconds of the motor (highest speed)
static const uint32_t kMaxStepDelay = 20000;  // Maximum delay (lowest speed)

// Pin mappings
static const size_t kBluetoothRxPin = 12;
static const size_t kBluetoothTxPin = 13;
static const size_t kServoControlPin = 11;
static const size_t kUltrasonicTriggerPin = 1;
static const size_t kUltrasconicEchoPin = 0;

// Servo motor positions
static const uint32_t kServoMinDegrees = 20;
static const uint32_t kServoMaxDegrees = 180;

// Rotation step speed
static const uint32_t kRotateSpeed = 0x1F;

static const size_t kMotorStateCount = 8;  // Number of motor states in a cycle

// Motor control takes up pins 2-9
static const size_t kMotorCount = 2; // Number of motors
static const size_t kMotorPinCount = 4;  // Number of pins to control one motor
static const size_t kRightMotorStartNumber = 2;  // Start index of the left motor
static const size_t kLeftMotorStartNumber = 6;  // Start index of the right motor
static const size_t kRighMotorIndex = 0;  // Index of right motor in motor_states
static const size_t kLeftMotorIndex = 1; // Index of left motor

// Motor control mask information
static const uint8_t kMotorControlDirectionMask = 0x80;
static const uint8_t kMotorControlDirectionIndex = 7;
static const uint8_t kMotorControlValueMask = 0x7F;

// Receive buffer length for incoming bluetooth messages
static const size_t kReceiveBufferLenght = 16;

// Message IDs and sizes
static const size_t kMotorControlId = 1;
static const size_t kMotorControlMessageSize = 2;
static const size_t kServoControlId = 2;
static const size_t kServoControlMessageSize = 1;
static const size_t kUltrasonicRequestId = 3;
static const size_t kUltrasonicResponseId = 4;
static const size_t kUltrasonicResponseSize = 4;
static const size_t kPingRequestId = 5;
static const size_t kPingResponseId = 6;
static const size_t kMoveStraightId = 7;
static const size_t kStopId = 8;
static const size_t kRotateId = 9;
static const size_t kRotateRequestSize = 2;

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
const MotorBridgeState motor_bridge_states[kMotorStateCount] {
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
const SoftwareSerial BTserial(kBluetoothRxPin, kBluetoothTxPin);

// Create the servo motor object
const Servo servo_motor;
 
// States for each motor, starts as set for no movement
MotorState motor_states[kMotorCount] {
  // Right Motor
  MotorState {  .reversed = 0,
                .pin_0_index = kRightMotorStartNumber,
                .last_step_time = 0,
                .step_delay = 0,
                .direction = 0,
                .bridge_state_index = 0
             },
  // Left Motor
  MotorState {  .reversed = 1,
                .pin_0_index = kLeftMotorStartNumber,
                .last_step_time = 0,
                .step_delay = 0,
                .direction = 0,
                .bridge_state_index = 0
             }
};

// Buffer for receiving bluetooth messages
uint8_t receive_buffer[kReceiveBufferLenght];

// Amount of steps taken by the motor, used to record distance travled in a straight line and
// to control how many steps are used in a turn operation
uint32_t steps_taken = 0;

// Set the motor delay from a control value provided in a motor control message
//
// control_value: The one byte control value from a control message
// motor_state_index: The index of the motor_state the delay is being set for
void setMotorDelayAndDirection(uint8_t control_value, uint8_t motor_state_index) {
  // Get the direction and speed value from the control value
  uint8_t direction = (control_value & kMotorControlDirectionMask) >> kMotorControlDirectionIndex;
  unsigned long value = (unsigned long) control_value & kMotorControlValueMask;

  // Calculate the delay, if the speed value is 0 then delay should also be zero
  unsigned long delay = 0;
  if (value > 0) {
    delay = kMaxStepDelay - ((value * (kMaxStepDelay - kMinStepDelay)) / kMotorControlValueMask);
  }

  // Set the motor states
  motor_states[motor_state_index].step_delay = delay;
  motor_states[motor_state_index].direction = direction;
}

// Set the motor bridge states to step the motor if it is time to do so
void stepMotorsIfTime() {
  unsigned long current_time = micros();
  bool step_taken = false;
  for (uint8_t index = 0; index < kMotorCount; index++) {
    MotorState *motor_state = &motor_states[index];

    // If delay is set to zero then the motor should not be stepped
    if (motor_state->step_delay == 0) {
      continue;
    }

    // If enough time has not yet passed then the motor should not be stepped
    if (current_time < (motor_state->last_step_time + motor_state->step_delay)) {
      continue;
    }

    step_taken = true;

    // Set the bridge state index to the next index in the sequence
    int8_t incr = (motor_state->reversed == motor_state->direction) ? 1 : -1;
    motor_state->bridge_state_index += incr;
    if (motor_state->bridge_state_index < 0) {
      motor_state->bridge_state_index = kMotorStateCount - 1;
    }
    motor_state->bridge_state_index %= kMotorStateCount;

    // Turn all motor pins off
    for (uint8_t pin_offset = 0; pin_offset < (kMotorPinCount); pin_offset++) {
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
  // Increment steps taken if a step was taken by either motor, this is usefull only when the motors are in lock step
  if (step_taken) {
    steps_taken ++;
  }
}

/**
 * Rotate using the motors
 * direction: <0 = right, 1 = left>
 * steps: number of steps to rotate by
 */
void rotate(uint8_t direction, uint8_t steps) {
  // Reset steps taken
  steps_taken = 0;

  // Set the motor states for rotation
  uint8_t left_motor_speed = kRotateSpeed;
  uint8_t right_motor_speed = kRotateSpeed;

  // Determin which motor should be reversed
  if (direction == 0) {
    left_motor_speed |= kMotorControlDirectionMask;
  } else if (direction == 1) {
    right_motor_speed |= kMotorControlDirectionMask;
  } else {
    return;
  }

  // Set the motor states
  setMotorDelayAndDirection(right_motor_speed, kRighMotorIndex);
  setMotorDelayAndDirection(left_motor_speed, kLeftMotorIndex);

  // Allow the motors to complete each step
  while (steps_taken != static_cast<uint32_t>(steps)) {
    stepMotorsIfTime();
  }

  // Stop the motors
  setMotorDelayAndDirection(0, kRighMotorIndex);
  setMotorDelayAndDirection(0, kLeftMotorIndex);
}

/**
 * Sets the Servo motor position from a motor control message byte
 * degrees_byte: The servo control byte
 */
void setServoPos(uint8_t degrees_byte) {
  int8_t servo_degrees = static_cast<int16_t>(degrees_byte);
  int16_t servo_center = (kServoMaxDegrees + kServoMinDegrees) / 2;
  int16_t translated_degrees = static_cast<int16_t>(servo_center + servo_degrees);

  // Set the motor position if the position is valid
  if ((translated_degrees < kServoMinDegrees) || (translated_degrees > kServoMaxDegrees)) {
    return;
  }
  servo_motor.write(translated_degrees);
}

/**
 * Get the duration of a ultrasonic pulse
 */
uint32_t getUltrasonicPulse() {
  // Pulse trigger pin
  digitalWrite(kUltrasonicTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kUltrasonicTriggerPin, LOW);
  uint32_t duration = pulseIn(kUltrasconicEchoPin, HIGH);
  return duration;
}

// Read a fixed number of bytes into the receive buffer
// num_bytes: The number of bytes to read into the buffer
void readBytesIntoReceiveBuffer(uint8_t num_bytes) {
  for (uint8_t index = 0; index < num_bytes; index++) {
    while(!BTserial.available()) {}
    receive_buffer[index] = BTserial.read();
  }
}

/**
 * Send a bluetooth message from payload bytes
 * buffer: pointer to payload buffer
 * size: size of payload buffer
 * message_id: message id for the message
 */
void sendBluetoothMessage(uint8_t *buffer, int size, uint8_t message_id) {
  BTserial.write(message_id);
  for (int i = 0; i < size; i++) {
    BTserial.write(buffer[i]);
  }
}

/**
 * Send the UltraSonicResponse with the duration
 */
void sendUltraSonicResponse(uint32_t duration) {
  uint8_t buffer[kUltrasonicResponseSize];
  memcpy(buffer, &duration, sizeof(duration));
  sendBluetoothMessage(buffer, kUltrasonicResponseSize, kUltrasonicResponseId);
}

// Process a bluetooth message if one is available
void processBluetoothMessageIfAvailable() {
    // Check if a messages is available
    if (!BTserial.available()) {
      return;
    }

    // Read the message ID
    uint8_t message_id = BTserial.read();

    // If a ping request
    if (message_id == kPingRequestId) {
      sendBluetoothMessage(nullptr, 0, kPingResponseId);
    }

    // If a motor control message
    else if (message_id == kMotorControlId) {
      readBytesIntoReceiveBuffer(kMotorControlMessageSize);
      // Set motor speed from control bytes
      setMotorDelayAndDirection(receive_buffer[0], kRighMotorIndex);
      setMotorDelayAndDirection(receive_buffer[1], kLeftMotorIndex);
    }

    // If a servo control message
    else if (message_id == kServoControlId) {
      readBytesIntoReceiveBuffer(kServoControlMessageSize);
      // set servo position
      setServoPos(receive_buffer[0]);
    }

    // If a ultrasonic request
    else if (message_id == kUltrasonicRequestId) {
      sendUltraSonicResponse(getUltrasonicPulse());
    }

    // If this is a move straight request
    else if (message_id == kMoveStraightId) {
      // Set both motors to move foward and reset steps taken
      setMotorDelayAndDirection(0x7F, kRighMotorIndex);
      setMotorDelayAndDirection(0x7F, kLeftMotorIndex);
      steps_taken = 0;
    }

    // If this is a rotate request
    else if (message_id == kRotateId) {
      readBytesIntoReceiveBuffer(kRotateRequestSize);
      rotate(receive_buffer[0], receive_buffer[1]);
      // TODO(mwallberg): send the motor rotate response
    }

    // If this is a stop request
    else if (message_id == kStopId) {
      setMotorDelayAndDirection(0, kRighMotorIndex);
      setMotorDelayAndDirection(0, kLeftMotorIndex);
      // TODO(mwallberg): send the steps taken to the PI
    }
}

// Arduino one time setup
void setup()
{
  // Set the motor pins to output
  for(int pin = kRightMotorStartNumber; pin < (kLeftMotorStartNumber + kMotorPinCount); pin++ ) {
    pinMode(pin, OUTPUT);
  }
  // Init bluetooth serial connection
  BTserial.begin(9600);
  BTserial.listen();

  // Attach the servo motor and center it
  servo_motor.attach(kServoControlPin);
  servo_motor.write((kServoMaxDegrees + kServoMinDegrees) / 2);

  // Set up ultrasonic pins
  pinMode(kUltrasonicTriggerPin, OUTPUT);
  pinMode(kUltrasconicEchoPin, INPUT);
  digitalWrite(kUltrasonicTriggerPin, LOW);
}

// Control loop, processes messages and runs motors
void loop()
{
  stepMotorsIfTime();
  processBluetoothMessageIfAvailable();
}
