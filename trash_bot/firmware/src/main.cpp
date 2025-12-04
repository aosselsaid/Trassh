/**
 * Trash Bot ESP32 Firmware
 * 
 * Receives velocity commands from Raspberry Pi via Serial
 * Controls two DC motors with PID control
 * Reads quadrature encoders and sends tick data back to Pi
 * 
 * Hardware:
 * - ESP32 Dev Board
 * - Motor Driver (H-Bridge) - e.g., L298N or TB6612FNG
 * - 2x DC Motors with Quadrature Encoders
 * 
 * Pin Configuration:
 * Left Motor:
 *   - PWM: GPIO 25
 *   - DIR1: GPIO 26
 *   - DIR2: GPIO 27
 *   - Encoder A: GPIO 32
 *   - Encoder B: GPIO 33
 * 
 * Right Motor:
 *   - PWM: GPIO 12
 *   - DIR1: GPIO 13
 *   - DIR2: GPIO 14
 *   - Encoder A: GPIO 34
 *   - Encoder B: GPIO 35
 */

#include <Arduino.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

// Left Motor Pins
const int LEFT_PWM_PIN = 25;
const int LEFT_DIR1_PIN = 26;
const int LEFT_DIR2_PIN = 27;
const int LEFT_ENC_A_PIN = 32;
const int LEFT_ENC_B_PIN = 33;

// Right Motor Pins
const int RIGHT_PWM_PIN = 12;
const int RIGHT_DIR1_PIN = 13;
const int RIGHT_DIR2_PIN = 14;
const int RIGHT_ENC_A_PIN = 34;
const int RIGHT_ENC_B_PIN = 35;

// ============================================================================
// CONSTANTS
// ============================================================================

const int PWM_FREQUENCY = 20000;  // 20 kHz
const int PWM_RESOLUTION = 8;     // 8-bit (0-255)
const int PWM_CHANNEL_LEFT = 0;
const int PWM_CHANNEL_RIGHT = 1;

const int ENCODER_TICKS_PER_REV = 360;  // Adjust based on your encoder
const float WHEEL_RADIUS = 0.0325;      // meters
const float WHEEL_CIRCUMFERENCE = 2.0 * PI * WHEEL_RADIUS;
const float METERS_PER_TICK = WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REV;

const unsigned long SERIAL_SEND_INTERVAL = 50;  // milliseconds (20 Hz)
const unsigned long PID_UPDATE_INTERVAL = 10;    // milliseconds (100 Hz)

// ============================================================================
// PID CONTROLLER CLASS
// ============================================================================

class PIDController {
private:
  float kp, ki, kd;
  float setpoint;
  float integral;
  float previous_error;
  float output_min, output_max;
  
public:
  PIDController(float p, float i, float d, float min_out, float max_out) 
    : kp(p), ki(i), kd(d), setpoint(0), integral(0), 
      previous_error(0), output_min(min_out), output_max(max_out) {}
  
  void setSetpoint(float sp) {
    setpoint = sp;
  }
  
  float compute(float measured_value, float dt) {
    float error = setpoint - measured_value;
    
    integral += error * dt;
    
    // Anti-windup
    if (integral > output_max) integral = output_max;
    if (integral < output_min) integral = output_min;
    
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    
    float output = kp * error + ki * integral + kd * derivative;
    
    // Clamp output
    if (output > output_max) output = output_max;
    if (output < output_min) output = output_min;
    
    return output;
  }
  
  void reset() {
    integral = 0;
    previous_error = 0;
  }
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// Encoder counts
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// Target velocities (m/s)
float left_target_velocity = 0.0;
float right_target_velocity = 0.0;

// Measured velocities (m/s)
float left_measured_velocity = 0.0;
float right_measured_velocity = 0.0;

// Previous encoder counts for velocity calculation
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// PID Controllers
PIDController left_pid(50.0, 20.0, 5.0, -255.0, 255.0);
PIDController right_pid(50.0, 20.0, 5.0, -255.0, 255.0);

// Timing
unsigned long last_serial_send = 0;
unsigned long last_pid_update = 0;
unsigned long last_velocity_calc = 0;

// Serial communication buffer
byte serial_buffer[16];
int buffer_index = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B_PIN)) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B_PIN)) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

void setMotorSpeed(int pwm_channel, int dir1_pin, int dir2_pin, float pwm_value) {
  if (pwm_value > 0) {
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
    ledcWrite(pwm_channel, (int)abs(pwm_value));
  } else if (pwm_value < 0) {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);
    ledcWrite(pwm_channel, (int)abs(pwm_value));
  } else {
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, LOW);
    ledcWrite(pwm_channel, 0);
  }
}

void stopMotors() {
  setMotorSpeed(PWM_CHANNEL_LEFT, LEFT_DIR1_PIN, LEFT_DIR2_PIN, 0);
  setMotorSpeed(PWM_CHANNEL_RIGHT, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, 0);
}

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

void processSerialCommand() {
  // Message format from Pi:
  // [0xFF][0x01][left_vel(float)][right_vel(float)][checksum]
  // Total: 11 bytes
  
  if (Serial.available() >= 11) {
    // Look for start byte
    if (Serial.read() == 0xFF) {
      byte cmd_type = Serial.read();
      
      if (cmd_type == 0x01) {  // Velocity command
        // Read velocity data
        byte vel_data[8];
        Serial.readBytes(vel_data, 8);
        byte checksum = Serial.read();
        
        // Verify checksum
        byte calc_checksum = cmd_type;
        for (int i = 0; i < 8; i++) {
          calc_checksum += vel_data[i];
        }
        calc_checksum %= 256;
        
        if (checksum == calc_checksum) {
          // Extract velocities
          float left_vel, right_vel;
          memcpy(&left_vel, vel_data, 4);
          memcpy(&right_vel, vel_data + 4, 4);
          
          left_target_velocity = left_vel;
          right_target_velocity = right_vel;
          
          // Update PID setpoints
          left_pid.setSetpoint(left_target_velocity);
          right_pid.setSetpoint(right_target_velocity);
        }
      }
    }
  }
}

void sendEncoderData() {
  // Message format to Pi:
  // [0xFF][0x02][left_ticks(int32)][right_ticks(int32)][checksum]
  // Total: 11 bytes
  
  byte message[11];
  message[0] = 0xFF;  // Start byte
  message[1] = 0x02;  // Message type: encoder data
  
  // Copy encoder ticks
  memcpy(message + 2, &left_encoder_ticks, 4);
  memcpy(message + 6, &right_encoder_ticks, 4);
  
  // Calculate checksum
  byte checksum = 0;
  for (int i = 1; i < 10; i++) {
    checksum += message[i];
  }
  message[10] = checksum % 256;
  
  // Send message
  Serial.write(message, 11);
}

// ============================================================================
// VELOCITY CALCULATION
// ============================================================================

void calculateVelocities(float dt) {
  // Calculate change in ticks
  long d_left = left_encoder_ticks - prev_left_ticks;
  long d_right = right_encoder_ticks - prev_right_ticks;
  
  // Calculate velocities (m/s)
  left_measured_velocity = (d_left * METERS_PER_TICK) / dt;
  right_measured_velocity = (d_right * METERS_PER_TICK) / dt;
  
  // Update previous counts
  prev_left_ticks = left_encoder_ticks;
  prev_right_ticks = right_encoder_ticks;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  // Configure motor pins
  pinMode(LEFT_DIR1_PIN, OUTPUT);
  pinMode(LEFT_DIR2_PIN, OUTPUT);
  pinMode(RIGHT_DIR1_PIN, OUTPUT);
  pinMode(RIGHT_DIR2_PIN, OUTPUT);
  
  // Configure PWM
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, PWM_CHANNEL_LEFT);
  ledcAttachPin(RIGHT_PWM_PIN, PWM_CHANNEL_RIGHT);
  
  // Configure encoder pins
  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), rightEncoderISR, RISING);
  
  // Stop motors
  stopMotors();
  
  // Initialize timing
  last_serial_send = millis();
  last_pid_update = millis();
  last_velocity_calc = millis();
  
  Serial.println("Trash Bot ESP32 Firmware Initialized");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long current_time = millis();
  
  // Process incoming serial commands
  processSerialCommand();
  
  // Calculate velocities at regular intervals
  if (current_time - last_velocity_calc >= 50) {  // 20 Hz
    float dt = (current_time - last_velocity_calc) / 1000.0;
    calculateVelocities(dt);
    last_velocity_calc = current_time;
  }
  
  // Update PID and motor control
  if (current_time - last_pid_update >= PID_UPDATE_INTERVAL) {
    float dt = (current_time - last_pid_update) / 1000.0;
    
    // Compute PID outputs
    float left_pwm = left_pid.compute(left_measured_velocity, dt);
    float right_pwm = right_pid.compute(right_measured_velocity, dt);
    
    // Apply to motors
    setMotorSpeed(PWM_CHANNEL_LEFT, LEFT_DIR1_PIN, LEFT_DIR2_PIN, left_pwm);
    setMotorSpeed(PWM_CHANNEL_RIGHT, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, right_pwm);
    
    last_pid_update = current_time;
  }
  
  // Send encoder data to Pi
  if (current_time - last_serial_send >= SERIAL_SEND_INTERVAL) {
    sendEncoderData();
    last_serial_send = current_time;
  }
}
