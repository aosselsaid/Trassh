# ESP32 Firmware for Trash Bot

This directory contains the PlatformIO project for the ESP32 motor controller.

## Features

- **PID Motor Control**: Closed-loop velocity control for both motors
- **Quadrature Encoder Reading**: Accurate position tracking
- **Serial Communication**: Binary protocol for low-latency command/telemetry
- **Differential Drive Kinematics**: Converts linear/angular velocity to wheel speeds

## Hardware Requirements

- ESP32 Development Board
- Motor Driver (H-Bridge): L298N, TB6612FNG, or similar
- 2x DC Motors with Quadrature Encoders (360 ticks/rev recommended)
- Power supply for motors (typically 7-12V)

## Pin Configuration

### Left Motor
- PWM: GPIO 25
- Direction 1: GPIO 26
- Direction 2: GPIO 27
- Encoder Channel A: GPIO 32
- Encoder Channel B: GPIO 33

### Right Motor
- PWM: GPIO 12
- Direction 1: GPIO 13
- Direction 2: GPIO 14
- Encoder Channel A: GPIO 34
- Encoder Channel B: GPIO 35

## Installation

### Using PlatformIO CLI

```bash
cd firmware

# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

### Using PlatformIO IDE (VS Code)

1. Install the PlatformIO extension in VS Code
2. Open the `firmware` folder
3. Click "Build" in the PlatformIO toolbar
4. Click "Upload" to flash the ESP32
5. Click "Serial Monitor" to view output

## Serial Protocol

### Message Format: Raspberry Pi → ESP32 (Velocity Command)

```
Byte 0:     0xFF (Start byte)
Byte 1:     0x01 (Command type: Velocity)
Bytes 2-5:  Left wheel velocity (float, m/s)
Bytes 6-9:  Right wheel velocity (float, m/s)
Byte 10:    Checksum (sum of bytes 1-9, mod 256)
```

### Message Format: ESP32 → Raspberry Pi (Encoder Data)

```
Byte 0:     0xFF (Start byte)
Byte 1:     0x02 (Message type: Encoder)
Bytes 2-5:  Left encoder ticks (int32)
Bytes 6-9:  Right encoder ticks (int32)
Byte 10:    Checksum (sum of bytes 1-9, mod 256)
```

## PID Tuning

Default PID values are set in `src/main.cpp`:

```cpp
PIDController left_pid(50.0, 20.0, 5.0, -255.0, 255.0);   // kp, ki, kd, min, max
PIDController right_pid(50.0, 20.0, 5.0, -255.0, 255.0);
```

To tune:
1. Start with P-only control (set I and D to 0)
2. Increase P until oscillation occurs, then reduce by 50%
3. Add I to eliminate steady-state error
4. Add D to reduce overshoot (usually small value)

## Update Rates

- Serial send (encoder data): 20 Hz (50ms)
- PID update: 100 Hz (10ms)
- Velocity calculation: 20 Hz (50ms)

## Encoder Configuration

If your encoders have a different resolution, update:

```cpp
const int ENCODER_TICKS_PER_REV = 360;  // Change to match your encoder
```

## Motor Driver Wiring

### For L298N:
- ENA → ESP32 PWM (Left)
- IN1 → ESP32 DIR1 (Left)
- IN2 → ESP32 DIR2 (Left)
- ENB → ESP32 PWM (Right)
- IN3 → ESP32 DIR1 (Right)
- IN4 → ESP32 DIR2 (Right)

### For TB6612FNG:
- PWMA → ESP32 PWM (Left)
- AIN1 → ESP32 DIR1 (Left)
- AIN2 → ESP32 DIR2 (Left)
- PWMB → ESP32 PWM (Right)
- BIN1 → ESP32 DIR1 (Right)
- BIN2 → ESP32 DIR2 (Right)
- STBY → 5V (always enabled)

## Troubleshooting

### Motors not spinning
- Check motor driver power supply
- Verify GPIO connections
- Check serial monitor for received commands

### Encoders not counting
- Verify encoder wiring (A, B, GND, VCC)
- Check pull-up resistors (internal pull-ups enabled in code)
- Monitor encoder values via Serial

### Erratic behavior
- Check power supply stability
- Reduce PID gains
- Verify encoder connections

### Serial communication issues
- Baud rate: 115200
- Ensure ESP32 and Pi share common ground
- Check USB cable quality

## Testing

### Test Motors (without encoders)

Add to `setup()`:
```cpp
setMotorSpeed(PWM_CHANNEL_LEFT, LEFT_DIR1_PIN, LEFT_DIR2_PIN, 100);
setMotorSpeed(PWM_CHANNEL_RIGHT, RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, 100);
delay(2000);
stopMotors();
```

### Test Encoders

Monitor serial output and manually rotate wheels. You should see tick counts changing.

## License

Apache 2.0
