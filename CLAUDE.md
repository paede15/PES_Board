# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded robotics platform for the **PES Board** (ZHAW course project), targeting the **STM32 Nucleo F446RE** microcontroller running **Mbed OS**. The current application implements a differential-drive robot that follows a black line, detects colored parcels at crossings, and performs pickup/drop routines.

## Build & Flash

**Build system:** PlatformIO (not Mbed Studio — uses gcc, not armclang)

```bash
# Build
pio run

# Flash (copy firmware to mounted board)
cp .pio/build/nucleo_f446re/firmware.bin /media/$USER/NOD_F446RE

# Serial monitor (115200 baud)
cu -l /dev/ttyACM0 -s 115200
```

There are no automated tests. Debugging is done via `printf()` over the serial connection.

## Architecture

### Entry Point

`src/main.cpp` is the sole application file. It follows a fixed pattern:
1. Object construction (motors, sensors, etc.)
2. `while(true)` loop timed to `main_task_period_ms` (default 20 ms = 50 Hz)
3. Blue button (`BUTTON1`) toggles `do_execute_main_task`; inside the loop, a `switch` on `robot_state` drives behavior

### State Machine (current application)

```
INITIAL → FORWARD ⇄ COLOR_SCAN → PICKUP_PARCEL → FORWARD
                                                ↘ DROP_PARCEL (stub)
```

A background `Thread` (`detect_cross_thread`) polls `SensorBar::getRaw()` every 4 ms and sets `cross_detected = true` when ≥3 center LEDs are active, triggering the `COLOR_SCAN` state.

### Library Layout (`lib/`)

All drivers live in `lib/<Name>/`. Each follows the standard PlatformIO layout with headers alongside source. Key drivers:

| Driver | Purpose |
|--------|---------|
| `DCMotor` | Velocity-controlled DC motor using `EncoderCounter` + PID; call `setVelocity(rot/s)` |
| `SensorBar` | 8-LED optical line sensor; `getAvgAngleRad()`, `getRaw()`, `isAnyLedActive()` |
| `ColorSensor` | TCS3200-style frequency-output color sensor; `getColor()` returns an int |
| `LineFollower` | High-level line following combining `SensorBar` + `DCMotor` |
| `Motion` | Motion profiler (trapezoidal) for `DCMotor` |
| `IMU` / `LSM9DS1` | 9-axis IMU over I2C |
| `Servo` | Servo control via PWM |
| `IRSensor` | Analog IR distance sensor |
| `UltrasonicSensor` | HC-SR04 ultrasonic sensor |
| `SDLogger` / `SDWriter` | SD card data logging over SPI |
| `SerialStream` / `SerialPipe` | High-speed serial data streaming |
| `PIDCntrl` | Generic discrete PID controller |
| `IIRFilter`, `AvgFilter`, `MedianFilter3` | Signal filters |
| `ThreadFlag`, `RealTimeThread` | Mbed RTOS threading utilities |
| `eigen-lib` | Eigen linear algebra (header-only, vectorization disabled) |

### Pin Mapping

All hardware pins are defined in `include/PESBoardPinMap.h`. Always use the symbolic names (e.g., `PB_PWM_M1`, `PB_ENC_A_M1`, `PB_ENABLE_DCMOTORS`) rather than raw pin names. The header supports two board versions via a preprocessor define.

### Kinematics Pattern

The application uses Eigen 2×2 matrices for differential-drive kinematics:
- Build `Cwheel2robot` from wheel radius (`r_wheel`) and wheelbase (`b_wheel`)
- `robot_coord = {v_forward, omega}` → `wheel_speed = Cwheel2robot.inverse() * robot_coord`
- Pass wheel speeds to `motor.setVelocity()` (units: rotations/second)

## Reference Solutions

Complete working examples for all major features are in `docs/solutions/`:
- `main_line_follower.cpp` — full line follower
- `main_dd_kinematic_calib.cpp` — differential drive calibration
- `main_gimbal.cpp` — IMU + servo gimbal
- `main_sd_card_logger.cpp` — SD card logging
- `main_comp_filter.cpp` — complementary filter / Mahony

## Code Style

`.clang-format` is present — use it. Inline comments in the codebase use `//` with a space. Variable and object names follow `snake_case`. Constants use `snake_case` with `const`.
