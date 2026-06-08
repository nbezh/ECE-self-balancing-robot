# Self-Balancing Robot (ESP32)

A two-wheeled self-balancing robot built on an ESP32. The robot keeps itself
upright with a complementary-filter angle estimate feeding a PID controller,
and exposes a WiFi access-point web page for live gain tuning. The firmware is
structured as a set of FreeRTOS tasks.

## Hardware

| Component        | Part                          |
|------------------|-------------------------------|
| MCU              | ESP32 DevKit (WROOM)          |
| Motor driver     | TB6612FNG (dual H-bridge)     |
| IMU              | MPU6050 (GY-521)              |
| Distance sensor  | HC-SR04 ultrasonic            |
| Motors           | TT gearmotors (2x)            |
| Power            | 8 V pack -> 5 V buck converter|

The IMU is mounted vertically between the wheels; the battery sits high, which
raises the centre of mass and makes the robot tip more slowly (easier to
control).

### Pin map

| Signal            | GPIO | Notes                                        |
|-------------------|------|----------------------------------------------|
| TB6612 PWMA       | 4    | Motor A speed                                |
| TB6612 AIN2       | 16   | Motor A direction                            |
| TB6612 AIN1       | 17   | Motor A direction                            |
| TB6612 BIN1       | 18   | Motor B direction                            |
| TB6612 BIN2       | 19   | Motor B direction                            |
| TB6612 PWMB       | 23   | Motor B speed                                |
| TB6612 STBY       | 5    | Driver enable. **10 kΩ pulldown to GND**     |
| MPU6050 SDA       | 21   | I2C (addr 0x68, AD0 -> GND)                  |
| MPU6050 SCL       | 22   | I2C                                          |
| HC-SR04 TRIG      | 26   | Direct                                       |
| HC-SR04 ECHO      | 25   | **Via 1 kΩ / 2 kΩ divider (5 V -> 3.3 V)**   |

GPIO5 is a strapping pin that boots HIGH, so the pulldown keeps the motor
driver disabled at power-up until firmware enables it.

### Power

`8 V -> buck -> 5 V` rail feeds the ESP32 `VIN`, the TB6612 `VM` (motor power),
and the HC-SR04. The ESP32's onboard 3.3 V regulator feeds the TB6612 `VCC`
(logic) and the MPU6050. All grounds are common. A 470–1000 µF bulk capacitor
on the 5 V rail absorbs motor current spikes so the ESP32 doesn't brown out.

> Set the buck to exactly 5.0 V (meter it with the output disconnected) before
> wiring it to the ESP32. Don't power from USB and the buck simultaneously.

## Software

### Build

PlatformIO project. `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
lib_deps =
    ESP32Async/ESPAsyncWebServer
    ESP32Async/AsyncTCP
build_flags =
    -D CONFIG_ASYNC_TCP_RUNNING_CORE=0
    -D CONFIG_ASYNC_TCP_STACK_SIZE=16384
```

The PWM layer auto-selects the correct LEDC API for ESP32 Arduino core 2.x or
3.x, so it builds on either.

### Run

1. Flash `src/main.cpp`.
2. Connect a phone/laptop to the WiFi network **`BalanceBot`** (password
   `balance123`).
3. Open `http://192.168.4.1`.
4. The robot boots **STOPPED**. Hold it near upright, press **GO** to arm.

The page shows a live angle graph and the latest ultrasonic distance, plus
sliders for Kp, Ki, Kd, balance offset, and the motor deadband. **Save to
flash** persists the current gains (NVS).

## RTOS architecture

The ESP32 Arduino core runs on FreeRTOS, so the firmware is split into three
tasks with explicit priorities and core affinity rather than a single
`loop()`. This separates the one hard-real-time activity (balancing) from the
soft-real-time ones (networking, ranging).

| Task           | Core | Priority | Rate    | Job                                            |
|----------------|------|----------|---------|------------------------------------------------|
| `controlTask`  | 1    | 3 (high) | 200 Hz  | MPU read → complementary filter → PID → motors |
| `commsTask`    | 0    | 1        | 25 Hz   | Push telemetry to the web page                 |
| `distanceTask` | 0    | 1        | 10 Hz   | Read HC-SR04 (`pulseIn`)                        |

**Deterministic control timing.** `controlTask` uses `vTaskDelayUntil`, which
schedules each iteration on a fixed period instead of "delay N ms after the
work finishes." This gives a constant control interval (`dt = 5 ms`), so the
filter and PID math use a fixed timestep instead of a measured, jittery one.

**Core isolation.** The control task is pinned alone to core 1. The WiFi stack
and the other two tasks live on core 0. Nothing on core 0 can steal time from
the balance loop.

**Why `distanceTask` exists.** `pulseIn()` is a *blocking* call that can stall
for up to 30 ms waiting for the ultrasonic echo. In a single-loop design that
stall would wreck the 200 Hz control timing. Isolating it in its own
low-priority task on core 0 means the blocking read can never disturb
balancing — a concrete demonstration of why task separation matters.

**Synchronization.** All shared data (gains, current angle, run flag, distance)
lives in one `State` struct guarded by a mutex (`stMutex`). Each task takes the
mutex only long enough to *snapshot* values into local variables, then releases
it before doing slow work (motor writes, `ws.textAll()`, `pulseIn`). Critical
sections stay short, so no task blocks another, and there is no nested locking
(no deadlock risk).

```
                 +---------------------+
   web sliders → |  stMutex-guarded    | → controlTask reads gains
                 |  State struct       |
   controlTask → |  (gains, angle,     | → commsTask reads angle/dist
   distanceTask →|   run, distance)    |
                 +---------------------+
```

## Tuning

1. Set the **Offset** to the robot's true balance point: with motors stopped,
   find the angle where it balances on its own on the wheels, read that value
   off the page, and enter it. Usually 1–3°.
2. Set **Min PWM** to the motor deadband — the smallest PWM that actually turns
   the wheels (TT motors ignore small commands below this, so corrections do
   nothing without it).
3. Raise **Kp** until the robot makes firm, quick corrections; add **Kd**
   (~1.0) to damp overshoot; add a little **Ki** last, only if it holds a
   steady lean.

## Known limitations

TT (yellow) gearmotors are marginal for balancing: they have a deadband, gear
backlash, and limited top speed. The deadband compensation helps, but these
motors may set a hard ceiling on how well any tuning can balance the robot.
