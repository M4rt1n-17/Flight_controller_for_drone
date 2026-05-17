# Arduino Quadcopter — Flight Controller + Transmitter

A complete DIY quadcopter build running on Arduino: an "X" configuration flight controller with a 250 Hz cascaded PID loop, plus a matching handheld transmitter that streams stick inputs over nRF24L01 at 50 Hz.

This repo contains **two sketches**:

- **Flight Controller** — runs on the drone, handles IMU fusion, PID, motor mixing, and ESC output.
- **Transmitter** — runs on a separate handheld Arduino, reads analog sticks and sends control packets.

---

# Part 1 — Flight Controller

## Features

- **250 Hz fixed-rate control loop** with microsecond-accurate scheduling
- **Cascaded PID** — outer loop controls angle (deg), inner loop controls rate (deg/s)
- **MPU6500 IMU** with auto gyro bias and accelerometer scale calibration on boot
- **Complementary filter** attitude estimation with adaptive accel trust (still vs. flying, high-G rejection)
- **nRF24L01 radio link** at 250 kbps with failsafe latch and hysteresis
- **Software-synchronous ESC PWM** — all four pulses rise together via direct PORTD writes for low jitter
- **D-on-measurement** to avoid setpoint kick, with separate D-term cutoffs for roll/pitch and yaw
- **Anti-windup** integrator clamps, gated by throttle level
- **Motor mix desaturation** — shifts and scales outputs to preserve control authority near saturation
- **Airmode idle** keeps motors spinning above a floor for stable control at low throttle
- **Stick-combo arm/disarm** (throttle low + sticks centered, hold 1.5 s)
- **Spin-hold after arming** until pilot raises throttle, as a pre-flight check
- **Liftoff I-term reset** to prevent integrator buildup before takeoff
- **Serial debug HUD** with attitude, setpoints, gyro rates, motor outputs, and event log

## Hardware

| Component | Notes |
|---|---|
| Arduino (UNO/Nano, ATmega328P) | Code uses direct PORTD writes — pins 3–6 must be on PORTD |
| MPU6500 IMU | I²C @ 400 kHz, address `0x68` |
| nRF24L01 radio | Hardware SPI, CE = D9, CSN = D8 |
| 4× ESCs | Standard 1000–2000 µs PWM input |
| Brushless motors, frame, LiPo, etc. | Standard quadcopter build |

### Pin Map

| Function | Pin |
|---|---|
| ESC Front-Left | D4 |
| ESC Front-Right | D6 |
| ESC Rear-Left | D5 |
| ESC Rear-Right | D3 |
| nRF24 CE | D9 |
| nRF24 CSN | D8 |
| SPI MOSI / MISO / SCK | D11 / D12 / D13 |
| I²C SDA / SCL | A4 / A5 |

### Motor Layout & Spin Directions

```
   FL (D4, CW)    FR (D6, CCW)
            \   /
             \ /
              X
             / \
            /   \
   RL (D5, CCW)  RR (D3, CW)
```

## Dependencies

Install via Arduino Library Manager:

- `RF24` by TMRh20
- `FastIMU` by LiquidCGS
- `Wire` and `SPI` (bundled with the Arduino IDE)

## Setup

1. Wire the hardware as in the pin map. **All four ESC pins must be on PORTD (digital pins 0–7).**
2. Place the quad on a level surface and power up. The IMU calibrates gyro bias and accelerometer scale automatically — keep it still for the first few seconds.
3. Open Serial Monitor at **115200 baud** to see the debug HUD.
4. Set `ROLL_TRIM_DEG` and `PITCH_TRIM_DEG` in the source: arm on a flat surface, read the `R=` and `P=` values from the HUD, and enter them with **opposite sign** so the FC reads 0° when level.

## Arming

- Throttle to minimum, all other sticks centered.
- Hold for **1.5 seconds** to toggle arm state.
- After arming, motors spin at a gentle idle until throttle is raised above ~1150 µs.

## Tuning

The default gains are a conservative starting point. Standard tuning order:

1. **Rate P** — raise until fast oscillation, then back off ~30%.
2. **Rate D** — raise to damp residual wobble.
3. **Rate I** — raise last, just enough to eliminate hover drift.
4. **Angle P / I** — tune outer loop after rates feel solid.

All gains are `#define`s at the top of the file.

---

# Part 2 — Transmitter

The handheld controller that talks to the flight controller above. Reads three analog gimbal axes, applies deadzone and expo, and streams 50 Hz packets over nRF24L01.

## Features

- **50 Hz fixed-rate transmission** to match the flight controller's expected packet rate
- **Auto-calibration** on power-up — sticks are sampled at boot to learn their center positions
- **Configurable expo curves** for both throttle and roll/pitch — soft response near center, full authority at endpoints
- **Deadzone** around stick centers to eliminate noise from cheap potentiometers
- **One-sided throttle mapping** — throttle stick below center stays at 0, above center scales 0–2000
- **Pitch inversion flag** for mounting flexibility
- **Serial debug output** at 10 Hz showing transmitted values and ACK status

## Hardware

| Component | Notes |
|---|---|
| Arduino (UNO/Nano, ATmega328P) | Any AVR Arduino works |
| nRF24L01 radio | Hardware SPI, CE = D7, CSN = D8 |
| 3× potentiometers / gimbal pots | Throttle, roll, pitch |
| Battery, enclosure, switches | Standard handheld build |

### Pin Map

| Function | Pin |
|---|---|
| Throttle pot | A2 |
| Roll pot | A3 |
| Pitch pot | A4 |
| nRF24 CE | D7 |
| nRF24 CSN | D8 |
| SPI MOSI / MISO / SCK | D11 / D12 / D13 |

## Setup

1. Wire the gimbal pots between 5 V and GND, with wipers on A2/A3/A4.
2. Power up with sticks centered — the transmitter samples them for ~3 seconds to set zero points.
3. Open Serial Monitor at **115200 baud** to verify packets are sending and ACKs are returning.

## Tuning

All transmitter tuning constants live at the top of the source file:

| Define | Default | Purpose |
|---|---|---|
| `EXPO_ROLL_PITCH` | `0.40` | Cubic expo amount for roll/pitch (0 = linear, 1 = full cubic) |
| `EXPO_THROTTLE` | `0.25` | Expo amount for throttle |
| `DEADZONE` | `25` | Stick units around center treated as zero |
| `TX_HZ` | `50` | Transmit rate in Hz |
| `INVERT_PITCH` | `true` | Flip pitch axis if it feels backwards |

A good starting expo for hover practice is **0.3–0.5** on roll/pitch. Higher values give softer hover response but require more stick travel for sharp maneuvers.

## Calibration

Stick centers are learned **every time** the transmitter powers up. Hold sticks centered for the first ~3 seconds after boot. If they drift later, just power-cycle.

---

# Shared — Radio Link

Both sketches must agree on the radio link parameters or no packets will get through.

| Parameter | Value |
|---|---|
| Address | `"00001"` |
| Data rate | `250 kbps` |
| PA level | `HIGH` |
| Packet rate | 50 Hz |

### Packet Format

The `ControllerData` struct must be **byte-for-byte identical** on both sides:

```cpp
struct ControllerData {
  int throttle;   // 0..2000
  int roll;       // -500..500
  int pitch;      // -500..500
  int yaw;        // -500..500   (unused in this build — no yaw stick)
};
```

---

## Safety

- Always test with **propellers off** first.
- The failsafe cuts motors after 1 second of radio loss and re-arms only after 200 ms of clean signal.
- This is a hobby project, not certified flight hardware. Fly at your own risk.

## License

MIT — see `LICENSE` for details.

## Contributing

Pull requests, bug reports, and tuning logs welcome. If you fly this code, share your gains and frame specs in an issue!
