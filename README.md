# Arduino Quadcopter Flight Controller

A DIY flight controller for an "X" configuration quadcopter, running on an Arduino with a 250 Hz control loop, cascaded PID, and software-generated ESC pulses.

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
2. Pair the transmitter to use address `"00001"` and the `ControllerData` packet structure:
   ```cpp
   struct ControllerData {
     int throttle;   // 0..2000
     int roll;       // -500..500
     int pitch;      // -500..500
     int yaw;        // -500..500
   };
   ```
3. Place the quad on a level surface and power up. The IMU calibrates gyro bias and accelerometer scale automatically — keep it still for the first few seconds.
4. Open Serial Monitor at **115200 baud** to see the debug HUD.
5. Set `ROLL_TRIM_DEG` and `PITCH_TRIM_DEG` in the source: arm on a flat surface, read the `R=` and `P=` values from the HUD, and enter them with **opposite sign** so the FC reads 0° when level.

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

## Safety

- Always test with **propellers off** first.
- The failsafe cuts motors after 1 second of radio loss and re-arms only after 200 ms of clean signal.
- This is a hobby project, not certified flight hardware. Fly at your own risk.

## License

MIT — see `LICENSE` for details.

## Contributing

Pull requests, bug reports, and tuning logs welcome. If you fly this code, share your gains and frame specs in an issue!
