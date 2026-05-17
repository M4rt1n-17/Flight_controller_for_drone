#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include "FastIMU.h"
#include <math.h>



// ========================== PIN MAP ==========================
// Radio  — nRF24L01 on hardware SPI (D11 MOSI, D12 MISO, D13 SCK)
#define CE_PIN   9
#define CSN_PIN  8

// ESC signal pins — ALL on PORTD for single-cycle parallel writes
// Verify these match your wiring!
#define ESC_PIN_FL  4   // Front-Left
#define ESC_PIN_FR  6   // Front-Right
#define ESC_PIN_RL  5   // Rear-Left
#define ESC_PIN_RR  3   // Rear-Right

// Bitmask for simultaneous HIGH/LOW (pins 3-6 are bits 3-6 of PORTD)
#define ESC_PORTD_MASK  ((1 << ESC_PIN_FL) | (1 << ESC_PIN_FR) | \
                         (1 << ESC_PIN_RL) | (1 << ESC_PIN_RR))

// ========================== ESC RANGE ==========================
#define RX_THROTTLE_MIN  0
#define RX_THROTTLE_MAX  2000

#define ESC_MIN_US   1000
#define ESC_MAX_US   2000

// Arming
#define ARM_THROTTLE_US  1050
#define THROTTLE_LOW_US  1020

// Minimum motor speed when armed (airmode idle)
#define MOTOR_IDLE_US    1060

#define RADIO_TIMEOUT    1000   // ms before failsafe trips

// ========================== FAILSAFE ==========================
#define FAILSAFE_CLEAR_MS  200  // stable RX for this long to exit failsafe

// ========================== LOOP ==========================
#define LOOP_HZ   250
#define LOOP_US   (1000000UL / LOOP_HZ)

#define DT_MIN    0.001f
#define DT_MAX    0.020f

// ========================== DEBUG HUD ==========================
#define DEBUG_HUD_ENABLE       1
#define DEBUG_HUD_INTERVAL_MS  200

// ========================== LIMITS ==========================
#define MAX_ANGLE_DEG   30.0f
#define MAX_RATE_DPS    300.0f
#define MAX_YAW_RATE_DPS 200.0f

// Integrator clamps
#define ANGLE_I_LIMIT   60.0f
#define RATE_I_LIMIT    80.0f

// ========================== IMU TRIM ==========================
// Compensate for IMU not being perfectly level on the frame.
// Measure: arm, sticks centered, read R= and P= from HUD at idle.
// Enter those values here (sign included) so the FC sees "0" when level.
#define ROLL_TRIM_DEG    1.8f
#define PITCH_TRIM_DEG   2.0f

// ========================== FILTER CUTOFFS (Hz) ==========================
#define GYRO_LPF_CUTOFF_HZ   60.0f   // gyro noise rejection
#define SP_LPF_CUTOFF_HZ     15.0f   // stick input smoothing
#define DTERM_CUTOFF_HZ_RP   40.0f   // D-term roll/pitch
#define DTERM_CUTOFF_HZ_Y    25.0f   // D-term yaw

// Complementary filter — accel correction bandwidth
#define COMP_FC_STILL_HZ   2.0f    // on the ground / hovering still
#define COMP_FC_FLY_HZ     0.8f    // in dynamic flight

// Stillness detection
#define STILL_GYRO_DPS     2.0f
#define STILL_ACCEL_G      0.08f

// ========================== PID GAINS ==========================
// --- Outer loop: angle (deg) → desired rate (deg/s) ---
#define KP_ANGLE_ROLL   4.5f
#define KI_ANGLE_ROLL   0.8f
#define KP_ANGLE_PITCH  4.5f
#define KI_ANGLE_PITCH  0.8f

// --- Inner loop: rate error (deg/s) → ESC correction (us) ---
//  Start conservative. Increase P until you see fast oscillation,
//  then back off 30 %. Raise D to damp remaining wobble.
//  Add I last until hover drift is gone.
#define KP_RATE_ROLL    0.90f
#define KI_RATE_ROLL    0.35f
#define KD_RATE_ROLL    0.014f

#define KP_RATE_PITCH   KP_RATE_ROLL
#define KI_RATE_PITCH   KI_RATE_ROLL
#define KD_RATE_PITCH   KD_RATE_ROLL

#define KP_RATE_YAW     1.80f
#define KI_RATE_YAW     0.40f
#define KD_RATE_YAW     0.0f

#define PID_OUT_LIMIT   300.0f

// ========================== STICK TUNING ==========================
#define STICK_DB          10     // deadband on stick raw value
#define ARM_STICK_THRESH  20     // sticks must be within this to arm

// ========================== MOTOR MIX ==========================
//  Motor spin directions (viewed from above):
//    FL (D4) = CW      FR (D6) = CCW
//    RL (D5) = CCW     RR (D3) = CW
//
//  Standard "X" quad mix (NED body frame, + pitch = nose down):
//    FL = throttle - pitch + roll - yaw   (CW  → negative yaw torque)
//    FR = throttle - pitch - roll + yaw   (CCW → positive yaw torque)
//    RL = throttle + pitch + roll + yaw   (CCW → positive yaw torque)
//    RR = throttle + pitch - roll - yaw   (CW  → negative yaw torque)
//
//  If your yaw is backwards, flip this define.
#define INVERT_YAW_MIX  1

// ========================== SPIN-UP HOLD ==========================
//  After arming, hold all motors at a fixed gentle spin until
//  the pilot raises throttle. Gives time to verify before flight.
#define ENABLE_ARM_SPIN_HOLD     1
#define ARM_SPIN_HOLD_US         1100
#define ARM_SPIN_EXIT_THR_US     1150

// =================================================================
//                       GLOBAL OBJECTS
// =================================================================
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

MPU6500 IMU;

calData calibration = {0};
AccelData accel;
GyroData gyro;

// ========================== STATE ==========================
unsigned long lastRadioTime = 0;
uint32_t lastLoopTick      = 0;
bool     armed             = false;
bool     armedPrev         = false;
bool     spinHoldActive    = false;
bool     hasLifted         = false;  // liftoff I-term reset trigger

// Failsafe
bool     failsafeLatched   = false;
uint32_t fsClearReadyAt    = 0;

// Sensor calibration
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
#define ENABLE_ACCEL_SCALE 1
float accelScale = 1.0f;

// Attitude (degrees)
float roll  = 0, pitch = 0;

// Filtered gyro rates for control (deg/s)
float gyroRollRate_f  = 0;
float gyroPitchRate_f = 0;
float gyroYawRate_f   = 0;

// Pilot setpoints (filtered)
float rollSetpoint    = 0;
float pitchSetpoint   = 0;
float yawRateSetpoint = 0;

// Outer loop outputs (desired rates)
float rollRateSetpoint  = 0;
float pitchRateSetpoint = 0;

// Integrators — outer
float rollAngleISum  = 0;
float pitchAngleISum = 0;

// Integrators — inner
float rollRateISum  = 0, pitchRateISum = 0, yawRateISum = 0;

// D-term on measurement state
float prevRollRate  = 0, prevPitchRate  = 0, prevYawRate  = 0;
float dRollRate_f   = 0, dPitchRate_f   = 0, dYawRate_f   = 0;

// Motor output (microseconds) — written by mixer, sent by pulse generator
uint16_t motorUS[4] = {ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US};
// Index: 0=FL, 1=FR, 2=RL, 3=RR

// Debug
static uint32_t dbgLastHudMs       = 0;
static bool     dbgLastArmed       = false;
static bool     dbgLastFailsafe    = false;
static uint32_t dbgRadioPackets    = 0;
static uint32_t dbgLastRadioPackets = 0;

// ========================== RX DATA ==========================
struct ControllerData {
  int throttle;   // 0..2000
  int roll;       // -500..500
  int pitch;      // -500..500
  int yaw;        // -500..500
};
ControllerData rxData = {RX_THROTTLE_MIN, 0, 0, 0};

// =================================================================
//                       UTILITY FUNCTIONS
// =================================================================
static inline int applyDeadband(int v, int db) {
  return (abs(v) < db) ? 0 : v;
}

static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// PT1 (first-order low-pass) alpha from dt and cutoff frequency
static inline float pt1Alpha(float dt, float fc_hz) {
  if (fc_hz <= 0.0f) return 1.0f;
  const float tau = 1.0f / (6.2831853f * fc_hz);
  return dt / (dt + tau);
}

static void resetPidStates() {
  rollAngleISum = pitchAngleISum = 0;
  rollRateISum  = pitchRateISum  = yawRateISum = 0;
  prevRollRate  = prevPitchRate  = prevYawRate  = 0;
  dRollRate_f   = dPitchRate_f   = dYawRate_f   = 0;
}

void dbgEvent(const __FlashStringHelper* msg) {
#if DEBUG_HUD_ENABLE
  Serial.print(F("[EVT] "));
  Serial.println(msg);
#endif
}

// =================================================================
//       250 Hz SYNCHRONOUS ESC PULSE GENERATOR
// =================================================================
//  Replaces the Arduino Servo library (50 Hz) with direct port
//  manipulation that fires all 4 pulses at the PID loop rate.
//
//  How it works:
//    1. Set all 4 ESC pins HIGH simultaneously via PORTD.
//    2. Sort the 4 pulse widths so we can drop them LOW in order.
//    3. Busy-wait on micros() for each successive falling edge.
//
//  Worst-case blocking time: ~2 ms (at max throttle).
//  Loop budget at 250 Hz is 4 ms, so ~2 ms remains for
//  IMU + radio + PID which easily fits on a 16 MHz AVR.
//
//  Jitter: micros() on AVR has ~4 µs resolution → 0.2-0.4 % of
//  pulse width. Perfectly fine for brushless ESCs.
// =================================================================

// Pin-to-PORTD-bit lookup (pins 3-6 only)
static const uint8_t escBit[4] = {
  (uint8_t)(1 << ESC_PIN_FL),   // index 0 = FL
  (uint8_t)(1 << ESC_PIN_FR),   // index 1 = FR
  (uint8_t)(1 << ESC_PIN_RL),   // index 2 = RL
  (uint8_t)(1 << ESC_PIN_RR)    // index 3 = RR
};

void writeESCPulses() {
  // Constrain all motor values
  for (uint8_t i = 0; i < 4; i++) {
    if (motorUS[i] < ESC_MIN_US) motorUS[i] = ESC_MIN_US;
    if (motorUS[i] > ESC_MAX_US) motorUS[i] = ESC_MAX_US;
  }

  // Build sorted index array (ascending pulse width)
  uint8_t order[4] = {0, 1, 2, 3};
  // Insertion sort — only 4 elements, very fast
  for (uint8_t i = 1; i < 4; i++) {
    uint8_t key = order[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && motorUS[order[j]] > motorUS[key]) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = key;
  }

  // All ESC pins HIGH at the same instant
  noInterrupts();
  PORTD |= ESC_PORTD_MASK;
  interrupts();

  uint32_t start = micros();

  // Drop each pin LOW at its target time, shortest first
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t target = motorUS[order[i]];
    while ((uint32_t)(micros() - start) < target) { /* spin */ }

    // Set this motor's pin LOW (others stay HIGH)
    noInterrupts();
    PORTD &= ~escBit[order[i]];
    interrupts();
  }
}

void setAllMotors(uint16_t us) {
  motorUS[0] = motorUS[1] = motorUS[2] = motorUS[3] = us;
  writeESCPulses();
}

// =================================================================
//                       PROTOTYPES
// =================================================================
void readRadioCommands(float dt);
void updateAttitude(float dt, float gyroR, float gyroP);
void calibrateGyroBias();
void calibrateAccelScale();

float angleToRate(float angleSp, float angleMeas,
                  float *iSum, float kp, float ki,
                  float dt, bool allowI);

float ratePID(float rateSp, float rateMeas,
              float *iSum, float *prevMeas, float *dState,
              float kp, float ki, float kd, float dt,
              bool allowI, float d_fc_hz);

void motorMix(int escBase, float rollOut, float pitchOut, float yawOut);

void dbgHud(uint32_t nowMs, bool failsafeNow, int escBase, float dt);

// =================================================================
//                          SETUP
// =================================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("FC Starting — 250 Hz loop, 250 Hz ESC output"));

  // ---- I2C ----
  Wire.begin();
  Wire.setClock(400000);

  // ---- IMU ----
  Serial.println(F("Init MPU6500..."));
  int err = IMU.init(calibration, 0x68);
  if (err != 0) {
    Serial.print(F("MPU6500 error: ")); Serial.println(err);
    while (true) {}
  }
  Serial.println(F("MPU6500 OK."));

  // ---- Calibration ----
  Serial.println(F("Gyro cal — hold still..."));
  calibrateGyroBias();
#if ENABLE_ACCEL_SCALE
  Serial.println(F("Accel scale cal..."));
  calibrateAccelScale();
#endif

  // ---- ESC pins as OUTPUT ----
  pinMode(ESC_PIN_FL, OUTPUT);
  pinMode(ESC_PIN_FR, OUTPUT);
  pinMode(ESC_PIN_RL, OUTPUT);
  pinMode(ESC_PIN_RR, OUTPUT);

  // ---- ESC initialization (send min throttle for 2 s) ----
  Serial.println(F("ESC init — sending 1000 us for 2 s..."));
  uint32_t escInitEnd = millis() + 2000;
  while (millis() < escInitEnd) {
    setAllMotors(ESC_MIN_US);
    delay(4);
  }

  // ---- Radio ----
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();

  // SPI master pin
  pinMode(10, OUTPUT);

  // ---- Timers ----
  lastLoopTick = micros();
  lastRadioTime = millis();
  dbgLastArmed = armed;

  Serial.println(F("Setup complete. Waiting for radio..."));
}

// =================================================================
//                         MAIN LOOP
// =================================================================
void loop() {
  // ---- Fixed-rate scheduler ----
  while ((uint32_t)(micros() - lastLoopTick) < LOOP_US) {}
  uint32_t nowTick = micros();
  float dt = (float)(nowTick - lastLoopTick) * 1e-6f;
  dt = clampf(dt, DT_MIN, DT_MAX);
  lastLoopTick += LOOP_US;

  // ---- Radio ----
  readRadioCommands(dt);
  uint32_t nowMs = millis();

  // ---- IMU ----
  IMU.update();
  IMU.getAccel(&accel);
  IMU.getGyro(&gyro);

  float gx = gyro.gyroX - gyroBiasX;
  float gy = gyro.gyroY - gyroBiasY;
  float gz = gyro.gyroZ - gyroBiasZ;

#if ENABLE_ACCEL_SCALE
  accel.accelX *= accelScale;
  accel.accelY *= accelScale;
  accel.accelZ *= accelScale;
#endif

  // ---- Gyro LPF ----
  float aG = pt1Alpha(dt, GYRO_LPF_CUTOFF_HZ);
  gyroRollRate_f  += aG * (gx - gyroRollRate_f);
  gyroPitchRate_f += aG * (gy - gyroPitchRate_f);
  gyroYawRate_f   += aG * (gz - gyroYawRate_f);

  // ---- Attitude ----
  updateAttitude(dt, gyroRollRate_f, gyroPitchRate_f);

  // ---- Throttle mapping ----
  int escBase = map(rxData.throttle, RX_THROTTLE_MIN, RX_THROTTLE_MAX,
                    ESC_MIN_US, ESC_MAX_US);
  escBase = constrain(escBase, ESC_MIN_US, ESC_MAX_US);

  // ======================== FAILSAFE ========================
  uint32_t rxAge = (uint32_t)(nowMs - lastRadioTime);
  if (rxAge > RADIO_TIMEOUT) {
    failsafeLatched = true;
    fsClearReadyAt = 0;
  } else if (failsafeLatched) {
    if (fsClearReadyAt == 0) fsClearReadyAt = nowMs;
    if ((uint32_t)(nowMs - fsClearReadyAt) >= FAILSAFE_CLEAR_MS) {
      failsafeLatched = false;
    }
  }

  bool failsafeNow = failsafeLatched;

#if DEBUG_HUD_ENABLE
  if (failsafeNow != dbgLastFailsafe) {
    dbgLastFailsafe = failsafeNow;
    dbgEvent(failsafeNow ? F("FAILSAFE ON") : F("FAILSAFE OFF"));
  }
#endif

  // ======================== SPIN HOLD EDGE ========================
#if ENABLE_ARM_SPIN_HOLD
  if (armed && !armedPrev) {
    spinHoldActive = true;
    dbgEvent(F("SPIN HOLD ON"));
  }
#endif
  armedPrev = armed;
  if (!armed || failsafeNow) spinHoldActive = false;

  // ======================== FAILSAFE → CUT ========================
  if (failsafeNow) {
    armed = false;
    spinHoldActive = false;
    setAllMotors(ESC_MIN_US);
    resetPidStates();
    dbgHud(nowMs, true, escBase, dt);
    return;
  }

  // ======================== DISARMED ========================
  if (!armed) {
    setAllMotors(ESC_MIN_US);
    resetPidStates();
    dbgHud(nowMs, false, escBase, dt);
    return;
  }

  // ======================== SPIN HOLD ========================
#if ENABLE_ARM_SPIN_HOLD
  if (spinHoldActive) {
    if (escBase > ARM_SPIN_EXIT_THR_US) {
      spinHoldActive = false;
      dbgEvent(F("SPIN HOLD OFF — throttle raised"));
    } else {
      setAllMotors(ARM_SPIN_HOLD_US);
      resetPidStates();
      dbgHud(nowMs, false, escBase, dt);
      return;
    }
  }
#endif

  // ======================== AIRMODE IDLE ========================
  //  Instead of cutting motors when throttle is at minimum,
  //  maintain a gentle idle spin and keep PID active.
  //  This allows attitude correction during descents.
  if (escBase < MOTOR_IDLE_US) {
    escBase = MOTOR_IDLE_US;
  }

  // ======================== LIFTOFF I-RESET ========================
  //  The integrator accumulates error while sitting on the ground
  //  (the motors can't physically correct the tilt). The moment
  //  throttle crosses a liftoff threshold, zero all integrators
  //  so the quad starts clean without a stored windup bias.
  if (!hasLifted && escBase > 1350) {
    hasLifted = true;
    resetPidStates();
    dbgEvent(F("LIFTOFF I-RESET"));
  }
  if (escBase <= MOTOR_IDLE_US + 20) {
    hasLifted = false;
  }

  // ======================== CASCADED PID ========================
  rollSetpoint  = clampf(rollSetpoint,  -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  pitchSetpoint = clampf(pitchSetpoint, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);

  // Allow integrators only when there's meaningful throttle
  bool allowAngleI = (escBase > (MOTOR_IDLE_US + 30));
  bool allowRateI  = (escBase > (MOTOR_IDLE_US + 50));

  // --- Outer: angle → rate setpoint ---
  rollRateSetpoint  = angleToRate(rollSetpoint,  roll,  &rollAngleISum,
                                  KP_ANGLE_ROLL,  KI_ANGLE_ROLL,  dt, allowAngleI);
  pitchRateSetpoint = angleToRate(pitchSetpoint, pitch, &pitchAngleISum,
                                  KP_ANGLE_PITCH, KI_ANGLE_PITCH, dt, allowAngleI);

  rollRateSetpoint  = clampf(rollRateSetpoint,  -MAX_RATE_DPS, MAX_RATE_DPS);
  pitchRateSetpoint = clampf(pitchRateSetpoint, -MAX_RATE_DPS, MAX_RATE_DPS);
  yawRateSetpoint   = clampf(yawRateSetpoint,   -MAX_YAW_RATE_DPS, MAX_YAW_RATE_DPS);

  // --- Inner: rate PID with D-on-measurement ---
  float rollOut  = ratePID(rollRateSetpoint,  gyroRollRate_f,
                           &rollRateISum, &prevRollRate, &dRollRate_f,
                           KP_RATE_ROLL, KI_RATE_ROLL, KD_RATE_ROLL,
                           dt, allowRateI, DTERM_CUTOFF_HZ_RP);

  float pitchOut = ratePID(pitchRateSetpoint, gyroPitchRate_f,
                           &pitchRateISum, &prevPitchRate, &dPitchRate_f,
                           KP_RATE_PITCH, KI_RATE_PITCH, KD_RATE_PITCH,
                           dt, allowRateI, DTERM_CUTOFF_HZ_RP);

  float yawOut   = ratePID(yawRateSetpoint,   gyroYawRate_f,
                           &yawRateISum, &prevYawRate, &dYawRate_f,
                           KP_RATE_YAW, KI_RATE_YAW, KD_RATE_YAW,
                           dt, allowRateI, DTERM_CUTOFF_HZ_Y);

  rollOut  = clampf(rollOut,  -PID_OUT_LIMIT, PID_OUT_LIMIT);
  pitchOut = clampf(pitchOut, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  yawOut   = clampf(yawOut,   -PID_OUT_LIMIT, PID_OUT_LIMIT);

  // ======================== MOTOR MIX + OUTPUT ========================
  motorMix(escBase, rollOut, pitchOut, yawOut);
  writeESCPulses();

  dbgHud(nowMs, false, escBase, dt);
}

// =================================================================
//                          RADIO
// =================================================================
void readRadioCommands(float dt) {
  ControllerData temp;
  bool gotPacket = false;

  while (radio.available()) {
    radio.read(&temp, sizeof(temp));
    gotPacket = true;
    dbgRadioPackets++;
  }
  if (!gotPacket) return;

  lastRadioTime = millis();

  // Throttle smoothing (prevents jumps from noisy ADC on TX)
  static float filtThrottle = 0.0f;
  filtThrottle += 0.15f * ((float)temp.throttle - filtThrottle);
  rxData.throttle = constrain((int)filtThrottle, RX_THROTTLE_MIN, RX_THROTTLE_MAX);

  rxData.roll  = constrain(temp.roll,  -500, 500);
  rxData.pitch = constrain(temp.pitch, -500, 500);
  rxData.yaw   = constrain(temp.yaw,   -500, 500);

  int r = applyDeadband(rxData.roll,  STICK_DB);
  int p = applyDeadband(rxData.pitch, STICK_DB);
  int y = applyDeadband(rxData.yaw,   STICK_DB);

  float rollSp_raw  = (r / 500.0f) * MAX_ANGLE_DEG;
  float pitchSp_raw = (p / 500.0f) * MAX_ANGLE_DEG;
  float yawSp_raw   = (y / 500.0f) * MAX_YAW_RATE_DPS;

  // PT1 smoothing on setpoints
  static float rSp = 0, pSp = 0, ySp = 0;
  float aS = pt1Alpha(dt, SP_LPF_CUTOFF_HZ);
  rSp += aS * (rollSp_raw  - rSp);
  pSp += aS * (pitchSp_raw - pSp);
  ySp += aS * (yawSp_raw   - ySp);

  rollSetpoint    = rSp;
  pitchSetpoint   = pSp;
  yawRateSetpoint = ySp;

  // ---- ARM / DISARM toggle ----
  static unsigned long stableLowStart = 0;
  static bool toggleUsed = false;
  const unsigned long TOGGLE_HOLD_MS = 1500;

  bool sticksCentered = (abs(rxData.roll)  < ARM_STICK_THRESH) &&
                        (abs(rxData.pitch) < ARM_STICK_THRESH) &&
                        (abs(rxData.yaw)   < ARM_STICK_THRESH);

  int escTmp = map(rxData.throttle, RX_THROTTLE_MIN, RX_THROTTLE_MAX,
                   ESC_MIN_US, ESC_MAX_US);
  bool throttleLow = (escTmp < THROTTLE_LOW_US);

  if (throttleLow && sticksCentered) {
    if (!toggleUsed) {
      if (stableLowStart == 0) stableLowStart = millis();
      if ((uint32_t)(millis() - stableLowStart) > TOGGLE_HOLD_MS) {
        armed = !armed;
        dbgEvent(armed ? F("ARMED") : F("DISARMED"));
        resetPidStates();
        toggleUsed = true;
        stableLowStart = 0;
      }
    }
  } else {
    stableLowStart = 0;
    toggleUsed = false;
  }
}

// =================================================================
//                    ATTITUDE ESTIMATION
// =================================================================
void updateAttitude(float dt, float gyroR, float gyroP) {
  const float ax = accel.accelX;
  const float ay = accel.accelY;
  const float az = accel.accelZ;

  const float accMag = sqrtf(ax * ax + ay * ay + az * az);

  // Safe denominators for atan2
  const float denomRoll = (fabsf(az) < 1e-6f)
                            ? (az >= 0 ? 1e-6f : -1e-6f)
                            : az;
  const float denomPitch = sqrtf(ay * ay + az * az);
  const float denomPitchSafe = (denomPitch < 1e-6f) ? 1e-6f : denomPitch;

  const float accelRoll  = atan2f(ay, denomRoll) * 57.2958f + ROLL_TRIM_DEG;
  const float accelPitch = atan2f(-ax, denomPitchSafe) * 57.2958f + PITCH_TRIM_DEG;

  // Stillness detection — use more accel trust when sitting still
  const float gyroMag = sqrtf(gyroR * gyroR + gyroP * gyroP +
                              gyroYawRate_f * gyroYawRate_f);
  const bool still = (gyroMag < STILL_GYRO_DPS) &&
                     (fabsf(accMag - 1.0f) < STILL_ACCEL_G);

  float fc = still ? COMP_FC_STILL_HZ : COMP_FC_FLY_HZ;

  // Reduce accel trust when experiencing high-G maneuvers
  const float magErr = fabsf(accMag - 1.0f);
  const float trust  = 1.0f - clampf(magErr / 0.4f, 0.0f, 0.9f);
  fc *= trust;

  const float a = pt1Alpha(dt, fc);

  // Integrate gyro
  roll  += gyroR * dt;
  pitch += gyroP * dt;

  // Blend accel correction
  roll  += a * (accelRoll  - roll);
  pitch += a * (accelPitch - pitch);
}

// =================================================================
//                       PID CONTROLLERS
// =================================================================

// Outer loop: angle error → desired rate
float angleToRate(float angleSp, float angleMeas,
                  float *iSum, float kp, float ki,
                  float dt, bool allowI) {
  float err = angleSp - angleMeas;

  if (allowI) {
    *iSum += err * dt;
    *iSum = clampf(*iSum, -ANGLE_I_LIMIT, ANGLE_I_LIMIT);
  } else {
    *iSum *= 0.95f;   // faster decay when disabled
  }

  return kp * err + ki * (*iSum);
}

// Inner loop: rate PID with derivative-on-measurement
float ratePID(float rateSp, float rateMeas,
              float *iSum, float *prevMeas, float *dState,
              float kp, float ki, float kd, float dt,
              bool allowI, float d_fc_hz) {
  float err = rateSp - rateMeas;

  // --- I term with anti-windup ---
  if (allowI) {
    *iSum += err * dt;
    *iSum = clampf(*iSum, -RATE_I_LIMIT, RATE_I_LIMIT);
  } else {
    *iSum *= 0.95f;
  }

  // --- D term on measurement (avoids setpoint kick) ---
  float dRaw = -(rateMeas - *prevMeas) / dt;   // negative: already "error derivative"
  *prevMeas = rateMeas;

  float aD = pt1Alpha(dt, d_fc_hz);
  *dState += aD * (dRaw - *dState);

  return kp * err + ki * (*iSum) + kd * (*dState);
}

// =================================================================
//                MOTOR MIXING + DESATURATION
// =================================================================
void motorMix(int escBase, float rollOut, float pitchOut, float yawOut) {
#if INVERT_YAW_MIX
  yawOut = -yawOut;
#endif

  //  Standard X-quad mix:
  //  FL = base - pitch + roll - yaw
  //  FR = base - pitch - roll + yaw
  //  RL = base + pitch + roll + yaw
  //  RR = base + pitch - roll - yaw
  float m[4];
  m[0] = escBase - pitchOut + rollOut - yawOut;   // FL
  m[1] = escBase - pitchOut - rollOut + yawOut;   // FR
  m[2] = escBase + pitchOut + rollOut + yawOut;   // RL
  m[3] = escBase + pitchOut - rollOut - yawOut;   // RR

  // --- Desaturation: symmetric shift ---
  //  If any motor exceeds bounds, shift ALL motors together
  //  to preserve the PID differential. If the range still doesn't
  //  fit, proportionally scale the PID contribution.
  float mMin = m[0], mMax = m[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (m[i] < mMin) mMin = m[i];
    if (m[i] > mMax) mMax = m[i];
  }

  // Shift to fit within bounds
  float shift = 0.0f;
  if (mMax > ESC_MAX_US) shift = ESC_MAX_US - mMax;      // negative shift
  if ((mMin + shift) < ESC_MIN_US) shift = ESC_MIN_US - mMin;  // positive shift

  for (uint8_t i = 0; i < 4; i++) m[i] += shift;

  // If the spread still exceeds 1000 us (rare, extreme input),
  // proportionally scale PID to fit
  mMin = m[0]; mMax = m[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (m[i] < mMin) mMin = m[i];
    if (m[i] > mMax) mMax = m[i];
  }
  float spread = mMax - mMin;
  float maxSpread = (float)(ESC_MAX_US - ESC_MIN_US);
  if (spread > maxSpread && spread > 0.0f) {
    float center = (mMax + mMin) * 0.5f;
    float scale  = maxSpread / spread;
    for (uint8_t i = 0; i < 4; i++) {
      m[i] = center + (m[i] - center) * scale;
    }
  }

  // Write to motor array with idle floor (airmode)
  for (uint8_t i = 0; i < 4; i++) {
    int val = (int)m[i];
    if (val < MOTOR_IDLE_US) val = MOTOR_IDLE_US;
    if (val > ESC_MAX_US)    val = ESC_MAX_US;
    motorUS[i] = (uint16_t)val;
  }
}

// =================================================================
//                       CALIBRATION
// =================================================================
void calibrateGyroBias() {
  const int N = 800;
  float sx = 0, sy = 0, sz = 0;

  for (int i = 0; i < N; i++) {
    IMU.update();
    IMU.getGyro(&gyro);
    sx += gyro.gyroX;
    sy += gyro.gyroY;
    sz += gyro.gyroZ;
    delay(3);
  }
  gyroBiasX = sx / N;
  gyroBiasY = sy / N;
  gyroBiasZ = sz / N;

  Serial.print(F("Gyro bias: "));
  Serial.print(gyroBiasX, 4); Serial.print(F(", "));
  Serial.print(gyroBiasY, 4); Serial.print(F(", "));
  Serial.println(gyroBiasZ, 4);
}

void calibrateAccelScale() {
  const int N = 400;
  float sumMag = 0;

  for (int i = 0; i < N; i++) {
    IMU.update();
    IMU.getAccel(&accel);
    float mag = sqrtf(accel.accelX * accel.accelX +
                      accel.accelY * accel.accelY +
                      accel.accelZ * accel.accelZ);
    sumMag += mag;
    delay(3);
  }

  float avg = sumMag / N;
  accelScale = (avg > 0.0001f) ? (1.0f / avg) : 1.0f;

  Serial.print(F("Accel scale: "));
  Serial.println(accelScale, 6);
}

// =================================================================
//                       DEBUG HUD
// =================================================================
void dbgHud(uint32_t nowMs, bool failsafeNow, int escBase, float dt) {
#if !DEBUG_HUD_ENABLE
  (void)nowMs; (void)failsafeNow; (void)escBase; (void)dt;
  return;
#else
  if (nowMs - dbgLastHudMs < DEBUG_HUD_INTERVAL_MS) return;
  dbgLastHudMs = nowMs;

  if (armed != dbgLastArmed) {
    dbgLastArmed = armed;
    dbgEvent(armed ? F("ARMED") : F("DISARMED"));
  }

  uint32_t pkts = dbgRadioPackets;
  uint32_t pktDelta = pkts - dbgLastRadioPackets;
  dbgLastRadioPackets = pkts;

  Serial.print(F("[HUD] t=")); Serial.print(nowMs);
  Serial.print(F(" ARM=")); Serial.print(armed);
  Serial.print(F(" FS="));  Serial.print(failsafeNow);
  Serial.print(F(" pk+"));  Serial.print(pktDelta);

  Serial.print(F(" thr=")); Serial.print(escBase);

  Serial.print(F(" R="));   Serial.print(roll, 1);
  Serial.print(F(" P="));   Serial.print(pitch, 1);

  Serial.print(F(" spR=")); Serial.print(rollSetpoint, 1);
  Serial.print(F(" spP=")); Serial.print(pitchSetpoint, 1);

  Serial.print(F(" gR="));  Serial.print(gyroRollRate_f, 1);
  Serial.print(F(" gP="));  Serial.print(gyroPitchRate_f, 1);
  Serial.print(F(" gY="));  Serial.print(gyroYawRate_f, 1);

  Serial.print(F(" FL="));  Serial.print(motorUS[0]);
  Serial.print(F(" FR="));  Serial.print(motorUS[1]);
  Serial.print(F(" RL="));  Serial.print(motorUS[2]);
  Serial.print(F(" RR="));  Serial.print(motorUS[3]);

  Serial.print(F(" dt="));  Serial.print(dt, 4);
  Serial.println();
#endif
}
