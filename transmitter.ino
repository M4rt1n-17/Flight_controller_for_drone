
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <stdint.h>

// ========== PINS ==========
#define CE_PIN  7
#define CSN_PIN 8

#define THROTTLE_PIN    A2
#define ROLL_PIN        A3
#define PITCH_PIN       A4

// ========== CONFIG ==========
#define INVERT_PITCH    true
#define TX_HZ           50
#define TX_INTERVAL_MS  (1000 / TX_HZ)
#define DEADZONE        25

// Expo curve: 0.0 = fully linear, 1.0 = fully cubic (very soft center)
// 0.3–0.5 is a good starting range for hover practice
#define EXPO_ROLL_PITCH  0.40f
#define EXPO_THROTTLE    0.25f

// ========== RADIO ==========
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// ========== CALIBRATION ==========
int throttleCenter = 512;
int rollCenter     = 512;
int pitchCenter    = 512;

// ========== TX STRUCT — MUST MATCH RECEIVER EXACTLY ==========
struct ControllerData {
  int throttle;   // 0..2000
  int roll;       // -500..500
  int pitch;      // -500..500
  int yaw;        // -500..500
};

ControllerData txData = {0, 0, 0, 0};

unsigned long lastTxTime = 0;

// ========== HELPERS ==========

// Attempt expo curve: soft around center, full authority at endpoints.
// Formula: out = expo * x^3 + (1 - expo) * x
// Input/output range: -500..500
int applyExpo(int value, float expo) {
  float x = (float)value / 500.0f;               // normalize to -1..1
  float curved = expo * x * x * x + (1.0f - expo) * x;
  return constrain((int)(curved * 500.0f), -500, 500);
}

int mapWithDeadzone(int raw, int center, int dz, float expo) {
  int mapped = map(raw, 0, 1023, -500, 500);
  int centerMapped = map(center, 0, 1023, -500, 500);
  mapped -= centerMapped;

  if (abs(mapped) < dz) return 0;
  mapped = constrain(mapped, -500, 500);

  return applyExpo(mapped, expo);
}

void calibrate() {
  Serial.println(F("Center both sticks. Calibrating in 3 s..."));
  delay(3000);

  long tSum = 0, rSum = 0, pSum = 0;
  const int N = 100;

  for (int i = 0; i < N; i++) {
    tSum += analogRead(THROTTLE_PIN);
    rSum += analogRead(ROLL_PIN);
    pSum += analogRead(PITCH_PIN);
    delay(10);
  }

  throttleCenter = tSum / N;
  rollCenter     = rSum / N;
  pitchCenter    = pSum / N;

  Serial.print(F("Centers — T:")); Serial.print(throttleCenter);
  Serial.print(F(" R:"));          Serial.print(rollCenter);
  Serial.print(F(" P:"));          Serial.println(pitchCenter);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);

  if (!radio.begin()) {
    Serial.println(F("Radio not found!"));
    while (1) {}
  }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.stopListening();

  Serial.print(F("TX struct size: ")); Serial.println(sizeof(ControllerData));

  calibrate();

  Serial.println(F("=== TX READY ==="));
}

// ========== LOOP ==========
void loop() {
  unsigned long now = millis();
  if (now - lastTxTime < TX_INTERVAL_MS) return;
  lastTxTime = now;

  // ---- Throttle: below center = 0, above center = 0..2000 with expo ----
  int throttleRaw = analogRead(THROTTLE_PIN);
  if (throttleRaw <= throttleCenter) {
    txData.throttle = 0;
  } else {
    long t = map(throttleRaw, throttleCenter, 1023, 0, 2000);
    t = constrain(t, 0L, 2000L);
    // Apply expo to throttle (normalize 0..2000 → -500..500, curve, map back)
    float x = (float)t / 2000.0f;   // 0..1
    float curved = EXPO_THROTTLE * x * x * x + (1.0f - EXPO_THROTTLE) * x;
    txData.throttle = constrain((int)(curved * 2000.0f), 0, 2000);
  }

  // ---- Roll / Pitch with expo ----
  txData.roll  = mapWithDeadzone(analogRead(ROLL_PIN),  rollCenter,  DEADZONE, EXPO_ROLL_PITCH);
  txData.pitch = mapWithDeadzone(analogRead(PITCH_PIN), pitchCenter, DEADZONE, EXPO_ROLL_PITCH);

#if INVERT_PITCH
  txData.pitch = -txData.pitch;
#endif

  // ---- Yaw: no yaw stick in this build ----
  txData.yaw = 0;

  // ---- Transmit ----
  bool ok = radio.write(&txData, sizeof(ControllerData));

  // ---- Debug (10 Hz) ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 100) {
    lastPrint = now;
    Serial.print(F("T:")); Serial.print(txData.throttle);
    Serial.print(F(" R:")); Serial.print(txData.roll);
    Serial.print(F(" P:")); Serial.print(txData.pitch);
    Serial.println(ok ? F(" OK") : F(" FAIL"));
  }
}
