#include <SPI.h>
#include <RF24.h>

// ---------- CONFIG ----------
#define BAUD 9600
#define USE_RADIO 1          // set to 0 to ignore RF24 (sensor-only debug)
#define DEBUG_PRINT_EVERY_MS 250
#define THRESHOLD_CM 30.0f
#define PING_INTERVAL_MS 60

// Pins (yours)
const uint8_t TRIG_PIN = 4;
const uint8_t ECHO_PIN = 3;

const uint8_t CE_PIN   = 9;
const uint8_t CSN_PIN  = 10;

// Radio
#if USE_RADIO
RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[5] = { 'F','G','H','I','J' };
const uint8_t RF_CHANNEL = 2;
const uint8_t PAYLOAD_LEN = 32;
const uint16_t CAN_ID = 5000; // 0x1388
#endif

// Lap timing / speed
const float SENSOR_ZONE_LENGTH_M = 0.30f;

bool     timerStarted   = false;
uint32_t lapStartMS     = 0;
uint32_t bestLapMS      = 0;
int      lapCount       = 0;

bool     prevCarPresent = false;
uint32_t carEnterUs     = 0;
uint32_t carExitUs      = 0;

// Onboard LED (UNO = 13)
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// ---- Robust ultrasonic read ----
float measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for rising edge with timeout
  unsigned long t0 = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - t0 > 30000UL) return 1e9f; // "far"
  }
  // Measure HIGH width (cap at ~5 m)
  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - start > 30000UL) return 1e9f;
  }
  unsigned long duration = micros() - start;
  return (duration * 0.0343f) * 0.5f;  // cm
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Keep SS as OUTPUT so SPI stays master
  pinMode(10, OUTPUT);

  Serial.begin(BAUD);
  delay(200);

  // Boot banner
  Serial.println(F("\n=== Lap TX w/ HEX payload (diag) ==="));
  Serial.print(F("TRIG=")); Serial.print(TRIG_PIN);
  Serial.print(F("  ECHO=")); Serial.println(ECHO_PIN);

#if USE_RADIO
  Serial.print(F("RF CE=")); Serial.print(CE_PIN);
  Serial.print(F("  CSN=")); Serial.println(CSN_PIN);
  Serial.println(F("Init RF24..."));
  if (!radio.begin()) Serial.println(F("radio.begin() FAILED (check power/CE/CSN/SPI/MISO)."));
  radio.setChannel(RF_CHANNEL);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(5, 15);
  radio.setPayloadSize(PAYLOAD_LEN);
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();
  Serial.println(F("RF24 configured."));
#else
  Serial.println(F("RF24 disabled (USE_RADIO=0). Sensor-only debug."));
#endif

  // Blink 3× at boot
  for (int i = 0; i < 3; i++) { digitalWrite(LED_BUILTIN, HIGH); delay(120); digitalWrite(LED_BUILTIN, LOW); delay(120); }
}

void loop() {
  static uint32_t lastBlink = 0, lastDbg = 0;
  static bool ledState = false;  // <-- FIX: track LED state

  // Heartbeat LED every ~250 ms
  if (millis() - lastBlink >= 250) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    lastBlink = millis();
  }

  float d = measureDistanceCM();
  bool carPresent = (d < THRESHOLD_CM);
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  // periodic debug
  if (millis() - lastDbg >= DEBUG_PRINT_EVERY_MS) {
    Serial.print(F("d="));
    if (d > 1e8f) Serial.print(F("far"));
    else Serial.print(d, 1);
    Serial.print(F(" cm  car="));
    Serial.println(carPresent ? F("1") : F("0"));
    lastDbg = millis();
  }

  // Rising edge
  if (!prevCarPresent && carPresent) {
    carEnterUs = nowUs;
    if (!timerStarted) {
      timerStarted = true;
      lapStartMS   = nowMs;
      lapCount     = 0;
      bestLapMS    = 0;
      Serial.println(F("START stopwatch"));
    } else {
      Serial.println(F("ENTER"));
    }
  }

  // Falling edge -> record lap
  if (prevCarPresent && !carPresent) {
    carExitUs = nowUs;

    if (timerStarted) {
      uint32_t lapMS = nowMs - lapStartMS;
      lapStartMS = nowMs;
      lapCount++;
      if (bestLapMS == 0 || lapMS < bestLapMS) bestLapMS = lapMS;

      // speed (safe)
      uint32_t crossUs = carExitUs - carEnterUs;
      float mph = 0.0f;
      if (crossUs > 0) {
        float secs = crossUs / 1e6f;
        float mps  = SENSOR_ZONE_LENGTH_M / secs;
        mph        = mps * 2.23694f;
      }
      long mphx100 = (long)(mph * 100.0f + 0.5f);

      // pack 64b: [63:32]=mph*100, [31:0]=lapMS  -> 16 hex chars
      uint64_t packed = ((uint64_t)(uint32_t)mphx100 << 32) | (uint32_t)lapMS;
      uint32_t hi = (uint32_t)(packed >> 32);
      uint32_t lo = (uint32_t)(packed & 0xFFFFFFFFUL);
      char dataHex[17];
      snprintf(dataHex, sizeof(dataHex), "%08lX%08lX", (unsigned long)hi, (unsigned long)lo);

      char packet[32];
      snprintf(packet, sizeof(packet), "%X,%s", (unsigned)CAN_ID, dataHex);

#if USE_RADIO
      bool ok = radio.write(packet, strlen(packet) + 1);
      Serial.print(F("LAP ")); Serial.print(lapCount);
      Serial.print(F(": ")); Serial.print(lapMS); Serial.print(F(" ms  best=")); Serial.print(bestLapMS);
      Serial.print(F(" ms  mph=")); Serial.print(mph, 2);
      Serial.print(F("  TX=")); Serial.print(ok ? F("OK") : F("FAIL"));
      Serial.print(F("  -> ")); Serial.println(packet);
#else
      Serial.print(F("LAP ")); Serial.print(lapCount);
      Serial.print(F(": ")); Serial.print(lapMS); Serial.print(F(" ms  best=")); Serial.print(bestLapMS);
      Serial.print(F(" ms  mph=")); Serial.print(mph, 2);
      Serial.print(F("  (RF disabled)  -> ")); Serial.println(packet);
#endif
    }
  }

  prevCarPresent = carPresent;
  delay(PING_INTERVAL_MS);
}
