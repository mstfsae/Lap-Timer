// esp32-s3 wroom (devkitc-style) — lap tx via nrf24 + hc-sr04
#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <printf.h>   // for radio.printDetails()

// -------- config --------
#define BAUD 9600
#define USE_RADIO 1
#define DEBUG_PRINT_EVERY_MS 250
#define THRESHOLD_CM 30.0f
#define PING_INTERVAL_MS 60

// ultrasonic (echo is 5v on hc-sr04 -> use a divider or a 3v3-safe variant)
const uint8_t TRIG_PIN = 14;
const uint8_t ECHO_PIN = 21;

// spi mapping (hspi-style)
const uint8_t SPI_SCK  = 12;
const uint8_t SPI_MISO = 13;
const uint8_t SPI_MOSI = 11;

// nrf24 control pins
const uint8_t CE_PIN  = 15;
const uint8_t CSN_PIN = 10;

// radio
#if USE_RADIO
RF24 radio(CE_PIN, CSN_PIN);
const uint8_t ADDRESS[5]  = { 'F','G','H','I','J' };
// use a cleaner channel and slower rate for extra link margin
const uint8_t RF_CHANNEL  = 76;                 // both ends must match
const uint16_t CAN_ID     = 5000;               // 0x1388
#endif

// lap timing / speed
const float SENSOR_ZONE_LENGTH_M = 0.30f;

bool     timerStarted   = false;
uint32_t lapStartMS     = 0;
uint32_t bestLapMS      = 0;
int      lapCount       = 0;

bool     prevCarPresent = false;
uint32_t carEnterUs     = 0;
uint32_t carExitUs      = 0;

// handle boards without a simple LED_BUILTIN
#ifdef LED_BUILTIN
constexpr int LED_PIN = LED_BUILTIN;
#else
constexpr int LED_PIN = -1;
#endif

float measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  // wait for rising edge with timeout
  unsigned long t0 = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - t0 > 30000UL) return 1e9f; // “far”
  }
  // measure high width (cap at ~5 m)
  unsigned long start = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - start > 30000UL) return 1e9f;
  }
  unsigned long duration = micros() - start;
  return (duration * 0.0343f) * 0.5f;  // cm
}

#if USE_RADIO
void rf24_print_diag() {
  printf_begin();                 // enable RF24's printf to Serial
  Serial.println(F("\n[rf24 diag]"));
  bool chip = radio.isChipConnected();
  Serial.print(F("isChipConnected="));
  Serial.println(chip ? F("yes") : F("no"));
  radio.printDetails();           // dump config/registers
}

void rf24_sanity_send_noack() {
  // try a one-off send with auto-ack disabled; this should return true
  // even without a receiver, proving spi + power + ce/csn are ok.
  bool prevAA = true;
  // there's no direct getter; assume we configured auto-ack=true just before.
  // temporarily disable and send a tiny payload
  radio.setAutoAck(false);
  const char probe[] = "diag_noack";
  bool ok = radio.write(&probe, sizeof(probe));
  Serial.print(F("[rf24 diag] no-ack sanity send -> "));
  Serial.println(ok ? F("ok (wiring/power look good)") : F("fail (check spi/ce/csn/3v3)"));
  radio.setAutoAck(true);
}

void rf24_configure() {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CSN_PIN);

  Serial.printf("rf pins: CE=%u  CSN=%u  SCK=%u  MISO=%u  MOSI=%u\n",
                CE_PIN, CSN_PIN, SPI_SCK, SPI_MISO, SPI_MOSI);

  Serial.println(F("init rf24..."));
  if (!radio.begin()) {
    Serial.println(F("radio.begin() failed (check power/ce/csn/spi wiring)."));
  }

  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(15, 15);
  radio.openWritingPipe(ADDRESS);
  radio.stopListening();

  rf24_print_diag();
  rf24_sanity_send_noack();
}
#endif

void setup() {
  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);  

  Serial.begin(BAUD);
  delay(200);

  Serial.println(F("\nesp32-s3 lap tx (nrf24)"));
  Serial.printf("ultrasonic: TRIG=%u  ECHO=%u (use divider on echo)\n", TRIG_PIN, ECHO_PIN);

#if USE_RADIO
  rf24_configure();
#else
  Serial.println(F("rf24 disabled (USE_RADIO=0). sensor-only debug."));
#endif

  // simple boot blink
  for (int i = 0; i < 3; i++) {
    if (LED_PIN >= 0) { digitalWrite(LED_PIN, HIGH); }
    delay(120);
    if (LED_PIN >= 0) { digitalWrite(LED_PIN, LOW); }
    delay(120);
  }
}

void loop() {
  static uint32_t lastBlink = 0, lastDbg = 0;
  static bool ledState = false;

  // heartbeat
  if (LED_PIN >= 0 && millis() - lastBlink >= 250) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
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

  // rising edge
  if (!prevCarPresent && carPresent) {
    carEnterUs = nowUs;
    if (!timerStarted) {
      timerStarted = true;
      lapStartMS   = nowMs;
      lapCount     = 0;
      bestLapMS    = 0;
      Serial.println(F("start stopwatch"));
    } else {
      Serial.println(F("enter"));
    }
  }

  // falling edge -> record lap
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
      snprintf(dataHex, sizeof(dataHex), "%08lX%08lX",
               (unsigned long)hi, (unsigned long)lo);

      char packet[32];
      snprintf(packet, sizeof(packet), "%X,%s", (unsigned)CAN_ID, dataHex);

#if USE_RADIO
      bool ok = radio.write(packet, strlen(packet) + 1);
      Serial.print(F("lap ")); Serial.print(lapCount);
      Serial.print(F(": ")); Serial.print(lapMS); Serial.print(F(" ms  best=")); Serial.print(bestLapMS);
      Serial.print(F(" ms  mph=")); Serial.print(mph, 2);
      Serial.print(F("  tx=")); Serial.print(ok ? F("ok") : F("fail"));
      Serial.print(F("  -> ")); Serial.println(packet);
#else
      Serial.print(F("lap ")); Serial.print(lapCount);
      Serial.print(F(": ")); Serial.print(lapMS); Serial.print(F(" ms  best=")); Serial.print(bestLapMS);
      Serial.print(F(" ms  mph=")); Serial.print(mph, 2);
      Serial.print(F("  (rf disabled)  -> ")); Serial.println(packet);
#endif
    }
  }

  prevCarPresent = carPresent;
  delay(PING_INTERVAL_MS);
}
