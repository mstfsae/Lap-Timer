#include <SPI.h>
#include <RF24.h>

const uint8_t TRIG_PIN  = 3;   // this pins need to be finalized i just used dummy values here
const uint8_t ECHO_PIN  = 4;

const uint8_t CE_PIN   = 9;
const uint8_t CSN_PIN  = 10;
RF24 radio(CE_PIN, CSN_PIN);

// found the address!! ("FGHIJ"):contentReference[oaicite:0]{index=0}
const byte ADDRESS[6] = "FGHIJ";

// i need to find a decent can id port that they want to integrate into the dash
const uint16_t CAN_ID = 5000;

// nRF24 channel and payload length to match the receiver
const uint8_t RF_CHANNEL = 2;
const uint8_t PAYLOAD_LEN = 32;

const float DETECTION_THRESHOLD_CM = 30.0; // adjust once we start testing
const float SENSOR_ZONE_LENGTH_M   = 0.30; // length of the zone used for speed

bool timerStarted     = false;
uint32_t lapStartTime = 0;
int      lapCount     = 0;
uint32_t bestLapTime  = 0;

bool     prevCarPresent = false;
uint32_t carEnterMicros = 0;
uint32_t carExitMicros  = 0;

// trigger the HC‑SR04 and return distance in cm
float measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // duration is round‑trip time so divide by 2 and multiply by sound speed to get distance:contentReference[oaicite:1]{index=1}.
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 20000UL);
  float distance_cm = (duration * 0.0343f) / 2.0f;
  return distance_cm;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // init radio and receiver
  radio.begin();
  radio.setChannel(RF_CHANNEL);            // same channel as receiver
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(ADDRESS);          // write to the receiver's address
  radio.setPayloadSize(PAYLOAD_LEN);       // 32‑byte payloads
  radio.stopListening();                   // transmitter mode:contentReference[oaicite:2]{index=2}
}

void loop() {
  float distance = measureDistanceCM();
  bool carPresent = (distance < DETECTION_THRESHOLD_CM);
  uint32_t nowMicros = micros();

  if (!prevCarPresent && carPresent) {
    carEnterMicros = nowMicros;
  }

  if (prevCarPresent && !carPresent) {
    carExitMicros = nowMicros;

    if (!timerStarted) {
      lapStartTime  = millis();
      timerStarted  = true;
      lapCount      = 0;
      bestLapTime   = 0;
    } else {
      uint32_t currentMillis = millis();
      uint32_t lapTimeMS     = currentMillis - lapStartTime;
      lapStartTime           = currentMillis;
      lapCount++;

      if (bestLapTime == 0 || lapTimeMS < bestLapTime) {
        bestLapTime = lapTimeMS;
      }

      // est. speed and record in mph (two decimal places)
      uint32_t crossTimeUS = carExitMicros - carEnterMicros;
      float crossTimeSec   = crossTimeUS / 1e6f;
      float speed_mps      = SENSOR_ZONE_LENGTH_M / crossTimeSec;
      float speed_mph      = speed_mps * 2.23694f;
      long mphTimes100     = (long)(speed_mph * 100.0f + 0.5f);

      // encode lap time (ms) in the lower 32 bits and speed (mph*100) in the upper 32 bits
      // i might need to change the packet scheme but who knows ig
      uint64_t packedData = ((uint64_t)mphTimes100 << 32) | (uint64_t)lapTimeMS;

      // convert the 64‑bit value to a 16‑digit hex string
      char dataHex[17];
      snprintf(dataHex, sizeof(dataHex), "%016llX", (unsigned long long)packedData);

      // build packet: "CAN_ID_HEX,DATA_HEX" but they might not want hex we'll see
      char packet[32];
      snprintf(packet, sizeof(packet), "%X,%s", CAN_ID, dataHex);

      // send and log
      radio.write(packet, strlen(packet) + 1);
      Serial.print("TX: "); Serial.println(packet);
    }
  }

  prevCarPresent = carPresent;
  delay(30);
}
