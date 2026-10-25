#include <Arduino.h>

// Connect the HW-827 analog output (AO/S) to GPIO4 and share GND.
constexpr uint8_t SENSOR_PIN = 4;
constexpr int PULSE_THRESHOLD = 120;
constexpr uint32_t SAMPLE_INTERVAL_MS = 5;
constexpr uint32_t REPORT_INTERVAL_MS = 100;
constexpr uint32_t REFRACTORY_PERIOD_MS = 300;
constexpr uint32_t STALE_PULSE_MS = 2500;

float baseline = 0.0f;
bool baselineInitialized = false;
bool signalAboveThreshold = false;
uint32_t lastSampleMs = 0;
uint32_t lastReportMs = 0;
uint32_t lastBeatMs = 0;
uint32_t beatIntervals[5] = {};
uint8_t beatIntervalCount = 0;
uint8_t nextInterval = 0;
int latestRaw = 0;
int latestSignal = 0;
int currentBpm = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(SENSOR_PIN, INPUT);
  Serial.println("HW-827 pulse monitor: GPIO4 analog input");
  Serial.println("Keep a finger steady on the sensor; readings follow below.");
}

void loop() {
  const uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    latestRaw = analogRead(SENSOR_PIN);

    if (!baselineInitialized) {
      baseline = static_cast<float>(latestRaw);
      baselineInitialized = true;
    } else {
      baseline += 0.002f * (static_cast<float>(latestRaw) - baseline);
    }

    latestSignal = latestRaw - static_cast<int>(baseline);
    if (latestSignal >= PULSE_THRESHOLD && !signalAboveThreshold) {
      signalAboveThreshold = true;
      if (now - lastBeatMs >= REFRACTORY_PERIOD_MS) {
        if (lastBeatMs != 0) {
          const uint32_t interval = now - lastBeatMs;
          if (interval >= 300 && interval <= 2000) {
            beatIntervals[nextInterval] = interval;
            nextInterval = (nextInterval + 1) % 5;
            if (beatIntervalCount < 5) {
              ++beatIntervalCount;
            }

            uint32_t intervalSum = 0;
            for (uint8_t i = 0; i < beatIntervalCount; ++i) {
              intervalSum += beatIntervals[i];
            }
            currentBpm = static_cast<int>(60000UL * beatIntervalCount / intervalSum);
          } else {
            currentBpm = 0;
            beatIntervalCount = 0;
            nextInterval = 0;
          }
        }
        lastBeatMs = now;
        Serial.print("Beat detected");
        if (currentBpm > 0) {
          Serial.print(" | BPM: ");
          Serial.println(currentBpm);
        } else {
          Serial.println(" | BPM: calculating");
        }
      }
    } else if (latestSignal < PULSE_THRESHOLD / 2) {
      signalAboveThreshold = false;
    }
  }

  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    if (lastBeatMs == 0 || now - lastBeatMs > STALE_PULSE_MS) {
      currentBpm = 0;
    }

    Serial.print("raw=");
    Serial.print(latestRaw);
    Serial.print(" signal=");
    Serial.print(latestSignal);
    Serial.print(" BPM=");
    if (currentBpm > 0) {
      Serial.println(currentBpm);
    } else {
      Serial.println("--");
    }
  }
}