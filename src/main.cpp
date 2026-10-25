#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Connect the HW-827 analog output (AO/S) to GPIO4 and share GND.
constexpr uint8_t SENSOR_PIN = 4;
constexpr uint8_t BEAT_LED_PIN = 5;
// Connect the SSD1306 OLED SDA to GPIO6 and SCL to GPIO7.
constexpr uint8_t OLED_SDA_PIN = 6;
constexpr uint8_t OLED_SCL_PIN = 7;
constexpr uint8_t OLED_ADDRESS = 0x3C;
constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr uint32_t OLED_REFRESH_INTERVAL_MS = 250;
constexpr uint32_t BEAT_LED_FLASH_MS = 60;
constexpr uint32_t SAMPLE_INTERVAL_MS = 5;
constexpr uint32_t REPORT_INTERVAL_MS = 50;
constexpr uint32_t REFRACTORY_PERIOD_MS = 300;
constexpr uint32_t STALE_PULSE_MS = 2500;
constexpr uint8_t BEAT_INTERVAL_WINDOW = 7;
constexpr int MIN_PULSE_THRESHOLD = 12;
constexpr float BASELINE_ALPHA = 0.002f;
constexpr float SIGNAL_ALPHA = 0.35f;
constexpr float ENVELOPE_ALPHA = 0.01f;
constexpr float THRESHOLD_FRACTION = 0.5f;
constexpr uint8_t GRAPH_SAMPLE_DIVIDER = 4;

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
bool oledReady = false;
portMUX_TYPE displayStateMux = portMUX_INITIALIZER_UNLOCKED;
int16_t graphSamples[OLED_WIDTH] = {};
uint8_t graphWriteIndex = 0;
uint8_t graphSampleCount = 0;
uint8_t samplesUntilGraphPoint = GRAPH_SAMPLE_DIVIDER;

float baseline = 0.0f;
float filteredSignal = 0.0f;
float signalEnvelope = 0.0f;
bool baselineInitialized = false;
bool signalAboveThreshold = false;
bool trackingBeatPeak = false;
int beatPeakValue = 0;
uint32_t beatPeakMs = 0;
uint32_t lastSampleMs = 0;
uint32_t lastReportMs = 0;
uint32_t lastBeatMs = 0;
uint32_t ledOffAtMs = 0;
uint32_t beatIntervals[BEAT_INTERVAL_WINDOW] = {};
uint8_t beatIntervalCount = 0;
uint8_t nextInterval = 0;
int latestRaw = 0;
int latestSignal = 0;
int currentBpm = 0;

int pulseThreshold() {
  return max(MIN_PULSE_THRESHOLD,
             static_cast<int>(signalEnvelope * THRESHOLD_FRACTION));
}

void recordGraphSample() {
  portENTER_CRITICAL(&displayStateMux);
  graphSamples[graphWriteIndex] = static_cast<int16_t>(filteredSignal);
  graphWriteIndex = (graphWriteIndex + 1) % OLED_WIDTH;
  if (graphSampleCount < OLED_WIDTH) {
    ++graphSampleCount;
  }
  portEXIT_CRITICAL(&displayStateMux);
}

void drawOled() {
  if (!oledReady) {
    return;
  }

  int16_t samples[OLED_WIDTH];
  uint8_t writeIndex;
  uint8_t sampleCount;
  float envelope;
  int bpm;
  portENTER_CRITICAL(&displayStateMux);
  for (uint8_t i = 0; i < OLED_WIDTH; ++i) {
    samples[i] = graphSamples[i];
  }
  writeIndex = graphWriteIndex;
  sampleCount = graphSampleCount;
  envelope = signalEnvelope;
  bpm = currentBpm;
  portEXIT_CRITICAL(&displayStateMux);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("BPM");
  display.setTextSize(2);
  display.setCursor(38, 0);
  if (bpm > 0) {
    display.print(bpm);
  } else {
    display.print("--");
  }

  constexpr int graphTop = 22;
  constexpr int graphBottom = OLED_HEIGHT - 1;
  constexpr int graphCenter = (graphTop + graphBottom) / 2;
  constexpr int graphAmplitude = (graphBottom - graphTop) / 2;
  float graphRange = envelope * 1.8f;
  if (graphRange < MIN_PULSE_THRESHOLD * 2) {
    graphRange = MIN_PULSE_THRESHOLD * 2;
  }

  for (int x = 0; x < OLED_WIDTH; x += 4) {
    display.drawPixel(x, graphCenter, SSD1306_WHITE);
  }

  const int threshold = max(MIN_PULSE_THRESHOLD,
                            static_cast<int>(envelope * THRESHOLD_FRACTION));
  int thresholdY = graphCenter - static_cast<int>(threshold * graphAmplitude / graphRange);
  if (thresholdY < graphTop) {
    thresholdY = graphTop;
  }
  for (int x = 2; x < OLED_WIDTH; x += 5) {
    display.drawPixel(x, thresholdY, SSD1306_WHITE);
  }

  const uint8_t firstIndex =
        (writeIndex + OLED_WIDTH - sampleCount) % OLED_WIDTH;
      const uint8_t firstX = OLED_WIDTH - sampleCount;
  int previousX = 0;
  int previousY = graphCenter;
  for (uint8_t i = 0; i < sampleCount; ++i) {
    const int x = firstX + i;
    const uint8_t sampleIndex = (firstIndex + i) % OLED_WIDTH;
    int y = graphCenter -
            static_cast<int>(samples[sampleIndex] * graphAmplitude / graphRange);
    if (y < graphTop) {
      y = graphTop;
    } else if (y > graphBottom) {
      y = graphBottom;
    }
    if (i > 0) {
      display.drawLine(previousX, previousY, x, y, SSD1306_WHITE);
    } else {
      display.drawPixel(x, y, SSD1306_WHITE);
    }
    previousX = x;
    previousY = y;
  }

  display.display();
}

void oledDisplayTask(void*) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(OLED_REFRESH_INTERVAL_MS));
    drawOled();
  }
}

uint32_t medianBeatInterval() {
  if (beatIntervalCount == 0) {
    return 0;
  }

  uint32_t sortedIntervals[BEAT_INTERVAL_WINDOW];
  for (uint8_t i = 0; i < beatIntervalCount; ++i) {
    sortedIntervals[i] = beatIntervals[i];
  }

  for (uint8_t i = 1; i < beatIntervalCount; ++i) {
    const uint32_t value = sortedIntervals[i];
    uint8_t position = i;
    while (position > 0 && sortedIntervals[position - 1] > value) {
      sortedIntervals[position] = sortedIntervals[position - 1];
      --position;
    }
    sortedIntervals[position] = value;
  }

  const uint8_t middle = beatIntervalCount / 2;
  if (beatIntervalCount % 2 == 0) {
    return (sortedIntervals[middle - 1] + sortedIntervals[middle]) / 2;
  }
  return sortedIntervals[middle];
}

void processDetectedBeat(uint32_t beatTime, uint32_t detectionTime) {
  if (lastBeatMs != 0) {
    const uint32_t interval = beatTime - lastBeatMs;
    if (interval >= 300 && interval <= 2000) {
      const uint32_t previousMedian = medianBeatInterval();
      const uint32_t tolerance = max(250UL, previousMedian * 40UL / 100UL);
      const bool isConsistent = beatIntervalCount < 3 ||
          (interval + tolerance >= previousMedian && interval <= previousMedian + tolerance);

      if (isConsistent) {
        beatIntervals[nextInterval] = interval;
        nextInterval = (nextInterval + 1) % BEAT_INTERVAL_WINDOW;
        if (beatIntervalCount < BEAT_INTERVAL_WINDOW) {
          ++beatIntervalCount;
        }
        const uint32_t medianInterval = medianBeatInterval();
        if (medianInterval > 0) {
          portENTER_CRITICAL(&displayStateMux);
          currentBpm = static_cast<int>(60000UL / medianInterval);
          portEXIT_CRITICAL(&displayStateMux);
        }
      }
    }
  }

  lastBeatMs = beatTime;
  digitalWrite(BEAT_LED_PIN, HIGH);
  ledOffAtMs = detectionTime + BEAT_LED_FLASH_MS;
  Serial.print("Beat detected");
  if (currentBpm > 0) {
    Serial.print(" | BPM: ");
    Serial.println(currentBpm);
  } else {
    Serial.println(" | BPM: calculating");
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(BEAT_LED_PIN, OUTPUT);
  digitalWrite(BEAT_LED_PIN, LOW);
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  oledReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  Serial.println("HW-827 pulse monitor: GPIO4 analog input");
  Serial.println("Beat LED: GPIO5");
  Serial.println(oledReady ? "SSD1306 OLED ready on GPIO6/7" : "SSD1306 OLED not found");
  if (oledReady && xTaskCreate(oledDisplayTask, "oled-display", 4096, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("OLED refresh task could not start");
  }
  Serial.println("Plot Signal, Threshold, and BPM with Arduino Serial Plotter.");
}
void loop() {
  const uint32_t now = millis();
  if (ledOffAtMs != 0 && static_cast<int32_t>(now - ledOffAtMs) >= 0) {
    digitalWrite(BEAT_LED_PIN, LOW);
    ledOffAtMs = 0;
  }

  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    lastSampleMs = now;
    latestRaw = analogRead(SENSOR_PIN);

    if (!baselineInitialized) {
      baseline = static_cast<float>(latestRaw);
      baselineInitialized = true;
    } else {
      baseline += BASELINE_ALPHA * (static_cast<float>(latestRaw) - baseline);
    }

    latestSignal = latestRaw - static_cast<int>(baseline);
    filteredSignal += SIGNAL_ALPHA * (static_cast<float>(latestSignal) - filteredSignal);
    portENTER_CRITICAL(&displayStateMux);
    signalEnvelope += ENVELOPE_ALPHA * (fabsf(filteredSignal) - signalEnvelope);
    portEXIT_CRITICAL(&displayStateMux);
    const int threshold = pulseThreshold();
    const int filteredValue = static_cast<int>(filteredSignal);

    if (--samplesUntilGraphPoint == 0) {
      recordGraphSample();
      samplesUntilGraphPoint = GRAPH_SAMPLE_DIVIDER;
    }

    if (filteredValue >= threshold && !signalAboveThreshold) {
      signalAboveThreshold = true;
      trackingBeatPeak = true;
      beatPeakValue = filteredValue;
      beatPeakMs = now;
    }

    if (trackingBeatPeak) {
      if (filteredValue > beatPeakValue) {
        beatPeakValue = filteredValue;
        beatPeakMs = now;
      } else {
        const int peakDrop = max(3, beatPeakValue / 8);
        if (beatPeakValue - filteredValue >= peakDrop) {
          trackingBeatPeak = false;
          if (beatPeakMs - lastBeatMs >= REFRACTORY_PERIOD_MS) {
            processDetectedBeat(beatPeakMs, now);
          }
        }
      }
    }

    if (filteredValue < threshold / 2) {
      signalAboveThreshold = false;
    }
  }

  if (now - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = now;
    if (lastBeatMs == 0 || now - lastBeatMs > STALE_PULSE_MS) {
      portENTER_CRITICAL(&displayStateMux);
      currentBpm = 0;
      portEXIT_CRITICAL(&displayStateMux);
    }

    const int threshold = pulseThreshold();
    Serial.print("Signal:");
    Serial.print(static_cast<int>(filteredSignal));
    Serial.print(" Threshold:");
    Serial.print(threshold);
    Serial.print(" BPM:");
    Serial.println(currentBpm);
  }

}