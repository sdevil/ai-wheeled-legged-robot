#include "RGBController.h"

#include <driver/rmt.h>

const CRGB CRGB::Black = CRGB(0, 0, 0);
const CRGB CRGB::Red = CRGB(255, 0, 0);
const CRGB CRGB::Green = CRGB(0, 255, 0);
const CRGB CRGB::Blue = CRGB(0, 0, 255);
const CRGB CRGB::Yellow = CRGB(255, 180, 0);

CRGB leds[NUM_LEDS];
unsigned long previousBlinkMillis = 0;
int blinkState = 0;
int blinkCount = 0;
bool isBlinking = false;
CRGB blinkColors[6] = {CRGB::Red, CRGB::Red, CRGB::Yellow, CRGB::Yellow, CRGB::Green, CRGB::Green};
unsigned long blinkDuration = 200;
unsigned long blinkInterval = 200;
CRGB currentBlinkColor = CRGB::Red;
int totalBlinkCount = 3;
bool useColorSequence = false;

namespace {
constexpr uint8_t kBrightness = 50;
constexpr unsigned long kBreathingPeriodMs = 2000;
constexpr unsigned long kBreathingFrameMs = 30;
constexpr rmt_channel_t kRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kRmtClockDiv = 2;
constexpr uint16_t kWs0HighTicks = 14;
constexpr uint16_t kWs0LowTicks = 36;
constexpr uint16_t kWs1HighTicks = 28;
constexpr uint16_t kWs1LowTicks = 24;
portMUX_TYPE ledMux = portMUX_INITIALIZER_UNLOCKED;
bool lowBatteryWarning = false;
bool systemReady = false;
unsigned long previousBreathingMillis = 0;
unsigned long breathingStartedMillis = 0;
uint8_t pendingClearFrames = 0;
unsigned long previousIdleClearMillis = 0;
rmt_item32_t ledItems[NUM_LEDS * 24];
bool rmtReady = false;

inline uint8_t scale(uint8_t value) {
  return (uint16_t)value * kBrightness / 255;
}

void appendByte(uint8_t value, size_t& index) {
  for (int bit = 7; bit >= 0; --bit) {
    const bool one = value & (1 << bit);
    ledItems[index].level0 = 1;
    ledItems[index].duration0 = one ? kWs1HighTicks : kWs0HighTicks;
    ledItems[index].level1 = 0;
    ledItems[index].duration1 = one ? kWs1LowTicks : kWs0LowTicks;
    ++index;
  }
}

void initRmt() {
  if (rmtReady) return;
  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_TX;
  config.channel = kRmtChannel;
  config.gpio_num = static_cast<gpio_num_t>(RGB_PIN);
  config.mem_block_num = 1;
  config.clk_div = kRmtClockDiv;
  config.tx_config.loop_en = false;
  config.tx_config.carrier_en = false;
  config.tx_config.idle_output_en = true;
  config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  config.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  if (rmt_config(&config) == ESP_OK &&
      rmt_driver_install(kRmtChannel, 0, 0) == ESP_OK) {
    rmtReady = true;
  }
}

void showLeds() {
  if (!rmtReady) initRmt();
  if (!rmtReady) return;
  portENTER_CRITICAL(&ledMux);
  size_t index = 0;
  for (int i = 0; i < NUM_LEDS; ++i) {
    appendByte(scale(leds[i].g), index);
    appendByte(scale(leds[i].r), index);
    appendByte(scale(leds[i].b), index);
  }
  portEXIT_CRITICAL(&ledMux);
  rmt_write_items(kRmtChannel, ledItems, index, true);
  rmt_wait_tx_done(kRmtChannel, pdMS_TO_TICKS(10));
  delayMicroseconds(320);
}

void fillSolid(CRGB color) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = color;
  }
}

void forceClearLeds(uint8_t frames = 3) {
  fillSolid(CRGB::Black);
  for (uint8_t i = 0; i < frames; ++i) {
    showLeds();
    delayMicroseconds(320);
  }
}
}  // namespace

void initLEDs() {
  initRmt();
  forceClearLeds(8);
}

void startLEDBlink(CRGB color, unsigned long duration, int count) {
  if (!systemReady || lowBatteryWarning) return;
  currentBlinkColor = color;
  blinkDuration = duration;
  blinkInterval = duration;
  totalBlinkCount = count;
  useColorSequence = false;
  blinkCount = 0;
  blinkState = 0;
  pendingClearFrames = 0;
  previousIdleClearMillis = 0;
  isBlinking = true;
  previousBlinkMillis = millis();
}

void startColorSequenceBlink() {
  if (!systemReady || lowBatteryWarning) return;
  blinkDuration = 50;
  blinkInterval = 50;
  totalBlinkCount = 6;
  useColorSequence = true;
  blinkCount = 0;
  blinkState = 0;
  pendingClearFrames = 0;
  previousIdleClearMillis = 0;
  isBlinking = true;
  previousBlinkMillis = millis();
}

void stopLEDBlink() {
  isBlinking = false;
  blinkCount = 0;
  blinkState = 0;
  pendingClearFrames = 0;
  previousIdleClearMillis = 0;
  forceClearLeds(8);
}

void setLEDColor(CRGB color) {
  if (!systemReady || lowBatteryWarning) return;
  fillSolid(color);
  showLeds();
}

void setLowBatteryWarning(bool active) {
  if (lowBatteryWarning == active) return;
  lowBatteryWarning = active;
  isBlinking = false;
  blinkCount = 0;
  blinkState = 0;
  previousBreathingMillis = 0;
  breathingStartedMillis = millis();
  pendingClearFrames = 0;
  previousIdleClearMillis = 0;
  forceClearLeds(8);
}

void indicateSystemReady() {
  if (systemReady) return;
  systemReady = true;
  startLEDBlink(CRGB::Green, 50, 6);
}

void handleLEDBlink() {
  const unsigned long now = millis();
  if (lowBatteryWarning) {
    if (now - previousBreathingMillis < kBreathingFrameMs) return;
    previousBreathingMillis = now;
    const unsigned long halfPeriod = kBreathingPeriodMs / 2;
    const unsigned long phase =
        (now - breathingStartedMillis) % kBreathingPeriodMs;
    const unsigned long triangle =
        phase <= halfPeriod ? phase : kBreathingPeriodMs - phase;
    const uint8_t red = 8 + (uint32_t)247 * triangle / halfPeriod;
    fillSolid(CRGB(red, 0, 0));
    showLeds();
    return;
  }
  if (!isBlinking) {
    if (pendingClearFrames > 0) {
      --pendingClearFrames;
      forceClearLeds(1);
      previousIdleClearMillis = now;
      return;
    }
    if (now - previousIdleClearMillis >= 1000) {
      previousIdleClearMillis = now;
      forceClearLeds(1);
    }
    return;
  }
  const unsigned long currentMillis = now;
  const unsigned long interval = blinkState == 0 ? blinkDuration : blinkInterval;
  if (currentMillis - previousBlinkMillis < interval) return;

  previousBlinkMillis = currentMillis;
  if (blinkState == 0) {
    fillSolid(useColorSequence ? blinkColors[blinkCount % 6] : currentBlinkColor);
    showLeds();
    blinkState = 1;
    return;
  }

  fillSolid(CRGB::Black);
  showLeds();
  blinkState = 0;
  blinkCount++;
  if (blinkCount >= totalBlinkCount) {
    isBlinking = false;
    blinkCount = 0;
    blinkState = 0;
    pendingClearFrames = 30;
    previousIdleClearMillis = 0;
    forceClearLeds(8);
  }
}

bool isStatusLedBlinking() {
  return isBlinking;
}

bool isLowBatteryWarningActive() {
  return lowBatteryWarning;
}

uint8_t pendingStatusLedClearFrames() {
  return pendingClearFrames;
}

CRGB getStatusLedColor(int index) {
  if (index < 0 || index >= NUM_LEDS) return CRGB::Black;
  return leds[index];
}
