#pragma once

#include <Arduino.h>

#define RGB_PIN 21
#define NUM_LEDS 8

struct CRGB {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  constexpr CRGB(uint8_t red = 0, uint8_t green = 0, uint8_t blue = 0)
      : r(red), g(green), b(blue) {}

  static const CRGB Black;
  static const CRGB Red;
  static const CRGB Green;
  static const CRGB Blue;
  static const CRGB Yellow;
};

extern CRGB leds[NUM_LEDS];
extern unsigned long previousBlinkMillis;
extern int blinkState;
extern int blinkCount;
extern bool isBlinking;
extern CRGB blinkColors[6];
extern unsigned long blinkDuration;
extern unsigned long blinkInterval;
extern CRGB currentBlinkColor;
extern int totalBlinkCount;
extern bool useColorSequence;

void initLEDs();
void handleLEDBlink();
void startLEDBlink(CRGB color, unsigned long duration, int count = 3);
void stopLEDBlink();
void setLEDColor(CRGB color);
void startColorSequenceBlink();
void setLowBatteryWarning(bool active);
void indicateSystemReady();
bool isStatusLedBlinking();
bool isLowBatteryWarningActive();
uint8_t pendingStatusLedClearFrames();
CRGB getStatusLedColor(int index);
