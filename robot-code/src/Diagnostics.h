#pragma once

#include <Arduino.h>

constexpr size_t DIAGNOSTIC_EVENT_CAPACITY = 24;
constexpr size_t DIAGNOSTIC_CATEGORY_LENGTH = 12;
constexpr size_t DIAGNOSTIC_MESSAGE_LENGTH = 72;

struct DiagnosticEvent {
  uint32_t timestampMs = 0;
  char category[DIAGNOSTIC_CATEGORY_LENGTH] = {};
  char message[DIAGNOSTIC_MESSAGE_LENGTH] = {};
};

void recordDiagnosticEvent(const char* category, const char* message);
void recordDiagnosticEvent(const char* category, const String& message);
size_t copyDiagnosticEvents(DiagnosticEvent* output, size_t capacity);

