#include "Diagnostics.h"

#include <string.h>

namespace {
DiagnosticEvent events[DIAGNOSTIC_EVENT_CAPACITY];
size_t writeIndex = 0;
size_t eventCount = 0;
char currentMotionTrigger[DIAGNOSTIC_TRIGGER_LENGTH] = "boot";
portMUX_TYPE diagnosticsMux = portMUX_INITIALIZER_UNLOCKED;
}

void recordDiagnosticEvent(const char* category, const char* message) {
  DiagnosticEvent event;
  event.timestampMs = millis();
  strlcpy(event.category, category ? category : "system",
          sizeof(event.category));
  strlcpy(event.message, message ? message : "", sizeof(event.message));

  portENTER_CRITICAL(&diagnosticsMux);
  events[writeIndex] = event;
  writeIndex = (writeIndex + 1) % DIAGNOSTIC_EVENT_CAPACITY;
  if (eventCount < DIAGNOSTIC_EVENT_CAPACITY) ++eventCount;
  portEXIT_CRITICAL(&diagnosticsMux);
}

void recordDiagnosticEvent(const char* category, const String& message) {
  recordDiagnosticEvent(category, message.c_str());
}

size_t copyDiagnosticEvents(DiagnosticEvent* output, size_t capacity) {
  if (!output || capacity == 0) return 0;
  portENTER_CRITICAL(&diagnosticsMux);
  const size_t count = min(eventCount, capacity);
  const size_t oldest =
      (writeIndex + DIAGNOSTIC_EVENT_CAPACITY - eventCount) %
      DIAGNOSTIC_EVENT_CAPACITY;
  const size_t skip = eventCount - count;
  for (size_t i = 0; i < count; ++i) {
    output[i] = events[(oldest + skip + i) % DIAGNOSTIC_EVENT_CAPACITY];
  }
  portEXIT_CRITICAL(&diagnosticsMux);
  return count;
}

void setMotionTrigger(const char* trigger) {
  portENTER_CRITICAL(&diagnosticsMux);
  strlcpy(currentMotionTrigger, trigger ? trigger : "",
          sizeof(currentMotionTrigger));
  portEXIT_CRITICAL(&diagnosticsMux);
}

void setMotionTrigger(const String& trigger) {
  setMotionTrigger(trigger.c_str());
}

void getMotionTrigger(char* output, size_t capacity) {
  if (!output || capacity == 0) return;
  portENTER_CRITICAL(&diagnosticsMux);
  strlcpy(output, currentMotionTrigger, capacity);
  portEXIT_CRITICAL(&diagnosticsMux);
}
