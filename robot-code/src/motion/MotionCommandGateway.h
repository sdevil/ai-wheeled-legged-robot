#ifndef MOTION_COMMAND_GATEWAY_H
#define MOTION_COMMAND_GATEWAY_H

#include <Arduino.h>

#include "MotionCommand.h"

enum class ExternalMotionAction : uint8_t {
  RawMotionCommand,
  Move,
  Stop,
  Stand,
  Sit,
  LegHeight,
  LegLean
};

struct ExternalMotionRequest {
  ExternalMotionAction action = ExternalMotionAction::RawMotionCommand;
  MotionCommand command;
  int forward = 0;
  int turn = 0;
  int amplitude = 0;
  int angleDeg = 0;
  int speedPercent = 100;
  uint32_t durationMs = 0;
};

class MotionCommandGateway {
 public:
  void dispatch(const MotionCommand& command, const char* trigger,
                bool event = false);
  void dispatch(const MotionCommand& command, const String& trigger,
                bool event = false);
  void dispatch(const ExternalMotionRequest& request, const char* trigger,
                bool event = false);
  void dispatch(const ExternalMotionRequest& request, const String& trigger,
                bool event = false);
};

MotionCommandGateway& motionGateway();

#endif
