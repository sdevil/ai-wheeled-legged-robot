#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H

#include <Arduino.h>

enum class TrackObservationState : uint8_t {
  Idle,
  Acquiring,
  Locked,
  Coasting,
  Reacquiring,
  Lost
};

enum class MotionCommandType : uint8_t {
  None,
  Stand,
  Sit,
  ResetPose,
  Move,
  Stop,
  JumpPlace,
  JumpForward,
  JumpBackward,
  JumpLeft,
  JumpRight,
  TrackStart,
  TrackStop,
  TrackTarget,
  TrackObservation,
  LegLean,
  LegHeight,
  LegHeightPercent,
  GuardServo,
  MaintenanceEnter,
  MaintenanceExit
};

struct MotionCommand {
  MotionCommandType type = MotionCommandType::None;
  int x = 0;
  int y = 0;
  int z = 9999;
  int width = 0;
  int height = 0;
  int confidence = 0;
  int trackingProfile = 0;
  int targetBoxX = 0;
  int targetBoxY = 0;
  int targetBoxW = 0;
  int targetBoxH = 0;
  int frameWidth = 0;
  int frameHeight = 0;
  int targetHeight = 0;
  int missedFrames = 0;
  int stableFrames = 0;
  int rawScore = 0;
  int velocityX = 0;
  int velocityY = 0;
  TrackObservationState trackState = TrackObservationState::Idle;

  static MotionCommand simple(MotionCommandType type) {
    MotionCommand command;
    command.type = type;
    return command;
  }

  static MotionCommand move(int joyX, int joyY) {
    MotionCommand command;
    command.type = MotionCommandType::Move;
    command.x = joyX;
    command.y = joyY;
    return command;
  }

  static MotionCommand legLean(int percent) {
    MotionCommand command;
    command.type = MotionCommandType::LegLean;
    command.x = percent;
    return command;
  }

  static MotionCommand legHeight(int direction) {
    MotionCommand command;
    command.type = MotionCommandType::LegHeight;
    command.y = constrain(direction, -1, 1);
    return command;
  }

  static MotionCommand legHeightPercent(int percent) {
    MotionCommand command;
    command.type = MotionCommandType::LegHeightPercent;
    command.y = constrain(percent, 0, 100);
    return command;
  }
  static MotionCommand guardServo(int angleDeg) {
    MotionCommand command;
    command.type = MotionCommandType::GuardServo;
    command.x = constrain(angleDeg, 0, 180);
    return command;
  }

  static MotionCommand trackTarget(int dx, int dy, int dz) {
    MotionCommand command;
    command.type = MotionCommandType::TrackTarget;
    command.x = dx;
    command.y = dy;
    command.z = dz;
    return command;
  }
  static MotionCommand trackObservation(TrackObservationState state, int dx,
                                        int dy, int dz, int confidence,
                                        int profile = 0) {
    MotionCommand command;
    command.type = MotionCommandType::TrackObservation;
    command.trackState = state;
    command.x = dx;
    command.y = dy;
    command.z = dz;
    command.confidence = confidence;
    command.trackingProfile = constrain(profile, 0, 3);
    return command;
  }
};

#endif
