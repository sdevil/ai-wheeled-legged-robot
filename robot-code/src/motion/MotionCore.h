#ifndef MOTION_CORE_H
#define MOTION_CORE_H

#include <Arduino.h>
#include "MotionCommand.h"

struct MotionTelemetry {
  bool enabled = false;
  bool sitting = true;
  bool maintenance = false;
  bool tracking = false;
  float balanceAngleDeg = 0.0f;
  float pitchRateDeg = 0.0f;
  float linearPositionM = 0.0f;
  float linearVelocityMps = 0.0f;
  float yawRateDeg = 0.0f;
  int legHeightPercent = 50;
  int legLeanPercent = 0;
  int legLeanActualPercent = 0;
  int leftLegPosition = 0;
  int rightLegPosition = 0;
  int leftLegLoad = 0;
  int rightLegLoad = 0;
  const char* trackingState = "idle";
  int trackingProfile = 0;
  int trackingConfidence = 0;
  int trackingRawErrorX = 0;
  int trackingRawErrorY = 0;
  int trackingRawErrorZ = 0;
  float trackingErrorX = 0.0f;
  float trackingErrorY = 0.0f;
  float trackingErrorZ = 0.0f;
  float trackingYawCommand = 0.0f;
  float trackingDriveCommand = 0.0f;
  bool trackingYawEngaged = false;
  bool trackingDistanceEngaged = false;
  bool trackingBalanceReady = false;
  bool trackingSettleActive = false;
  uint32_t trackingObservationAgeMs = 0;
  uint32_t trackingControlAgeMs = 0;
  uint32_t trackingControlIntervalMs = 0;
  int trackingYawCandidateFrames = 0;
  int trackingDistanceCandidateFrames = 0;
  int trackingBoxX = 0;
  int trackingBoxY = 0;
  int trackingBoxW = 0;
  int trackingBoxH = 0;
  int trackingFrameW = 0;
  int trackingFrameH = 0;
  int trackingTargetH = 0;
  int trackingMissedFrames = 0;
  int trackingStableFrames = 0;
  int trackingRawScore = 0;
  int trackingVelocityX = 0;
  int trackingVelocityY = 0;
  const char* trackingDecision = "idle";
  float driveAxis = 0.0f;
  float steeringAxis = 0.0f;
  float linearReferenceMps = 0.0f;
  float yawReferenceRad = 0.0f;
  uint32_t controlLoopMaxGapUs = 0;
  uint32_t motorLoopMaxGapUs = 0;
  uint32_t motorLoopLateCount = 0;
  bool motorEnabled = false;
  float leftMotorVoltageQ = 0.0f;
  float rightMotorVoltageQ = 0.0f;
  bool standRecoverActive = false;
  uint32_t standRecoverElapsedMs = 0;
  float standRecoverDisplacementM = 0.0f;
  float standRecoverMaxDisplacementM = 0.0f;
  float standRecoverMinSignedDisplacementM = 0.0f;
  float standRecoverMaxSignedDisplacementM = 0.0f;
  uint32_t standRecoverPeakTimeMs = 0;
  float standRecoverCorrectionMps = 0.0f;
  const char* mode = "boot";
  const char* core = "unknown";
};

class MotionCore {
 public:
  virtual ~MotionCore() = default;
  virtual void begin() = 0;
  virtual void update() = 0;
  virtual void command(const MotionCommand& command) = 0;
  virtual MotionTelemetry telemetry() const = 0;
  virtual const char* name() const = 0;
};

#endif
