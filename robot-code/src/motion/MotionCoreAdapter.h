#ifndef MOTION_CORE_ADAPTER_H
#define MOTION_CORE_ADAPTER_H

#include "MotionCore.h"

class MotionCoreAdapter final : public MotionCore {
 public:
  void begin() override;
  void update() override;
  void command(const MotionCommand& command) override;
  MotionTelemetry telemetry() const override;
  const char* name() const override { return "motion-core"; }
  bool selfCheckPassed() const;

 private:
  void pulseButton(uint16_t button, uint32_t durationMs = 80);
  void setMoveAxes(int joyX, int joyY);
  void stopMove();
  void holdTrackingChassis(bool resetReference = false);
  void applyTrackTarget(int dx, int dy, int dz);
  void applyTrackObservation(const MotionCommand& command);
  void enterTrackingState(TrackObservationState state);
  void updateTrackingMotion(uint32_t now);
  void updateStandNudge(uint32_t now);
  void updateLegHeightTarget(uint32_t now);
  void updateCameraGimbal(uint32_t now);
  void updateCameraCalibration(uint32_t now);
  int legHeightPercent() const;
  int legLeanPercent() const;
  float legHeightBaseFromPercent(int percent) const;
  const char* modeName() const;
  const char* trackingStateName() const;

  bool started_ = false;
  bool maintenance_ = false;
  bool tracking_ = false;
  bool trackHasLockedTarget_ = false;
  uint16_t pulseButtons_ = 0;
  uint16_t heldPostureButtons_ = 0;
  uint32_t pulseUntilMs_ = 0;
  uint32_t trackSettleUntilMs_ = 0;
  uint32_t trackStateSinceMs_ = 0;
  uint32_t lastTrackObservationMs_ = 0;
  uint32_t lastTrackMotionUpdateMs_ = 0;
  uint32_t lastTrackGimbalUpdateMs_ = 0;
  uint32_t lastTrackControlUpdateMs_ = 0;
  uint32_t trackBalanceReadySinceMs_ = 0;
  uint32_t trackChassisHoldUntilMs_ = 0;
  uint32_t standNudgeBalanceSinceMs_ = 0;
  uint32_t standNudgeUntilMs_ = 0;
  TrackObservationState trackState_ = TrackObservationState::Idle;
  float axes_[6] = {};
  float trackYawTarget_ = 0.0f;
  float trackDriveTarget_ = 0.0f;
  int rawTrackDx_ = 0;
  int rawTrackDy_ = 0;
  int rawTrackDz_ = 0;
  float filteredTrackDx_ = 0.0f;
  float filteredTrackDy_ = 0.0f;
  float filteredTrackDz_ = 0.0f;
  int trackBoxX_ = 0;
  int trackBoxY_ = 0;
  int trackBoxW_ = 0;
  int trackBoxH_ = 0;
  int trackFrameW_ = 0;
  int trackFrameH_ = 0;
  int trackTargetH_ = 0;
  int trackMissedFrames_ = 0;
  int trackStableFrames_ = 0;
  int trackRawScore_ = 0;
  int trackVelocityX_ = 0;
  int trackVelocityY_ = 0;
  const char* trackDecision_ = "idle";
  int trackConfidence_ = 0;
  int trackProfile_ = 0;
  bool trackYawEngaged_ = false;
  bool trackYawReversePending_ = false;
  bool trackDistanceEngaged_ = false;
  int trackYawCandidateDirection_ = 0;
  uint8_t trackYawCandidateFrames_ = 0;
  int trackDistanceCandidateDirection_ = 0;
  uint8_t trackDistanceCandidateFrames_ = 0;
  int lastTargetDirection_ = 1;
  float searchCameraCenterAngle_ = 105.0f;
  float cameraAngle_ = 105.0f;
  float cameraTargetAngle_ = 105.0f;
  uint32_t lastCameraGimbalUpdateMs_ = 0;
  bool cameraCalibrationActive_ = false;
  uint32_t cameraCalibrationStartedMs_ = 0;
  bool legHeightTargetActive_ = false;
  bool standNudgePending_ = false;
  float legHeightBaseTarget_ = 0.0f;
  uint32_t lastLegHeightUpdateMs_ = 0;
};

MotionCoreAdapter& motionCore();

#endif
