#include "MotionCoreAdapter.h"

#include <math.h>
#include <string.h>

#include "controller/controller.h"
#include "Diagnostics.h"
#include "devices/ptk7350.h"
#include "motion_buttons.h"
#include "system/task.h"

namespace {
constexpr float kTrackYawEngageError = 80.0f;
constexpr float kTrackYawReleaseError = 38.0f;
constexpr float kTrackDriveAlignError = 300.0f;
constexpr float kTrackDistanceEngageError = 16.0f;
constexpr float kTrackDistanceReleaseError = 8.0f;
constexpr float kTrackPitchDeadband = 45.0f;
constexpr float kTrackYawScale = 430.0f;
constexpr float kTrackDistanceScale = 340.0f;
constexpr float kTrackFilterAlpha = 0.78f;
constexpr float kTrackMaxYaw = 0.26f;
constexpr float kTrackMaxDrive = 0.42f;
constexpr float kTrackYawAccelPerSecond = 3.20f;
constexpr float kTrackYawDecelPerSecond = 6.00f;
constexpr float kTrackDriveSlewPerSecond = 4.00f;
constexpr float kTrackMinimumConfidence = 460.0f;
constexpr float kTrackYawRateDamping = 0.075f;
constexpr float kTrackYawIntegralDecaySeconds = 0.05f;
constexpr uint8_t kTrackYawEngageFrames = 3;
constexpr uint8_t kTrackYawReverseFrames = 3;
constexpr uint8_t kTrackDistanceEngageFrames = 2;
constexpr float kTrackYawPolarity = 1.0f;
constexpr float kTrackDrivePolarity = 1.0f;
constexpr float kLegHeightBaseMin = -4.0f;
constexpr float kLegHeightBaseMax = 52.0f;
constexpr float kLegHeightControlMin = -1.0f;
constexpr float kLegHeightControlMax = 49.0f;
constexpr float kLegHeightSlewPerSecond = 42.0f;
constexpr uint32_t kLegHeightForceSyncHoldMs = 220;
constexpr int kDefaultLegHeightPercent = 55;
constexpr int kLegHeightStableMaxPercent = 90;
constexpr int kLegLeanStableMaxPercent = 72;
constexpr int kHighLegLeanReduceStartPercent = 78;
constexpr int kHighLegLeanMinPercent = 20;
constexpr int kHighLegSyncStartPercent = 88;
constexpr int kDefaultGuardAngleDeg = 0;
constexpr uint32_t kTrackSettleMs = 300;
constexpr uint32_t kTrackBalanceStableMs = 500;
constexpr uint32_t kTrackCommandTimeoutMs = 2500;
constexpr uint32_t kTrackControlIntervalMs = 80;
constexpr uint32_t kSearchTurnStartMs = 3800;
constexpr uint32_t kSearchTurnEndMs = 4600;
constexpr uint32_t kSearchGiveUpMs = 6200;
constexpr uint32_t kGimbalUpdateIntervalMs = 20;
constexpr uint32_t kSearchGimbalIntervalMs = 80;
constexpr float kSearchGimbalAmplitudeDeg = 18.0f;
constexpr float kSearchChassisYaw = 0.025f;
constexpr uint32_t kStandNudgeStableMs = 250;
constexpr uint32_t kStandNudgeDurationMs = 0;
constexpr float kStandNudgeAxis = 0.0f;
constexpr uint32_t kDanceCueDurationMs = 1000;
constexpr uint32_t kDanceDemoDurationMs = 120000;
constexpr uint32_t kDanceBalanceReadyMs = 450;
constexpr uint32_t kDanceStandTriggerMs = 5000;

struct DanceCue {
  uint16_t durationMs;
  int leanPercent;
  int heightPercent;
  int steerPercent;
  int drivePercent;
  int guardAngleDeg;
};

constexpr DanceCue kDanceDemoCues[] = {
    // 0:00 - 0:15 ACT I — Entrance / Awakening
    {1000,  0, 46,   0,  0,  0}, {1000,  0, 46,   0,  0,  0}, {1000,  0, 46,   0,  0,  0},
    {1000,  0, 46,   0,  0,  0}, {1000,  0, 46,   0,  0, 15}, {1000,  0, 49,   0,  0, 15},
    {1000,  0, 52,   0,  0, 15}, {1000,  0, 55,   0,  0, 15}, {1000,  0, 59,   0,  0, 20},
    {1000,  0, 60,   0,  0, 20}, {1000,-12, 60,   0,  0, 20}, {1000,  0, 57,   0,  0, 20},
    {1000, 12, 57,   0,  0, 20}, {1000,  0, 57,   0,  0, 25}, {1000,  0, 55,   0,  0, 25},

    // 0:15 - 0:35 ACT II — First Waltz Breath
    {1000,  0, 62,   0,  0, 25}, {1000,  0, 58,   0,  0, 25}, {1000,-18, 55,   0,  0, 25},
    {1000,-18, 52,   0,  0, 25}, {1000,-10, 52,   0,  0, 25}, {1000,  0, 55,  10,  0, 25},
    {1000, 18, 58,  10,  0, 25}, {1000, 18, 55,   8,  0, 25}, {1000, 10, 55,   6,  0, 30},
    {1000,  0, 55,   4,  0, 30}, {1000,  0, 60, -10,  0, 30}, {1000,  0, 60,-10,  0, 30},
    {1000,  0, 58,  10,  0, 30}, {1000,  0, 55,  10,  0, 30}, {1000,-15, 57,   0,  0, 30},
    {1000, 15, 57,   0,  0, 30}, {1000,  0, 55,   0,  0, 25}, {1000,  0, 55,   0,  0, 25},
    {1000,  0, 55,   0,  0, 25}, {1000,  0, 55,   0,  0, 25},

    // 0:35 - 0:55 ACT III — Promenade Glide
    {1000,  0, 60,   6, 16, 25}, {1000,  0, 60,   6, 16, 25}, {1000,  0, 58,   6, 14, 25},
    {1000,  0, 55,   4, 10, 25}, {1000,  0, 58, -18,  6, 25}, {1000,-18, 60,-18,  6, 25},
    {1000,-18, 58,-18,  4, 25}, {1000,-10, 55,-12,  0, 25}, {1000, 18, 55, 18,  4, 25},
    {1000, 18, 58, 18,  4, 25}, {1000, 10, 60, 18,  2, 25}, {1000,  0, 58, 10,  0, 25},
    {1000,  0, 57,   6,  8, 30}, {1000,  0, 60,   6, 10, 35}, {1000,  0, 58,   4,  8, 35},
    {1000,  0, 55,   0,  0, 35}, {1000,  0, 60,   0,  0, 35}, {1000,-12, 60,   0,  0, 35},
    {1000,  0, 58,   0,  0, 35}, {1000,  0, 55,   0,  0, 35},

    // 0:55 - 1:15 ACT IV — Waltz Turn Sequence
    {1000,-18, 62,-26,  0, 35}, {1000,-20, 60,-26,  0, 35}, {1000,-12, 58,-26,  0, 35},
    {1000,  0, 55,-20,  0, 35}, {1000, 18, 55, 26,  0, 35}, {1000, 20, 58, 26,  0, 35},
    {1000, 10, 60, 26,  0, 35}, {1000,  0, 58, 20,  0, 35}, {1000,  0, 60,-14,  0, 35},
    {1000,  0, 60,-14,  0, 35}, {1000,  0, 58, 14,  0, 35}, {1000,  0, 58, 14,  0, 35},
    {1000,  0, 60,   0,  0, 35}, {1000,  0, 60,-30,  0, 35}, {1000,  0, 60, 30,  0, 35},
    {1000,  0, 58,   0,  0, 35}, {1000,  0, 55,   0,  0, 30}, {1000,  0, 55,   0,  0, 25},
    {1000,  0, 55,   0,  0, 25}, {1000,  0, 55,   0,  0, 25},

    // 1:15 - 1:35 ACT V — Spin Flourish
    {1000,  0, 64,   0,  0, 25}, {1000,  0, 64,-78,  0, 25}, {1000,  0, 62,-78,  0, 25},
    {1000,  0, 58,   0,  0, 25}, {1000,  0, 58,   0,  0, 25}, {1000,  0, 60, 26,  0, 25},
    {1000, 18, 58, 26,  0, 25}, {1000, 18, 55, 18,  0, 25}, {1000,  0, 55,-26,  0, 25},
    {1000,-18, 58,-26,  0, 25}, {1000,-18, 55,-18,  0, 25}, {1000,  0, 58,   0,  0, 25},
    {1000,  0, 64,  78,  0, 25}, {1000,  0, 64,  78,  0, 25}, {1000,  0, 60,   0,  0, 25},
    {1000,  0, 60,   0,  0, 40}, {1000,  0, 60,   0,  0, 40}, {1000,  0, 58,   0,  0, 35},
    {1000,  0, 58,   0,  0, 30}, {1000,  0, 58,   0,  0, 30},

    // 1:35 - 1:55 ACT VI — Elegant Return / Bow Preparation
    {1000,  0, 60,   4, 10, 30}, {1000,  0, 60,   4, 10, 30}, {1000,  0, 58,   4, 10, 30},
    {1000,  0, 55,   0,  0, 30}, {1000,-15, 55,-12,  0, 30}, {1000,-15, 55,-12,  0, 30},
    {1000, 15, 55, 12,  0, 30}, {1000, 15, 55, 12,  0, 30}, {1000,  0, 55,   0,  0, 30},
    {1000,  0, 60,   0,  0, 30}, {1000,  0, 60,   0,  0, 35}, {1000,  0, 58,   0,  0, 45},
    {1000,  0, 55,   0,  0, 45}, {1000,  0, 57,   4,  8, 45}, {1000,  0, 58,   0,  0, 45},
    {1000,  0, 58,   0,  0, 35}, {1000,  0, 57,   0,  0, 30}, {1000,  0, 57,   0,  0, 30},
    {1000,  0, 57,   0,  0, 30}, {1000,  0, 57,   0,  0, 30},

    // 1:55 - 2:00 ACT VII — Final Bow / Presentation
    {1000,  0, 60,   0,  0, 45}, {1000,  0, 60,   0,  0, 45}, {1000,  0, 52,   0,  0, 45},
    {1000,  0, 55,   0,  0, 45}, {1000,  0, 58,   0,  0, 35}, {1000,  0, 58,   0,  0, 30},
    {1000,  0, 58,   0,  0, 25}, {1000,  0, 58,   0,  0, 25}, {1000,  0, 58,   0,  0, 25},
    {1000,  0, 58,   0,  0, 25}, {1000,  0, 58,   0,  0, 25}, {1000,  0, 58,   0,  0, 25},
};

struct TrackingTuning {
  float yawScale;
  float distanceScale;
  float filterAlpha;
  float maxYaw;
  float maxDrive;
  float yawAccel;
  float driveSlew;
  float minimumConfidence;
  float searchYaw;
};

TrackingTuning trackingTuning(int profile) {
  switch (profile) {
    case 3:  // Ping-pong ball: prioritize fast interception.
      return {250.0f, 230.0f, 0.36f, 0.50f, 0.62f,
              5.40f, 5.80f, 340.0f, 0.13f};
    case 2:  // Animal.
      return {320.0f, 285.0f, 0.84f, 0.40f, 0.52f,
              4.30f, 4.80f, 400.0f, 0.09f};
    case 1:  // Face/person: moderate response; avoid vision-balance feedback oscillation.
      return {420.0f, 520.0f, 0.45f, 0.24f, 0.18f,
              4.20f, 3.20f, 560.0f, 0.0f};
    default:  // Unknown manually selected object.
      return {kTrackYawScale, kTrackDistanceScale, kTrackFilterAlpha,
              kTrackMaxYaw, kTrackMaxDrive, kTrackYawAccelPerSecond,
              kTrackDriveSlewPerSecond, kTrackMinimumConfidence, 0.055f};
  }
}

float approach(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

volatile uint32_t controlLoopMaxGapUs = 0;
volatile uint32_t motorLoopMaxGapUs = 0;
volatile uint32_t motorLoopLateCount = 0;

void controlPipelineTask(uint32_t tick) {
  static uint32_t previousUs = 0;
  const uint32_t nowUs = micros();
  if (previousUs != 0) {
    const uint32_t gapUs = nowUs - previousUs;
    if (gapUs > controlLoopMaxGapUs) controlLoopMaxGapUs = gapUs;
  }
  previousUs = nowUs;

  // Keep sampling and control calculations ordered on the real-time core.
  controller::sensor_update_proc(tick);
  controller::loop_proc(tick);
}

void motorUpdateTask(uint32_t tick) {
  static uint32_t previousUs = 0;
  const uint32_t nowUs = micros();
  if (previousUs != 0) {
    const uint32_t gapUs = nowUs - previousUs;
    if (gapUs > motorLoopMaxGapUs) motorLoopMaxGapUs = gapUs;
    if (gapUs > 2500) ++motorLoopLateCount;
  }
  previousUs = nowUs;
  controller::motor_update_proc(tick);
}
}  // namespace

MotionCoreAdapter& motionCore() {
  static MotionCoreAdapter instance;
  return instance;
}

void MotionCoreAdapter::begin() {
  if (started_) return;
  started_ = true;
  ctrl.init();
  setGuardServoAngle(kDefaultGuardAngleDeg);

  // WiFi and the web server run on core 0. Keep the full motion pipeline on
  // core 1 so network activity cannot interrupt FOC output. Electrical angle
  // sampling is 1 kHz, so a deterministic 1 kHz motor task retains all useful
  // control bandwidth from the former 2 kHz shared esp_timer callback.
  static task controlTask(1, controlPipelineTask, 8192, 5, 1);
  static task motorTask(1, motorUpdateTask, 4096, 6, 1);
  controlTask.start();
  motorTask.start();
}

void MotionCoreAdapter::update() {
  if (!started_) return;
  const uint32_t now = millis();
  if (pulseButtons_ && deadlineReached(now, pulseUntilMs_)) pulseButtons_ = 0;
  updateDanceDemo(now);
  if (defaultStandPosePending_ &&
      ctrl.fsm_state_machine.mode == fsm::mode_state::BALANCE) {
    defaultStandPosePending_ = false;
    setLegHeightTargetPercent(kDefaultLegHeightPercent);
  }
  updateLegHeightTarget(now);
  updateStandNudge(now);

  ctrl.host_data.buttons = pulseButtons_ | heldPostureButtons_;
  memcpy(ctrl.host_data.axes, axes_, sizeof(axes_));
}

void MotionCoreAdapter::command(const MotionCommand& command) {
  const bool isDanceCommand =
      command.type == MotionCommandType::DanceDemoStart ||
      command.type == MotionCommandType::DanceDemoStop;
  if (!isDanceCommand && (danceDemoActive_ || danceDemoQueued_)) {
    stopDanceDemo(true);
  }

  switch (command.type) {
    case MotionCommandType::Stand:
      maintenance_ = false;
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      legHeightTargetActive_ = false;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      heldPostureButtons_ = 0;
      ctrl.roll_adjust_target = 0.0f;
      ctrl.leg_lean = 0.0f;
      ctrl.leg_lean_target = 0.0f;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      armDefaultStandPose();
      pulseButton(BTN_RB, 120);
      break;
    case MotionCommandType::Sit:
      tracking_ = false;
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      legHeightTargetActive_ = false;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      heldPostureButtons_ = 0;
      enterTrackingState(TrackObservationState::Idle);
      ctrl.roll_adjust_target = 0.0f;
      ctrl.leg_lean = 0.0f;
      ctrl.leg_lean_target = 0.0f;
      stopMove();
      pulseButton(BTN_LB, 120);
      break;
    case MotionCommandType::ResetPose:
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      ctrl.roll_adjust_target = 0.0f;
      ctrl.leg_lean = 0.0f;
      ctrl.leg_lean_target = 0.0f;
      heldPostureButtons_ = 0;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      setLegHeightTargetPercent(kDefaultLegHeightPercent);
      setGuardServoAngle(kDefaultGuardAngleDeg);
      break;
    case MotionCommandType::Move:
      if (!maintenance_) {
        standNudgePending_ = false;
        standNudgeBalanceSinceMs_ = 0;
        standNudgeUntilMs_ = 0;
        setMoveAxes(command.x, command.y);
      }
      break;
    case MotionCommandType::Stop:
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      stopMove();
      break;
    case MotionCommandType::JumpPlace:
      pulseButton(BTN_RS, 120);
      break;
    case MotionCommandType::JumpForward:
      pulseButton(BTN_Y, 120);
      break;
    case MotionCommandType::JumpBackward:
      pulseButton(BTN_A, 120);
      break;
    case MotionCommandType::JumpLeft:
      pulseButton(BTN_X, 120);
      break;
    case MotionCommandType::JumpRight:
      pulseButton(BTN_B, 120);
      break;
    case MotionCommandType::TrackStart:
      tracking_ = true;
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      legHeightTargetActive_ = false;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      trackHasLockedTarget_ = false;
      trackReturnScanSent_ = false;
      trackChassisHoldUntilMs_ = 0;
      lastTrackObservationMs_ = 0;
      trackBalanceReadySinceMs_ = 0;
      lastTrackControlUpdateMs_ = 0;
      trackSettleUntilMs_ = millis() + kTrackSettleMs;
      filteredTrackDx_ = 0.0f;
      filteredTrackDy_ = 0.0f;
      filteredTrackDz_ = 0.0f;
      trackYawEngaged_ = false;
      trackYawReversePending_ = false;
      trackDistanceEngaged_ = false;
      trackYawCandidateDirection_ = 0;
      trackYawCandidateFrames_ = 0;
      trackDistanceCandidateDirection_ = 0;
      trackDistanceCandidateFrames_ = 0;
      trackProfile_ = 0;
      trackDecision_ = "camera_motion_disabled";
      enterTrackingState(TrackObservationState::Acquiring);
      break;
    case MotionCommandType::TrackStop:
      tracking_ = false;
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      legHeightTargetActive_ = false;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      trackHasLockedTarget_ = false;
      trackReturnScanSent_ = false;
      lastTrackObservationMs_ = 0;
      trackBalanceReadySinceMs_ = 0;
      lastTrackControlUpdateMs_ = 0;
      trackProfile_ = 0;
      enterTrackingState(TrackObservationState::Idle);
      trackDecision_ = "idle";
      break;
    case MotionCommandType::DanceDemoStart:
      startDanceDemo(millis());
      break;
    case MotionCommandType::DanceDemoStop:
      stopDanceDemo(true);
      break;
    case MotionCommandType::TrackTarget:
      if (tracking_ && !maintenance_) {
        trackHasLockedTarget_ = true;
        trackReturnScanSent_ = false;
        enterTrackingState(TrackObservationState::Locked);
        lastTrackObservationMs_ = millis();
        rawTrackDx_ = constrain(command.x * 1000 / 320, -1000, 1000);
        rawTrackDy_ = constrain(command.y * 1000 / 240, -1000, 1000);
        rawTrackDz_ = command.z == 9999 ? 9999 : constrain(command.z * 6, -1000, 1000);
        filteredTrackDx_ = rawTrackDx_;
        filteredTrackDy_ = rawTrackDy_;
        filteredTrackDz_ = rawTrackDz_;
        trackYawTarget_ = 0.0f;
        trackDriveTarget_ = 0.0f;
        trackYawEngaged_ = false;
        trackDistanceEngaged_ = false;
        trackDecision_ = "camera_motion_disabled";
      }
      break;
    case MotionCommandType::TrackObservation:
      if (tracking_ && !maintenance_) applyTrackObservation(command);
      break;
    case MotionCommandType::LegLean:
      if (!maintenance_) {
        const int dynamicLimit = currentLegLeanLimitPercent();
        const int stableLeanPercent =
            constrain(command.x, -dynamicLimit, dynamicLimit);
        ctrl.leg_lean_target =
            constrain((float)stableLeanPercent / 100.0f, -1.0f, 1.0f);
      }
      break;
    case MotionCommandType::LegHeight:
      if (command.y == 0) {
        legHeightTargetPercent_ = -1;
        heldPostureButtons_ = 0;
        legHeightForceSync_ = false;
        legHeightForceSyncHoldUntilMs_ = 0;
        ctrl.force_sync_leg_motion = 0;
        ctrl.symmetric_leg_motion = 0;
      } else if (!maintenance_) {
        defaultStandPosePending_ = false;
        legHeightTargetPercent_ = -1;
        legHeightTargetActive_ = false;
        legHeightForceSync_ = false;
        legHeightForceSyncHoldUntilMs_ = 0;
        ctrl.force_sync_leg_motion = 0;
        ctrl.symmetric_leg_motion = 1;
        heldPostureButtons_ = command.y > 0 ? BTN_UP : BTN_DOWN;
      }
      break;
    case MotionCommandType::LegHeightPercent:
      if (!maintenance_) {
        defaultStandPosePending_ = false;
        setLegHeightTargetPercent(command.y);
      }
      break;
    case MotionCommandType::GuardServo:
      setGuardServoAngle(command.x);
      break;
    case MotionCommandType::MaintenanceEnter:
      maintenance_ = true;
      tracking_ = false;
      defaultStandPosePending_ = false;
      legHeightTargetPercent_ = -1;
      legHeightTargetActive_ = false;
      legHeightForceSync_ = false;
      legHeightForceSyncHoldUntilMs_ = 0;
      ctrl.force_sync_leg_motion = 0;
      ctrl.symmetric_leg_motion = 0;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      enterTrackingState(TrackObservationState::Idle);
      stopMove();
      break;
    case MotionCommandType::MaintenanceExit:
      maintenance_ = false;
      break;
    case MotionCommandType::None:
      break;
  }
}

MotionTelemetry MotionCoreAdapter::telemetry() const {
  MotionTelemetry data;
  data.maintenance = maintenance_;
  data.tracking = tracking_;
  data.balanceAngleDeg = ctrl.lqi_param.state.pitch_angle * 180.0f / PI;
  data.pitchRateDeg = ctrl.lqi_param.state.pitch_rate * 180.0f / PI;
  data.linearPositionM = ctrl.lqi_param.state.avg_linear_pos;
  data.linearVelocityMps = ctrl.lqi_param.state.avg_linear_vel;
  data.yawRateDeg = ctrl.lqi_param.state.yaw_rate * 180.0f / PI;
  data.legHeightPercent = legHeightPercent();
  data.legLeanPercent = legLeanPercent();
  data.legLeanActualPercent =
      constrain((int)roundf(ctrl.leg_lean * 100.0f), -100, 100);
  data.guardAngleDeg = guardServoAngleDeg_;
  data.leftLegPosition = sts_servo_state[0].position;
  data.rightLegPosition = sts_servo_state[1].position;
  data.leftLegLoad = sts_servo_state[0].load;
  data.rightLegLoad = sts_servo_state[1].load;
  data.trackingState = trackingStateName();
  data.trackingProfile = trackProfile_;
  data.trackingConfidence = trackConfidence_;
  data.trackingRawErrorX = rawTrackDx_;
  data.trackingRawErrorY = rawTrackDy_;
  data.trackingRawErrorZ = rawTrackDz_;
  data.trackingErrorX = filteredTrackDx_;
  data.trackingErrorY = filteredTrackDy_;
  data.trackingErrorZ = filteredTrackDz_;
  data.trackingYawCommand = trackYawTarget_;
  data.trackingDriveCommand = trackDriveTarget_;
  data.trackingYawEngaged = trackYawEngaged_;
  data.trackingDistanceEngaged = trackDistanceEngaged_;
  data.trackingBalanceReady = trackBalanceReadySinceMs_ != 0 && millis() - trackBalanceReadySinceMs_ >= kTrackBalanceStableMs;
  data.trackingSettleActive = !deadlineReached(millis(), trackSettleUntilMs_);
  data.trackingObservationAgeMs = lastTrackObservationMs_ == 0 ? 0 : millis() - lastTrackObservationMs_;
  data.trackingControlAgeMs = lastTrackControlUpdateMs_ == 0 ? 0 : millis() - lastTrackControlUpdateMs_;
  data.trackingControlIntervalMs =
      trackProfile_ == 3 ? 60U : (trackProfile_ == 1 ? 45U : kTrackControlIntervalMs);
  data.trackingYawCandidateFrames = trackYawCandidateFrames_;
  data.trackingDistanceCandidateFrames = trackDistanceCandidateFrames_;
  data.trackingBoxX = trackBoxX_;
  data.trackingBoxY = trackBoxY_;
  data.trackingBoxW = trackBoxW_;
  data.trackingBoxH = trackBoxH_;
  data.trackingFrameW = trackFrameW_;
  data.trackingFrameH = trackFrameH_;
  data.trackingTargetH = trackTargetH_;
  data.trackingMissedFrames = trackMissedFrames_;
  data.trackingStableFrames = trackStableFrames_;
  data.trackingRawScore = trackRawScore_;
  data.trackingVelocityX = trackVelocityX_;
  data.trackingVelocityY = trackVelocityY_;
  data.trackingDecision = trackDecision_;
  data.driveAxis = axes_[3];
  data.steeringAxis = axes_[0];
  data.linearReferenceMps = ctrl.lqi_param.ref.linear_vel;
  data.yawReferenceRad = ctrl.lqi_param.ref.yaw_rate;
  data.controlLoopMaxGapUs = controlLoopMaxGapUs;
  data.motorLoopMaxGapUs = motorLoopMaxGapUs;
  data.motorLoopLateCount = motorLoopLateCount;
  data.motorEnabled = ctrl.enable_motor;
  data.leftMotorVoltageQ = left_motor.voltage.q;
  data.rightMotorVoltageQ = right_motor.voltage.q;
  data.standRecoverActive = ctrl.balance_recover_active != 0;
  data.standRecoverElapsedMs = ctrl.balance_recover_active
                                   ? ctrl.balance_recover_timer
                                   : ctrl.balance_recover_last_elapsed;
  data.standRecoverDisplacementM = ctrl.balance_recover_displacement;
  data.standRecoverMaxDisplacementM = ctrl.balance_recover_max_displacement;
  data.standRecoverMinSignedDisplacementM = ctrl.balance_recover_min_signed_displacement;
  data.standRecoverMaxSignedDisplacementM = ctrl.balance_recover_max_signed_displacement;
  data.standRecoverPeakTimeMs = ctrl.balance_recover_peak_time;
  data.standRecoverCorrectionMps = ctrl.balance_recover_position_correction;
  data.danceActive = danceDemoQueued_ || danceDemoActive_;
  data.danceElapsedMs =
      danceDemoActive_ ? millis() - danceDemoStartedMs_ : 0;
  data.danceCueIndex = danceCueIndex_;
  data.mode = modeName();
  data.core = name();
  data.enabled = ctrl.fsm_state_machine.mode == fsm::mode_state::BALANCE ||
                 ctrl.fsm_state_machine.mode == fsm::mode_state::JUMP;
  data.sitting =
      (ctrl.fsm_state_machine.mode == fsm::mode_state::SIT &&
       ctrl.fsm_state_machine.sit == fsm::sit_state::DONE) ||
      ctrl.fsm_state_machine.mode == fsm::mode_state::FIRST_BOOT;
  return data;
}

bool MotionCoreAdapter::selfCheckPassed() const {
  if (!started_ ||
      ctrl.fsm_state_machine.mode == fsm::mode_state::ERROR) {
    return false;
  }

  const float accelerationMagnitude = sqrtf(
      mpu6050_dev.acc[0] * mpu6050_dev.acc[0] +
      mpu6050_dev.acc[1] * mpu6050_dev.acc[1] +
      mpu6050_dev.acc[2] * mpu6050_dev.acc[2]);
  const bool imuReady = isfinite(mpu6050_dev.temperature) &&
                        isfinite(mpu6050_dev.angle[0]) &&
                        isfinite(mpu6050_dev.angle[1]) &&
                        isfinite(accelerationMagnitude) &&
                        mpu6050_dev.temperature > -20.0f &&
                        mpu6050_dev.temperature < 100.0f &&
                        accelerationMagnitude > 0.5f &&
                        accelerationMagnitude < 1.5f;
  const bool servosReady = sts_servo_state[0].position > 1000 &&
                           sts_servo_state[0].position < 3000 &&
                           sts_servo_state[1].position > 1000 &&
                           sts_servo_state[1].position < 3000;
  return imuReady && servosReady;
}

float MotionCoreAdapter::legHeightBaseFromPercent(int percent) const {
  const float normalized = constrain((float)percent / 100.0f, 0.0f, 1.0f);
  return kLegHeightControlMax -
         normalized * (kLegHeightControlMax - kLegHeightControlMin);
}

int MotionCoreAdapter::legHeightPercent() const {
  const float normalized =
      (kLegHeightControlMax - ctrl.leg_height_base) /
      (kLegHeightControlMax - kLegHeightControlMin);
  return constrain((int)roundf(normalized * 100.0f), 0, 100);
}

int MotionCoreAdapter::legLeanPercent() const {
  return constrain((int)roundf(ctrl.leg_lean_target * 100.0f), -100, 100);
}

int MotionCoreAdapter::currentLegLeanLimitPercent() const {
  const int currentHeightPercent =
      max(legHeightPercent(), max(0, legHeightTargetPercent_));
  if (currentHeightPercent <= kHighLegLeanReduceStartPercent) {
    return kLegLeanStableMaxPercent;
  }

  const float span =
      (float)(kLegHeightStableMaxPercent - kHighLegLeanReduceStartPercent);
  const float progress =
      span <= 0.0f
          ? 1.0f
          : constrain((currentHeightPercent - kHighLegLeanReduceStartPercent) /
                          span,
                      0.0f, 1.0f);
  const float limited =
      kLegLeanStableMaxPercent -
      progress * (kLegLeanStableMaxPercent - kHighLegLeanMinPercent);
  return constrain((int)roundf(limited), kHighLegLeanMinPercent,
                   kLegLeanStableMaxPercent);
}

void MotionCoreAdapter::updateLegHeightTarget(uint32_t now) {
  if (!legHeightTargetActive_ || maintenance_) return;
  if (ctrl.fsm_state_machine.mode != fsm::mode_state::BALANCE) {
    legHeightTargetActive_ = false;
    legHeightForceSync_ = false;
    legHeightForceSyncHoldUntilMs_ = 0;
    ctrl.force_sync_leg_motion = 0;
    ctrl.symmetric_leg_motion = 0;
    return;
  }

  const uint32_t elapsedMs =
      lastLegHeightUpdateMs_ == 0 ? 2 : now - lastLegHeightUpdateMs_;
  lastLegHeightUpdateMs_ = now;
  const bool keepHighLegSync = legHeightTargetPercent_ >= kHighLegSyncStartPercent;
  const bool keepSync =
      keepHighLegSync || legHeightForceSync_ ||
      static_cast<int32_t>(now - legHeightForceSyncHoldUntilMs_) < 0;
  ctrl.force_sync_leg_motion = keepSync ? 1 : 0;
  ctrl.symmetric_leg_motion = keepSync ? 1 : ctrl.symmetric_leg_motion;
  const float dt = min(elapsedMs, (uint32_t)50) / 1000.0f;
  ctrl.leg_height_base = approach(ctrl.leg_height_base, legHeightBaseTarget_,
                                  kLegHeightSlewPerSecond * dt);
  if (fabsf(ctrl.leg_height_base - legHeightBaseTarget_) < 0.05f) {
    ctrl.leg_height_base = legHeightBaseTarget_;
    legHeightTargetActive_ = false;
    legHeightForceSync_ = false;
    if (!keepHighLegSync) {
      ctrl.symmetric_leg_motion = 0;
      ctrl.force_sync_leg_motion = 0;
    }
  }
}

void MotionCoreAdapter::armDefaultStandPose() {
  defaultStandPosePending_ = true;
}

void MotionCoreAdapter::setLegHeightTargetPercent(int percent) {
  percent = constrain(percent, 0, kLegHeightStableMaxPercent);
  heldPostureButtons_ = 0;
  const float requestedTarget = legHeightBaseFromPercent(percent);
  const bool samePercent = (legHeightTargetPercent_ == percent);
  const bool alreadyAtTarget =
      fabsf(ctrl.leg_height_base - requestedTarget) < 0.05f;
  legHeightTargetPercent_ = percent;
  legHeightBaseTarget_ = requestedTarget;
  legHeightForceSync_ =
      requestedTarget < ctrl.leg_height_base || percent >= kHighLegSyncStartPercent;
  legHeightForceSyncHoldUntilMs_ = millis() + kLegHeightForceSyncHoldMs;
  legHeightTargetActive_ = !alreadyAtTarget;
  ctrl.symmetric_leg_motion = 1;
  ctrl.force_sync_leg_motion =
      (legHeightForceSync_ || !alreadyAtTarget) ? 1 : 0;
  if (!samePercent || !alreadyAtTarget) {
    lastLegHeightUpdateMs_ = millis();
  }
}

void MotionCoreAdapter::setGuardServoAngle(int angleDeg) {
  guardServoAngleDeg_ = constrain(angleDeg, 0, 180);
  frontier_servo.set_angle((uint16_t)guardServoAngleDeg_);
}

void MotionCoreAdapter::pulseButton(uint16_t button, uint32_t durationMs) {
  pulseButtons_ = button;
  pulseUntilMs_ = millis() + durationMs;
}

void MotionCoreAdapter::setMoveAxes(int joyX, int joyY) {
  axes_[0] = constrain((float)joyX / 100.0f, -1.0f, 1.0f);
  axes_[3] = constrain((float)joyY / 100.0f, -1.0f, 1.0f);
}

void MotionCoreAdapter::stopMove() {
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  axes_[0] = 0.0f;
  axes_[3] = 0.0f;
}

void MotionCoreAdapter::startDanceDemo(uint32_t now) {
  maintenance_ = false;
  tracking_ = false;
  enterTrackingState(TrackObservationState::Idle);
  trackDecision_ = "idle";
  stopMove();
  heldPostureButtons_ = 0;
  pulseButtons_ = 0;
  legHeightTargetPercent_ = -1;
  legHeightTargetActive_ = false;
  legHeightForceSync_ = false;
  legHeightForceSyncHoldUntilMs_ = 0;
  ctrl.force_sync_leg_motion = 0;
  ctrl.symmetric_leg_motion = 0;
  ctrl.roll_adjust_target = 0.0f;
  ctrl.leg_lean = 0.0f;
  ctrl.leg_lean_target = 0.0f;
  setGuardServoAngle(kDefaultGuardAngleDeg);

  danceDemoQueued_ = true;
  danceDemoActive_ = false;
  danceStandTriggered_ = false;
  danceDemoStartedMs_ = 0;
  danceBalanceReadySinceMs_ = 0;
  danceCueIndex_ = -1;
}

void MotionCoreAdapter::stopDanceDemo(bool restoreNeutralPose) {
  danceDemoQueued_ = false;
  danceDemoActive_ = false;
  danceStandTriggered_ = false;
  danceDemoStartedMs_ = 0;
  danceBalanceReadySinceMs_ = 0;
  danceCueIndex_ = -1;
  stopMove();
  if (restoreNeutralPose) {
    ctrl.leg_lean_target = 0.0f;
    ctrl.roll_adjust_target = 0.0f;
    setLegHeightTargetPercent(kDefaultLegHeightPercent);
    setGuardServoAngle(kDefaultGuardAngleDeg);
  }
}

void MotionCoreAdapter::applyDanceCue(int cueIndex) {
  const int cueCount = sizeof(kDanceDemoCues) / sizeof(kDanceDemoCues[0]);
  if (cueIndex < 0 || cueIndex >= cueCount) return;
  const DanceCue& cue = kDanceDemoCues[cueIndex];
  ctrl.leg_lean_target =
      constrain((float)cue.leanPercent / 100.0f, -1.0f, 1.0f);
  ctrl.roll_adjust_target = 0.0f;
  setLegHeightTargetPercent(cue.heightPercent);
  setGuardServoAngle(cue.guardAngleDeg);
  setMoveAxes(cue.steerPercent, cue.drivePercent);
}

void MotionCoreAdapter::updateDanceDemo(uint32_t now) {
  if (!danceDemoQueued_ && !danceDemoActive_) return;

  if (maintenance_) {
    stopDanceDemo(false);
    return;
  }

  if (!danceDemoActive_) {
    danceDemoQueued_ = false;
    danceDemoActive_ = true;
    danceDemoStartedMs_ = now;
    danceCueIndex_ = -1;
  }

  const uint32_t elapsedMs = now - danceDemoStartedMs_;
  if (elapsedMs >= kDanceDemoDurationMs) {
    stopDanceDemo(true);
    return;
  }

  if (!danceStandTriggered_ && elapsedMs >= kDanceStandTriggerMs) {
    danceStandTriggered_ = true;
    armDefaultStandPose();
    pulseButton(BTN_RB, 120);
    danceBalanceReadySinceMs_ = 0;
  }

  const int cueCount = sizeof(kDanceDemoCues) / sizeof(kDanceDemoCues[0]);
  const int cueIndex =
      min((int)(elapsedMs / kDanceCueDurationMs), cueCount - 1);
  if (cueIndex != danceCueIndex_) {
    danceCueIndex_ = cueIndex;
    applyDanceCue(cueIndex);
  }

  if (!danceStandTriggered_) {
    stopMove();
    return;
  }

  if (ctrl.fsm_state_machine.mode != fsm::mode_state::BALANCE) {
    stopMove();
    danceBalanceReadySinceMs_ = 0;
    return;
  }

  if (danceBalanceReadySinceMs_ == 0) {
    danceBalanceReadySinceMs_ = now;
    stopMove();
    return;
  }

  if (now - danceBalanceReadySinceMs_ < kDanceBalanceReadyMs) {
    stopMove();
  }
}

void MotionCoreAdapter::holdTrackingChassis(bool resetReference) {
  (void)resetReference;
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  trackYawEngaged_ = false;
  trackYawReversePending_ = false;
  trackDistanceEngaged_ = false;
  trackYawCandidateDirection_ = 0;
  trackYawCandidateFrames_ = 0;
  trackDistanceCandidateDirection_ = 0;
  trackDistanceCandidateFrames_ = 0;
}

void MotionCoreAdapter::updateStandNudge(uint32_t now) {
  if (!standNudgePending_ && standNudgeUntilMs_ == 0) return;

  if (maintenance_) {
    standNudgePending_ = false;
    standNudgeBalanceSinceMs_ = 0;
    standNudgeUntilMs_ = 0;
    return;
  }

  if (ctrl.fsm_state_machine.mode != fsm::mode_state::BALANCE) {
    standNudgeBalanceSinceMs_ = 0;
    return;
  }

  if (standNudgePending_) {
    if (standNudgeBalanceSinceMs_ == 0) standNudgeBalanceSinceMs_ = now;
    if (now - standNudgeBalanceSinceMs_ < kStandNudgeStableMs) return;
    standNudgePending_ = false;
    standNudgeUntilMs_ = now + kStandNudgeDurationMs;
  }

  if (standNudgeUntilMs_ != 0 && !deadlineReached(now, standNudgeUntilMs_)) {
    if (fabsf(axes_[0]) < 0.02f && fabsf(axes_[3]) < 0.02f) {
      axes_[3] = kStandNudgeAxis;
    }
  } else {
    if (fabsf(axes_[3] - kStandNudgeAxis) < 0.02f) axes_[3] = 0.0f;
    standNudgeBalanceSinceMs_ = 0;
    standNudgeUntilMs_ = 0;
  }
}

void MotionCoreAdapter::enterTrackingState(TrackObservationState state) {
  if (trackState_ == state) return;
  trackState_ = state;
  trackStateSinceMs_ = millis();
  recordDiagnosticEvent("tracking", String("state=") + trackingStateName());
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  trackYawEngaged_ = false;
  trackDistanceEngaged_ = false;
  trackYawCandidateDirection_ = 0;
  trackYawCandidateFrames_ = 0;
  trackDistanceCandidateDirection_ = 0;
  trackDistanceCandidateFrames_ = 0;
}

void MotionCoreAdapter::applyTrackObservation(const MotionCommand& command) {
  lastTrackObservationMs_ = millis();
  trackConfidence_ = command.confidence;
  trackProfile_ = constrain(command.trackingProfile, 0, 3);
  trackBoxX_ = command.targetBoxX;
  trackBoxY_ = command.targetBoxY;
  trackBoxW_ = command.targetBoxW;
  trackBoxH_ = command.targetBoxH;
  trackFrameW_ = command.frameWidth;
  trackFrameH_ = command.frameHeight;
  trackTargetH_ = command.targetHeight;
  trackMissedFrames_ = command.missedFrames;
  trackStableFrames_ = command.stableFrames;
  trackRawScore_ = command.rawScore;
  trackVelocityX_ = command.velocityX;
  trackVelocityY_ = command.velocityY;
  TrackObservationState nextState = command.trackState;
  if (!trackHasLockedTarget_ &&
      (nextState == TrackObservationState::Coasting ||
       nextState == TrackObservationState::Reacquiring)) {
    nextState = TrackObservationState::Acquiring;
  }
  if (nextState == TrackObservationState::Locked) {
    trackHasLockedTarget_ = true;
    trackReturnScanSent_ = false;
  }
  else {
    trackHasLockedTarget_ = false;
    trackChassisHoldUntilMs_ = 0;
  }
  enterTrackingState(nextState);
  rawTrackDx_ = command.x;
  rawTrackDy_ = command.y;
  rawTrackDz_ = command.z;
  filteredTrackDx_ = rawTrackDx_;
  filteredTrackDy_ = rawTrackDy_;
  filteredTrackDz_ = rawTrackDz_;
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  trackYawEngaged_ = false;
  trackDistanceEngaged_ = false;
  trackDecision_ = tracking_ ? "camera_motion_disabled" : "idle";
}

void MotionCoreAdapter::applyTrackTarget(int dx, int dy, int dz) {
  rawTrackDx_ = dx;
  rawTrackDy_ = dy;
  rawTrackDz_ = dz;
  filteredTrackDx_ = rawTrackDx_;
  filteredTrackDy_ = rawTrackDy_;
  filteredTrackDz_ = rawTrackDz_;
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  trackYawEngaged_ = false;
  trackDistanceEngaged_ = false;
  trackDecision_ = "camera_motion_disabled";
}

void MotionCoreAdapter::updateTrackingMotion(uint32_t now) {
  const uint32_t elapsedMs =
      lastTrackMotionUpdateMs_ == 0 ? 2 : now - lastTrackMotionUpdateMs_;
  (void)elapsedMs;
  lastTrackMotionUpdateMs_ = now;
  trackYawTarget_ = 0.0f;
  trackDriveTarget_ = 0.0f;
  trackYawEngaged_ = false;
  trackDistanceEngaged_ = false;
  trackDecision_ = "camera_motion_disabled";
}

const char* MotionCoreAdapter::modeName() const {
  if (danceDemoQueued_ || danceDemoActive_) return "dance_demo";
  switch (ctrl.fsm_state_machine.mode) {
    case fsm::mode_state::FIRST_BOOT:
      return "boot";
    case fsm::mode_state::BALANCE:
      return tracking_ ? "tracking" : "balance";
    case fsm::mode_state::SIT:
      return "sit";
    case fsm::mode_state::JUMP:
      return "jump";
    case fsm::mode_state::ERROR:
      return "error";
  }
  return "unknown";
}

const char* MotionCoreAdapter::trackingStateName() const {
  switch (trackState_) {
    case TrackObservationState::Acquiring: return "acquiring";
    case TrackObservationState::Locked: return "locked";
    case TrackObservationState::Coasting: return "coasting";
    case TrackObservationState::Reacquiring: return "reacquiring";
    case TrackObservationState::Lost: return "lost";
    case TrackObservationState::Idle:
    default: return "idle";
  }
}



