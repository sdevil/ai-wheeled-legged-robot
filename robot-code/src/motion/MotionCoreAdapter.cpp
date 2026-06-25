#include "MotionCoreAdapter.h"

#include <math.h>
#include <string.h>

#include "controller/controller.h"
#include "Diagnostics.h"
#include "motion_buttons.h"
#include "devices/ptk7350.h"
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
constexpr float kLegLeanMaxDeg = 16.0f;
constexpr int kCameraMinDeg = 45;
constexpr int kCameraMaxDeg = 150;
constexpr int kCameraStandbyDeg = 105;
constexpr int kTrackCameraMinDeg = 75;
constexpr int kTrackCameraMaxDeg = 130;
constexpr float kCameraSlewDegPerSecond = 70.0f;
constexpr uint32_t kCameraCalibrationHighMs = 1200;
constexpr uint32_t kCameraCalibrationLowMs = 3000;
constexpr uint32_t kCameraCalibrationFinishMs = 4500;
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
  cameraAngle_ = kCameraStandbyDeg;
  cameraTargetAngle_ = kCameraMaxDeg;
  cam_servo.set_angle((uint16_t)roundf(cameraAngle_));
  cameraCalibrationActive_ = true;
  cameraCalibrationStartedMs_ = millis();
  frontier_servo.set_angle(0);

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
  updateLegHeightTarget(now);
  updateCameraCalibration(now);
  if (tracking_ && !cameraCalibrationActive_) updateTrackingMotion(now);
  updateStandNudge(now);
  updateCameraGimbal(now);

  ctrl.host_data.buttons = pulseButtons_ | heldPostureButtons_;
  memcpy(ctrl.host_data.axes, axes_, sizeof(axes_));
}

void MotionCoreAdapter::command(const MotionCommand& command) {
  switch (command.type) {
    case MotionCommandType::Stand:
      maintenance_ = false;
      legHeightTargetActive_ = false;
      ctrl.symmetric_leg_motion = 0;
      heldPostureButtons_ = 0;
      ctrl.roll_adjust_target = 0.0f;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      pulseButton(BTN_RB, 120);
      break;
    case MotionCommandType::Sit:
      tracking_ = false;
      legHeightTargetActive_ = false;
      ctrl.symmetric_leg_motion = 0;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      heldPostureButtons_ = 0;
      enterTrackingState(TrackObservationState::Idle);
      ctrl.roll_adjust_target = 0.0f;
      stopMove();
      pulseButton(BTN_LB, 120);
      break;
    case MotionCommandType::ResetPose:
      cameraTargetAngle_ = kCameraStandbyDeg;
      frontier_servo.set_angle(0);
      break;
    case MotionCommandType::Move:
      if (!maintenance_ && !tracking_) {
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
      legHeightTargetActive_ = false;
      ctrl.symmetric_leg_motion = 0;
      standNudgePending_ = false;
      standNudgeBalanceSinceMs_ = 0;
      standNudgeUntilMs_ = 0;
      trackHasLockedTarget_ = false;
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
      enterTrackingState(TrackObservationState::Acquiring);
      stopMove();
      break;
    case MotionCommandType::TrackStop:
      tracking_ = false;
      ctrl.symmetric_leg_motion = 0;
      trackHasLockedTarget_ = false;
      lastTrackObservationMs_ = 0;
      trackBalanceReadySinceMs_ = 0;
      lastTrackControlUpdateMs_ = 0;
      trackProfile_ = 0;
      enterTrackingState(TrackObservationState::Idle);
      stopMove();
      cameraAngle_ = kCameraStandbyDeg;
      cameraTargetAngle_ = kCameraStandbyDeg;
      break;
    case MotionCommandType::TrackTarget:
      if (tracking_ && !maintenance_) {
        trackHasLockedTarget_ = true;
        enterTrackingState(TrackObservationState::Locked);
        lastTrackObservationMs_ = millis();
        applyTrackTarget(constrain(command.x * 1000 / 320, -1000, 1000),
                         constrain(command.y * 1000 / 240, -1000, 1000),
                         command.z == 9999 ? 9999 : constrain(command.z * 6, -1000, 1000));
      }
      break;
    case MotionCommandType::TrackObservation:
      if (tracking_ && !maintenance_) applyTrackObservation(command);
      break;
    case MotionCommandType::CameraGimbal:
      if (!cameraCalibrationActive_) {
        cameraTargetAngle_ = constrain(cameraTargetAngle_ + command.y,
                                       (float)kCameraMinDeg,
                                       (float)kCameraMaxDeg);
      }
      break;
    case MotionCommandType::LegLean:
      if (!maintenance_ && !tracking_) {
        ctrl.roll_adjust_target =
            constrain((float)command.x * (kLegLeanMaxDeg / 100.0f),
                      -kLegLeanMaxDeg, kLegLeanMaxDeg);
      }
      break;
    case MotionCommandType::LegHeight:
      if (command.y == 0) {
        heldPostureButtons_ = 0;
        ctrl.symmetric_leg_motion = 0;
      } else if (!maintenance_ && !tracking_) {
        legHeightTargetActive_ = false;
        ctrl.symmetric_leg_motion = 1;
        heldPostureButtons_ = command.y > 0 ? BTN_UP : BTN_DOWN;
      }
      break;
    case MotionCommandType::LegHeightPercent:
      if (!maintenance_ && !tracking_) {
        heldPostureButtons_ = 0;
        legHeightBaseTarget_ = legHeightBaseFromPercent(command.y);
        legHeightTargetActive_ = true;
        ctrl.symmetric_leg_motion = 1;
        lastLegHeightUpdateMs_ = millis();
      }
      break;
    case MotionCommandType::MaintenanceEnter:
      maintenance_ = true;
      tracking_ = false;
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
  data.cameraAngleDeg = cameraAngle_;
  data.cameraTargetDeg = cameraTargetAngle_;
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
  if (!started_ || cameraCalibrationActive_ ||
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

void MotionCoreAdapter::updateCameraCalibration(uint32_t now) {
  if (!cameraCalibrationActive_) return;
  const uint32_t elapsed = now - cameraCalibrationStartedMs_;
  if (elapsed < kCameraCalibrationHighMs) {
    cameraTargetAngle_ = kCameraMaxDeg;
  } else if (elapsed < kCameraCalibrationLowMs) {
    cameraTargetAngle_ = kCameraMinDeg;
  } else {
    cameraTargetAngle_ = kCameraStandbyDeg;
  }
  if (elapsed >= kCameraCalibrationFinishMs &&
      fabsf(cameraAngle_ - (float)kCameraStandbyDeg) < 1.0f) {
    cameraAngle_ = kCameraStandbyDeg;
    cameraTargetAngle_ = kCameraStandbyDeg;
    cam_servo.set_angle(kCameraStandbyDeg);
    cameraCalibrationActive_ = false;
  }
}

float MotionCoreAdapter::legHeightBaseFromPercent(int percent) const {
  const float normalized = constrain((float)percent / 100.0f, 0.0f, 1.0f);
  return kLegHeightControlMax -
         normalized * (kLegHeightControlMax - kLegHeightControlMin);
}

int MotionCoreAdapter::legHeightPercent() const {
  const float normalized =
      (kLegHeightBaseMax - ctrl.leg_height_base) /
      (kLegHeightBaseMax - kLegHeightBaseMin);
  return constrain((int)roundf(normalized * 100.0f), 0, 100);
}

int MotionCoreAdapter::legLeanPercent() const {
  return constrain((int)roundf(ctrl.roll_adjust_target * 10.0f), -100, 100);
}

void MotionCoreAdapter::updateLegHeightTarget(uint32_t now) {
  if (!legHeightTargetActive_ || maintenance_ || tracking_) return;
  if (ctrl.fsm_state_machine.mode != fsm::mode_state::BALANCE) {
    legHeightTargetActive_ = false;
    ctrl.symmetric_leg_motion = 0;
    return;
  }

  const uint32_t elapsedMs =
      lastLegHeightUpdateMs_ == 0 ? 2 : now - lastLegHeightUpdateMs_;
  lastLegHeightUpdateMs_ = now;
  const float dt = min(elapsedMs, (uint32_t)50) / 1000.0f;
  ctrl.leg_height_base = approach(ctrl.leg_height_base, legHeightBaseTarget_,
                                  kLegHeightSlewPerSecond * dt);
  if (fabsf(ctrl.leg_height_base - legHeightBaseTarget_) < 0.05f) {
    ctrl.leg_height_base = legHeightBaseTarget_;
    legHeightTargetActive_ = false;
    ctrl.symmetric_leg_motion = 0;
  }
}

void MotionCoreAdapter::updateCameraGimbal(uint32_t now) {
  const uint32_t elapsedMs =
      lastCameraGimbalUpdateMs_ == 0 ? 2 : now - lastCameraGimbalUpdateMs_;
  lastCameraGimbalUpdateMs_ = now;
  const float dt = min(elapsedMs, (uint32_t)50) / 1000.0f;
  const float previous = cameraAngle_;
  cameraAngle_ = approach(cameraAngle_, cameraTargetAngle_,
                          kCameraSlewDegPerSecond * dt);
  if ((int)roundf(previous) != (int)roundf(cameraAngle_)) {
    cam_servo.set_angle((uint16_t)roundf(cameraAngle_));
  }
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

void MotionCoreAdapter::clearTrackDriveOutput() {
  trackDriveTarget_ = 0.0f;
  trackDistanceEngaged_ = false;
  trackDistanceCandidateDirection_ = 0;
  trackDistanceCandidateFrames_ = 0;
  filteredTrackDz_ = 0.0f;
  axes_[3] = 0.0f;
}

void MotionCoreAdapter::updateStandNudge(uint32_t now) {
  if (!standNudgePending_ && standNudgeUntilMs_ == 0) return;

  if (maintenance_ || tracking_ || cameraCalibrationActive_) {
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
  if (state == TrackObservationState::Reacquiring) {
    stopMove();
    searchCameraCenterAngle_ = cameraTargetAngle_;
    ctrl.lqi_param.ref.yaw_rate = 0.0f;
    ctrl.lqi_param.integral.yaw_rate_error = 0.0f;
    trackYawTarget_ = 0.0f;
    trackDriveTarget_ = 0.0f;
    trackYawEngaged_ = false;
    trackYawReversePending_ = false;
    trackDistanceEngaged_ = false;
    trackYawCandidateDirection_ = 0;
    trackYawCandidateFrames_ = 0;
    trackDistanceCandidateDirection_ = 0;
    trackDistanceCandidateFrames_ = 0;
  } else if (state == TrackObservationState::Acquiring ||
             state == TrackObservationState::Coasting ||
             state == TrackObservationState::Lost ||
             state == TrackObservationState::Idle) {
    stopMove();
    trackYawEngaged_ = false;
    trackYawReversePending_ = false;
    trackDistanceEngaged_ = false;
    trackYawCandidateDirection_ = 0;
    trackYawCandidateFrames_ = 0;
    trackDistanceCandidateDirection_ = 0;
    trackDistanceCandidateFrames_ = 0;
  }
  if ((state == TrackObservationState::Lost ||
       state == TrackObservationState::Idle) && trackHasLockedTarget_) {
    cameraTargetAngle_ = kCameraStandbyDeg;
  }
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
  if (nextState == TrackObservationState::Locked) trackHasLockedTarget_ = true;
  enterTrackingState(nextState);

  if (nextState == TrackObservationState::Locked) {
    applyTrackTarget(command.x, command.y, command.z);
  } else if (nextState == TrackObservationState::Coasting ||
             nextState == TrackObservationState::Reacquiring) {
    // Visual gaps are not reliable target observations. Stop the chassis and
    // hold the gimbal. Brief detector misses are common; snapping the camera
    // back to standby creates a vision-balance feedback loop.
    trackYawTarget_ = 0.0f;
    trackDriveTarget_ = 0.0f;
    axes_[0] = 0.0f;
    axes_[3] = 0.0f;
    ctrl.lqi_param.ref.yaw_rate = 0.0f;
    ctrl.lqi_param.ref.linear_vel = 0.0f;
    ctrl.lqi_param.integral.yaw_rate_error = 0.0f;
    ctrl.lqi_param.integral.linear_vel_error = 0.0f;
  } else if (nextState == TrackObservationState::Acquiring ||
             nextState == TrackObservationState::Lost ||
             nextState == TrackObservationState::Idle) {
    stopMove();
  }
}

void MotionCoreAdapter::applyTrackTarget(int dx, int dy, int dz) {
  rawTrackDx_ = dx;
  rawTrackDy_ = dy;
  rawTrackDz_ = dz;
  trackDecision_ = "input_received";
  if (dx == 9999 || dy == 9999 ||
      !deadlineReached(millis(), trackSettleUntilMs_)) {
    trackYawTarget_ = 0.0f;
    clearTrackDriveOutput();
    trackDecision_ = dx == 9999 || dy == 9999 ? "invalid_error" : "settling";
    return;
  }

  const TrackingTuning tuning = trackingTuning(trackProfile_);
  trackYawTarget_ = 0.0f;
  trackYawEngaged_ = false;
  clearTrackDriveOutput();
  trackDecision_ = "vision_lock_only";
  if (trackProfile_ == 1 && trackStableFrames_ < 6) {
    trackDecision_ = "waiting_stable_face";
    return;
  }
  if (trackConfidence_ > 0 && trackConfidence_ < tuning.minimumConfidence) {
    trackYawTarget_ = 0.0f;
    trackYawEngaged_ = false;
    clearTrackDriveOutput();
    trackDecision_ = "low_confidence";
    return;
  }

  int pitchControlDy = dy;
  if (trackFrameH_ > 0 && trackBoxH_ > 0) {
    const float targetCenterY = (float)trackBoxY_ + (float)trackBoxH_ * 0.5f;
    const float safeTop = (float)trackFrameH_ / 3.0f;
    const float safeBottom = (float)trackFrameH_ * 2.0f / 3.0f;
    float pitchErrorPx = 0.0f;
    if (targetCenterY < safeTop) {
      pitchErrorPx = targetCenterY - safeTop;
    } else if (targetCenterY > safeBottom) {
      pitchErrorPx = targetCenterY - safeBottom;
    }
    pitchControlDy = constrain((int)roundf(pitchErrorPx * 1000.0f /
                                           max(1.0f, (float)trackFrameH_ * 0.5f)),
                               -1000, 1000);
  }

  if (trackProfile_ == 1) {
    const bool unreliablePitch = abs(dy) >= 850;
    const bool unreliableDistance = dz != 9999 && abs(dz) >= 650;
    dx = constrain(dx, -260, 260);
    pitchControlDy = unreliablePitch ? 0 : constrain(pitchControlDy, -260, 260);
    if (unreliableDistance) dz = 9999;
    else if (dz != 9999) dz = constrain(dz, -300, 300);
  }

  int yawControlDx = dx;
  if (trackFrameW_ > 0 && trackBoxW_ > 0) {
    const float targetCenterX = (float)trackBoxX_ + (float)trackBoxW_ * 0.5f;
    const float safeLeft = (float)trackFrameW_ / 3.0f;
    const float safeRight = (float)trackFrameW_ * 2.0f / 3.0f;
    float yawErrorPx = 0.0f;
    if (targetCenterX < safeLeft) {
      yawErrorPx = targetCenterX - safeLeft;
    } else if (targetCenterX > safeRight) {
      yawErrorPx = targetCenterX - safeRight;
    }
    yawControlDx = constrain((int)roundf(yawErrorPx * 1000.0f /
                                         max(1.0f, (float)trackFrameW_ * 0.5f)),
                             -1000, 1000);
  }

  filteredTrackDx_ += ((float)yawControlDx - filteredTrackDx_) * tuning.filterAlpha;
  if (pitchControlDy == 0) {
    filteredTrackDy_ = 0.0f;
  } else {
    filteredTrackDy_ = (float)pitchControlDy;
  }
  dz = 9999;

  const float horizontalError = fabsf(filteredTrackDx_);
  const float pitchAngleDeg = ctrl.lqi_param.state.pitch_angle * 180.0f / PI;
  const float pitchRateDeg = ctrl.lqi_param.state.pitch_rate * 180.0f / PI;
  const bool balanceQuiet = trackProfile_ != 1 ||
      (fabsf(pitchAngleDeg) < 7.5f && fabsf(pitchRateDeg) < 5.0f);
  if (!balanceQuiet) {
    trackYawTarget_ = 0.0f;
    trackYawEngaged_ = false;
    clearTrackDriveOutput();
    trackDecision_ = "balance_not_quiet";
    return;
  }

  const uint32_t now = millis();
  const float visionOnlyPitchDeadband = trackProfile_ == 3 ? 12.0f : 18.0f;
  trackYawTarget_ = 0.0f;
  trackYawEngaged_ = false;
  filteredTrackDx_ = 0.0f;
  axes_[0] = 0.0f;
  ctrl.jump_turn_yaw_rate_cmd = 0.0f;
  ctrl.lqi_param.ref.yaw_rate = 0.0f;
  ctrl.lqi_param.integral.yaw_rate_error = 0.0f;

  if (fabsf(filteredTrackDy_) > visionOnlyPitchDeadband &&
      now - lastTrackGimbalUpdateMs_ >= 24U) {
    const float pitchGain = trackProfile_ == 3 ? 0.018f : 0.008f;
    const int maxPitchStep = 2;
    int delta = constrain((int)roundf(-filteredTrackDy_ * pitchGain), -maxPitchStep, maxPitchStep);
    if (delta == 0) delta = filteredTrackDy_ > 0.0f ? -1 : 1;
    cameraTargetAngle_ = constrain(cameraTargetAngle_ + delta,
                                   (float)kTrackCameraMinDeg,
                                   (float)kTrackCameraMaxDeg);
    lastTrackGimbalUpdateMs_ = now;
  }
  if (fabsf(filteredTrackDy_) <= visionOnlyPitchDeadband) {
    trackDecision_ = "inside_pitch_safe_zone";
    return;
  }
  trackDecision_ = "vision_pitch_only";
  lastTrackControlUpdateMs_ = now;
  clearTrackDriveOutput();
  return;

  const uint32_t controlIntervalMs =
      trackProfile_ == 3 ? 60U : (trackProfile_ == 1 ? 45U : kTrackControlIntervalMs);
  if (lastTrackControlUpdateMs_ != 0 &&
      now - lastTrackControlUpdateMs_ < controlIntervalMs) {
    trackDecision_ = "rate_limited";
    return;
  }
  lastTrackControlUpdateMs_ = now;

  const float yawEngageError = trackProfile_ == 3 ? 70.0f : (trackProfile_ == 1 ? 120.0f : kTrackYawEngageError);
  const float yawReleaseError = trackProfile_ == 3 ? 35.0f : (trackProfile_ == 1 ? 28.0f : kTrackYawReleaseError);
  const float yawEngageThreshold = trackProfile_ == 1 ? 58.0f : yawEngageError;
  const int requestedDirection = filteredTrackDx_ > 0.0f ? 1 : -1;
  if (!trackYawEngaged_) {
    if (horizontalError >= yawEngageThreshold) {
      if (trackYawCandidateDirection_ == requestedDirection) {
        trackYawCandidateFrames_++;
      } else {
        trackYawCandidateDirection_ = requestedDirection;
        trackYawCandidateFrames_ = 1;
      }
      const uint8_t requiredFrames = trackProfile_ == 1
                                         ? 1
                                         : (trackYawReversePending_
                                                ? kTrackYawReverseFrames
                                                : kTrackYawEngageFrames);
      if (trackYawCandidateFrames_ >= requiredFrames) {
        trackYawEngaged_ = true;
        trackYawReversePending_ = false;
        lastTargetDirection_ = requestedDirection;
      }
    } else {
      trackYawCandidateDirection_ = 0;
      trackYawCandidateFrames_ = 0;
      trackYawReversePending_ = false;
    }
  } else if (horizontalError <= yawReleaseError) {
    trackYawEngaged_ = false;
    trackYawCandidateDirection_ = 0;
    trackYawCandidateFrames_ = 0;
    trackYawReversePending_ = false;
  } else if (requestedDirection != lastTargetDirection_) {
    trackYawEngaged_ = false;
    trackYawReversePending_ = true;
    trackYawCandidateDirection_ = requestedDirection;
    trackYawCandidateFrames_ = 1;
  }

  if (trackYawEngaged_) {
    trackYawTarget_ = constrain(
                                kTrackYawPolarity * filteredTrackDx_ / tuning.yawScale,
                                -tuning.maxYaw, tuning.maxYaw);
  } else {
    trackYawTarget_ = 0.0f;
  }

  const float driveAlignError = trackProfile_ == 1 ? 150.0f : kTrackDriveAlignError;
  const float distanceEngageError = trackProfile_ == 1 ? 70.0f : kTrackDistanceEngageError;
  const float distanceReleaseError = trackProfile_ == 1 ? 35.0f : kTrackDistanceReleaseError;
  const bool horizontallyAligned = horizontalError <= driveAlignError;
  const float distanceError = fabsf(filteredTrackDz_);
  const int distanceDirection = filteredTrackDz_ >= 0.0f ? 1 : -1;
  if (!horizontallyAligned || dz == 9999 ||
      distanceError <= distanceReleaseError) {
    trackDistanceEngaged_ = false;
    trackDistanceCandidateDirection_ = 0;
    trackDistanceCandidateFrames_ = 0;
  } else if (!trackDistanceEngaged_ &&
             distanceError >= distanceEngageError) {
    if (trackDistanceCandidateDirection_ == distanceDirection) {
      trackDistanceCandidateFrames_++;
    } else {
      trackDistanceCandidateDirection_ = distanceDirection;
      trackDistanceCandidateFrames_ = 1;
    }
    const uint8_t requiredDistanceFrames = trackProfile_ == 1 ? 4 : kTrackDistanceEngageFrames;
    if (trackDistanceCandidateFrames_ >= requiredDistanceFrames) {
      trackDistanceEngaged_ = true;
    }
  } else if (!trackDistanceEngaged_) {
    trackDistanceCandidateDirection_ = 0;
    trackDistanceCandidateFrames_ = 0;
  } else if (trackDistanceEngaged_ &&
             distanceDirection != trackDistanceCandidateDirection_) {
    trackDistanceEngaged_ = false;
    trackDistanceCandidateDirection_ = distanceDirection;
    trackDistanceCandidateFrames_ = 1;
  }

  if (trackDistanceEngaged_ && horizontallyAligned) {
    // Continue distance correction during a moderate turn, but progressively
    // reduce it as horizontal error grows so the robot does not drive sideways
    // out of the target lock.
    const float minAlignmentScale = trackProfile_ == 1 ? 0.15f : 0.25f;
    const float alignmentScale = constrain(
        1.0f - horizontalError / kTrackDriveAlignError, minAlignmentScale, 1.0f);
    trackDriveTarget_ = constrain(
                                  kTrackDrivePolarity * filteredTrackDz_ /
                                      tuning.distanceScale,
                                  -tuning.maxDrive, tuning.maxDrive) *
                        alignmentScale;
  } else {
    trackDriveTarget_ = 0.0f;
  }

  trackDecision_ = trackYawEngaged_ || trackDistanceEngaged_ ? "locked_command" : "inside_deadband";

  const float pitchDeadband = trackProfile_ == 3 ? 28.0f : (trackProfile_ == 1 ? 32.0f : kTrackPitchDeadband);
  if (fabsf(filteredTrackDy_) > pitchDeadband &&
      now - lastTrackGimbalUpdateMs_ >= (trackProfile_ == 1 ? 45U : kGimbalUpdateIntervalMs)) {
    const float pitchGain = trackProfile_ == 3 ? 0.034f : (trackProfile_ == 1 ? 0.016f : 0.026f);
    const int maxPitchStep = trackProfile_ == 1 ? 4 : 12;
    const int delta = constrain((int)roundf(-filteredTrackDy_ * pitchGain), -maxPitchStep, maxPitchStep);
    cameraTargetAngle_ = constrain(cameraTargetAngle_ + delta,
                                   (float)kCameraMinDeg,
                                   (float)kCameraMaxDeg);
    lastTrackGimbalUpdateMs_ = now;
  }
}
void MotionCoreAdapter::updateTrackingMotion(uint32_t now) {
  if (ctrl.fsm_state_machine.mode != fsm::mode_state::BALANCE) {
    trackBalanceReadySinceMs_ = 0;
    trackYawTarget_ = 0.0f;
    axes_[0] = 0.0f;
    clearTrackDriveOutput();
    lastTrackMotionUpdateMs_ = now;
    return;
  }

  if (trackBalanceReadySinceMs_ == 0) {
    trackBalanceReadySinceMs_ = now;
  }
  if (now - trackBalanceReadySinceMs_ < kTrackBalanceStableMs) {
    trackYawTarget_ = 0.0f;
    axes_[0] = 0.0f;
    clearTrackDriveOutput();
    lastTrackMotionUpdateMs_ = now;
    return;
  }

  if (lastTrackObservationMs_ != 0 &&
      now - lastTrackObservationMs_ > kTrackCommandTimeoutMs &&
      trackHasLockedTarget_ && trackState_ != TrackObservationState::Lost) {
    // Missing UART traffic is a transport/inference stall, not proof that the
    // target disappeared. Stop safely and wait for the camera's explicit state.
    enterTrackingState(TrackObservationState::Coasting);
  }

  if (trackState_ == TrackObservationState::Reacquiring) {
    const uint32_t elapsed = now - trackStateSinceMs_;
    trackYawTarget_ = 0.0f;
    clearTrackDriveOutput();
    cameraTargetAngle_ = kCameraStandbyDeg;
    if (elapsed >= kSearchGiveUpMs) {
      enterTrackingState(TrackObservationState::Lost);
    }
  }

  const uint32_t elapsedMs = lastTrackMotionUpdateMs_ == 0 ? 2 : now - lastTrackMotionUpdateMs_;
  lastTrackMotionUpdateMs_ = now;
  const float dt = min(elapsedMs, (uint32_t)50) / 1000.0f;
  trackYawTarget_ = 0.0f;
  trackYawEngaged_ = false;
  axes_[0] = 0.0f;
  ctrl.jump_turn_yaw_rate_cmd = 0.0f;
  ctrl.lqi_param.ref.yaw_rate = 0.0f;
  ctrl.lqi_param.integral.yaw_rate_error = 0.0f;
  clearTrackDriveOutput();
}

const char* MotionCoreAdapter::modeName() const {
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



