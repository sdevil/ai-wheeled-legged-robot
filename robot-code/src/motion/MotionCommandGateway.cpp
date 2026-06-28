#include "MotionCommandGateway.h"

#include "Diagnostics.h"
#include "MotionCoreAdapter.h"

MotionCommandGateway& motionGateway() {
  static MotionCommandGateway instance;
  return instance;
}

void MotionCommandGateway::dispatch(const MotionCommand& command,
                                    const char* trigger, bool event) {
  setMotionTrigger(trigger);
  if (event) recordDiagnosticEvent("motion", trigger);
  motionCore().command(command);
}

void MotionCommandGateway::dispatch(const MotionCommand& command,
                                    const String& trigger, bool event) {
  dispatch(command, trigger.c_str(), event);
}

void MotionCommandGateway::dispatch(const ExternalMotionRequest& request,
                                    const char* trigger, bool event) {
  MotionCommand command = request.command;
  switch (request.action) {
    case ExternalMotionAction::Move:
      command = MotionCommand::move(request.turn, request.forward);
      break;
    case ExternalMotionAction::Stop:
      command = MotionCommand::simple(MotionCommandType::Stop);
      break;
    case ExternalMotionAction::Stand:
      command = MotionCommand::simple(MotionCommandType::Stand);
      break;
    case ExternalMotionAction::Sit:
      command = MotionCommand::simple(MotionCommandType::Sit);
      break;
    case ExternalMotionAction::LegHeight:
      command = MotionCommand::legHeightPercent(request.amplitude);
      break;
    case ExternalMotionAction::LegLean:
      command = MotionCommand::legLean(request.amplitude);
      break;
    case ExternalMotionAction::RawMotionCommand:
    default:
      break;
  }
  dispatch(command, trigger, event);
}

void MotionCommandGateway::dispatch(const ExternalMotionRequest& request,
                                    const String& trigger, bool event) {
  dispatch(request, trigger.c_str(), event);
}
