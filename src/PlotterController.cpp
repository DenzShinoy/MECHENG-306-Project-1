#include "PlotterController.h"

#include "GCodeParser.h"
#include "Pins.h"

PlotterController::PlotterController(Encoder& encL, Encoder& encR,
                                     MotorDriver& motL, MotorDriver& motR,
                                     PIDController& pidL, PIDController& pidR,
                                     LimitSwitch& swTop, LimitSwitch& swBottom,
                                     LimitSwitch& swLeft, LimitSwitch& swRight,
                                     TrajectoryPlanner& planner,
                                     StateMachine& fsm)
    : _encL(encL),
      _encR(encR),
      _motL(motL),
      _motR(motR),
      _pidL(pidL),
      _pidR(pidR),
      _swTop(swTop),
      _swBottom(swBottom),
      _swLeft(swLeft),
      _swRight(swRight),
      _planner(planner),
      _fsm(fsm) {}

void PlotterController::begin() {
  _encL.begin();
  _encR.begin();
  _motL.begin();
  _motR.begin();
  _swTop.begin();
  _swBottom.begin();
  _swLeft.begin();
  _swRight.begin();
}

void PlotterController::update(uint32_t nowMs) { (void)nowMs; }

void PlotterController::pumpSerial() {
  // TODO: accumulate chars into _lineBuf until newline; on a full line
  // call _parser.parseLine() and forward to handleCommand().
}

void PlotterController::handleCommand(const GCodeCommand& cmd) {
  // TODO: G28 -> _fsm.dispatch(HOME_CMD); G1 -> convert mm to counts
  // (Kinematics::mmToCounts), _planner.plan(...), _fsm.dispatch(MOVE_CMD).
  (void)cmd;
}

void PlotterController::serviceHoming(uint32_t nowMs) {
  // TODO: drive axes toward the switches at cfg::HOMING_PWM; on
  // justPressed() stop that axis and reset the relevant encoder; when
  // done _fsm.dispatch(HOMED).
  (void)nowMs;
}

void PlotterController::runControlTick(uint32_t nowMs) {
  // TODO: _planner.update(nowMs); map Cartesian setpoint -> A/B counts
  // (Kinematics::xyToAB); PID each axis against encoder counts; write
  // motors; on _planner.isDone() dispatch TARGET_REACHED.
  (void)nowMs;
}

void PlotterController::updateSwitches(uint32_t nowMs) {
  // TODO: update() all four switches; an unexpected press outside HOMING
  // -> enterFault().
  (void)nowMs;
}

void PlotterController::enterFault() {
  // TODO: stop both motors and _fsm.dispatch(LIMIT_HIT).
}
