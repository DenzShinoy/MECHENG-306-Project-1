#pragma once
#include <Arduino.h>

// =====================================================================
//  Timing utilities — non-blocking replacements for delay()
// ---------------------------------------------------------------------
//  delay() busy-waits: it halts loop() for the whole duration, so PID,
//  motor updates, serial and limit-switch checks all freeze (only ISRs
//  keep running). These helpers never wait. You poll them each pass
//  through loop() and they tell you when time is up, so every task keeps
//  running at its own rate on one shared millis() clock.
//
//  All arithmetic is done as (now - mark) on unsigned long, which stays
//  correct across the ~49-day millis() overflow. Never write
//  (now >= mark + interval) — that breaks at the wrap.
// =====================================================================

namespace timing {

// ---------------------------------------------------------------------
//  Interval — fires repeatedly, once per period.
//
//  Use for anything periodic (the control loop, a status print). Call
//  ready() every pass; it returns true at most once per `period` ms and
//  re-arms itself.
//
//    timing::Interval control(cfg::CONTROL_PERIOD_MS);
//    void loop() {
//      if (control.ready()) { /* run one control step */ }
//      // ... other work keeps running ...
//    }
// ---------------------------------------------------------------------
class Interval {
public:
  explicit Interval(unsigned long period_ms) : _period(period_ms), _mark(0) {}

  // True when at least _period ms have elapsed since the last fire. On a
  // true result the deadline advances, so calling it every loop gives a
  // steady cadence. Non-blocking — returns immediately either way.
  bool ready() {
    const unsigned long now = millis();
    if (now - _mark >= _period) {
      _mark = now;
      return true;
    }
    return false;
  }

  // Change the period on the fly (e.g. slow/fast status output).
  void setPeriod(unsigned long period_ms) { _period = period_ms; }

  // Restart the interval from now (skips the current pending fire).
  void reset() { _mark = millis(); }

private:
  unsigned long _period;
  unsigned long _mark;   // millis() at the last fire
};

// ---------------------------------------------------------------------
//  Timeout — a one-shot, non-blocking "delay".
//
//  This is the direct stand-in for `delay(ms)` when you mean "do X once,
//  ms milliseconds from now, without stopping the machine". Arm it with
//  start(ms), then poll expired() each loop until it returns true.
//
//    timing::Timeout settle;
//    settle.start(cfg::DEBOUNCE_MS);      // instead of delay(5)
//    ...
//    if (settle.expired()) { /* the wait is over */ }
// ---------------------------------------------------------------------
class Timeout {
public:
  Timeout() : _mark(0), _duration(0), _armed(false) {}

  // Begin a wait of `ms` milliseconds. Does not block.
  void start(unsigned long ms) {
    _mark = millis();
    _duration = ms;
    _armed = true;
  }

  // True once the wait has elapsed. Returns false while still waiting and
  // false again after it has fired (so it triggers your action just once)
  // until start() is called again. Non-blocking.
  bool expired() {
    if (!_armed) return false;
    if (millis() - _mark >= _duration) {
      _armed = false;
      return true;
    }
    return false;
  }

  // Still counting down?
  bool running() const { return _armed; }

  // Cancel a pending wait without firing.
  void cancel() { _armed = false; }

private:
  unsigned long _mark;      // millis() when start() was called
  unsigned long _duration;  // wait length in ms
  bool          _armed;     // false once fired or cancelled
};

} // namespace timing
