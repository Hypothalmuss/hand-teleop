// Latching e-stop state machine: RUNNING -> STOPPED -> (clear) -> RUNNING (no ROS).
#pragma once

namespace diff_ik {

class EstopLatch {
 public:
  enum class State { RUNNING, STOPPED };

  // Returns true on the RUNNING -> STOPPED edge (caller freezes q_cmd = q_measured).
  bool trigger() {
    if (state_ == State::STOPPED) return false;
    state_ = State::STOPPED;
    return true;
  }
  // Returns true on the STOPPED -> RUNNING edge (caller resyncs q_cmd = q_measured).
  bool clear() {
    if (state_ == State::RUNNING) return false;
    state_ = State::RUNNING;
    return true;
  }
  bool stopped() const { return state_ == State::STOPPED; }
  State state() const { return state_; }

 private:
  State state_ = State::RUNNING;
};

}  // namespace diff_ik
