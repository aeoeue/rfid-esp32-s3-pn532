#include "door_controller.h"

#include <vector>

void DoorController::configure(const DoorControllerConfig& config) {
  if (cfg_.enabled && cfg_.pin >= 0 && cfg_.pin != config.pin) {
    pinMode(cfg_.pin, INPUT);
  }
  cfg_ = config;
  if (cfg_.pulseMs > cfg_.maxPulseMs) {
    cfg_.pulseMs = cfg_.maxPulseMs;
  }
  active_ = false;
  logicalState_ = false;
  activeUntilMs_ = 0;
  lockoutUntilMs_ = 0;
  activations_.clear();
  if (!cfg_.enabled) {
    if (cfg_.pin >= 0) {
      pinMode(cfg_.pin, INPUT);
    }
    return;
  }
  applyIdle();
}

void DoorController::loop() {
  pruneActivationTimes(millis());
  if (active_ && millis() > activeUntilMs_) {
    active_ = false;
    logicalState_ = false;
    applyIdle();
  }
}

bool DoorController::requestPulse(String* reason) {
  if (!cfg_.enabled) {
    if (reason) *reason = "disabled";
    return false;
  }
  if (!checkLockout(reason)) return false;

  uint32_t now = millis();
  pruneActivationTimes(now);
  activations_.push_back(now);
  if (cfg_.lockoutLimit > 0 && activations_.size() > cfg_.lockoutLimit) {
    lockoutUntilMs_ = now + cfg_.lockoutMs;
    if (reason) *reason = "lockout";
    applyIdle();
    return false;
  }

  bool idleKnown = false;
  bool idleLevel = false;
  idleKnown = computeIdleOutputLevel(idleLevel);

  bool pulseLevel = idleKnown ? !idleLevel : (cfg_.activeHigh ? HIGH : LOW);
  driveOutputLevel(pulseLevel);
  logicalState_ = true;
  active_ = true;
  activeUntilMs_ = now + cfg_.pulseMs;
  if (reason) *reason = "ok";
  return true;
}

bool DoorController::setManual(bool on, String* reason) {
  if (!cfg_.enabled) {
    if (reason) *reason = "disabled";
    return false;
  }
  if (!on) {
    active_ = false;
    logicalState_ = false;
    applyIdle();
    if (reason) *reason = "ok";
    return true;
  }
  if (!checkLockout(reason)) return false;
  uint32_t now = millis();
  pruneActivationTimes(now);
  activations_.push_back(now);
  if (cfg_.lockoutLimit > 0 && activations_.size() > cfg_.lockoutLimit) {
    lockoutUntilMs_ = now + cfg_.lockoutMs;
    if (reason) *reason = "lockout";
    applyIdle();
    return false;
  }

  driveOutputLevel(cfg_.activeHigh ? HIGH : LOW);
  active_ = true;
  logicalState_ = true;
  activeUntilMs_ = now + cfg_.pulseMs;
  if (reason) *reason = "ok";
  return true;
}

bool DoorController::enabled() const { return cfg_.enabled; }
bool DoorController::isActive() const { return active_; }
bool DoorController::isLockedOut() const { return lockoutUntilMs_ > millis(); }
bool DoorController::currentLogicalState() const { return logicalState_; }
uint32_t DoorController::pulseRemainingMs() const { return active_ ? (activeUntilMs_ - millis()) : 0; }
uint32_t DoorController::lockoutRemainingMs() const { return isLockedOut() ? (lockoutUntilMs_ - millis()) : 0; }

uint8_t DoorController::activationCountInWindow() const {
  return static_cast<uint8_t>(activations_.size());
}

String DoorController::statusText() const {
  if (!cfg_.enabled) return "disabled";
  if (isLockedOut()) return "locked_out";
  if (active_) return "active";
  return "idle";
}

void DoorController::applyIdle() {
  if (!cfg_.enabled || cfg_.pin < 0) return;
  if (cfg_.idleMode == "pullup") {
    pinMode(cfg_.pin, INPUT_PULLUP);
  } else if (cfg_.idleMode == "pulldown") {
    pinMode(cfg_.pin, INPUT_PULLDOWN);
  } else if (cfg_.idleMode == "floating") {
    pinMode(cfg_.pin, INPUT);
  } else {
    pinMode(cfg_.pin, OUTPUT);
    bool level = (cfg_.idleMode == "high");
    digitalWrite(cfg_.pin, level ? HIGH : LOW);
  }
}

void DoorController::pruneActivationTimes(uint32_t now) {
  std::vector<uint32_t> keep;
  keep.reserve(activations_.size());
  for (uint32_t ts : activations_) {
    if (now - ts <= cfg_.lockoutWindowMs) {
      keep.push_back(ts);
    }
  }
  activations_ = keep;
}

bool DoorController::checkLockout(String* reason) {
  if (isLockedOut()) {
    if (reason) *reason = "locked_out";
    return false;
  }
  if (reason) *reason = "ok";
  return true;
}

bool DoorController::computeIdleOutputLevel(bool& level) const {
  if (cfg_.idleMode == "high") {
    level = HIGH;
    return true;
  }
  if (cfg_.idleMode == "low") {
    level = LOW;
    return true;
  }
  return false;
}

void DoorController::driveOutputLevel(bool level) {
  if (cfg_.pin < 0) return;
  pinMode(cfg_.pin, OUTPUT);
  digitalWrite(cfg_.pin, level ? HIGH : LOW);
}
