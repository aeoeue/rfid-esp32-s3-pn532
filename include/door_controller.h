#pragma once

#include <Arduino.h>
#include <vector>

struct DoorControllerConfig {
  bool enabled = false;
  int pin = 4;
  bool activeHigh = true;
  uint32_t pulseMs = 800;
  String idleMode = "low";
  uint32_t maxPulseMs = 5000;
  uint8_t lockoutLimit = 3;
  uint32_t lockoutWindowMs = 15000;
  uint32_t lockoutMs = 10000;
};

class DoorController {
 public:
  void configure(const DoorControllerConfig& config);
  void loop();

  bool requestPulse(String* reason = nullptr);
  bool setManual(bool on, String* reason = nullptr);

  bool enabled() const;
  bool isActive() const;
  bool isLockedOut() const;
  bool currentLogicalState() const;
  uint32_t pulseRemainingMs() const;
  uint32_t lockoutRemainingMs() const;
  uint8_t activationCountInWindow() const;
  String statusText() const;

 private:
  void applyIdle();
  void pruneActivationTimes(uint32_t now);
  bool checkLockout(String* reason);
  bool computeIdleOutputLevel(bool& level) const;
  void driveOutputLevel(bool level);

  DoorControllerConfig cfg_;
  bool active_ = false;
  bool logicalState_ = false;
  uint32_t activeUntilMs_ = 0;
  uint32_t lockoutUntilMs_ = 0;
  std::vector<uint32_t> activations_;
};
