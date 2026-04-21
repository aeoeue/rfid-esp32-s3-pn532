#include "tag_reader.h"

#include <Adafruit_PN532.h>
#include <SPI.h>

namespace {

String uidToHex(const uint8_t* uid, uint8_t uidLength) {
  String s;
  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

class Pn532SpiReader final : public ITagReader {
 public:
  bool begin(const ReaderConfig& config) override {
    cfg_ = config;
    return initReader();
  }

  void loop() override {
    if (state_ != ReaderState::Ready) {
      if (millis() - lastInitAttemptMs_ >= cfg_.reinitIntervalMs) {
        initReader();
      }
      return;
    }

    if (currentUid_.length() > 0 && millis() - lastSeenMs_ > cfg_.removeAfterMs) {
      currentUid_ = "";
    }
  }

  bool poll(TagReadResult& out) override {
    if (state_ != ReaderState::Ready) return false;
    uint32_t now = millis();
    if (now - lastPollMs_ < cfg_.pollIntervalMs) return false;
    lastPollMs_ = now;

    uint8_t uidBytes[7] = {0};
    uint8_t uidLength = 0;
    bool found = nfc_ != nullptr &&
                 nfc_->readPassiveTargetID(PN532_MIFARE_ISO14443A, uidBytes, &uidLength, cfg_.pollIntervalMs);
    if (!found || uidLength == 0) return false;

    String uid = uidToHex(uidBytes, uidLength);
    currentUid_ = uid;
    lastSeenMs_ = now;

    if (uid == lastDeliveredUid_ && now - lastDeliveredMs_ < cfg_.duplicateSuppressMs) {
      return false;
    }

    lastDeliveredUid_ = uid;
    lastDeliveredMs_ = now;
    out.uid = uid;
    out.timestampMs = now;
    return true;
  }

  String readerName() const override { return "PN532 SPI"; }

  String statusText() const override {
    switch (state_) {
      case ReaderState::Ready:
        return currentUid_.length() > 0 ? "ready/tag-present" : "ready/idle";
      case ReaderState::NotFound:
        return "not-detected";
      case ReaderState::Error:
        return "error";
      case ReaderState::Uninitialized:
      default:
        return "uninitialized";
    }
  }

  ReaderState state() const override { return state_; }
  String currentUid() const override { return currentUid_; }
  uint32_t currentUidAgeMs() const override { return currentUid_.isEmpty() ? 0 : millis() - lastSeenMs_; }
  int lastErrorCode() const override { return lastErrorCode_; }

 private:
  bool initReader() {
    lastInitAttemptMs_ = millis();
    lastErrorCode_ = 0;
    currentUid_ = "";

    if (nfc_ != nullptr) {
      delete nfc_;
      nfc_ = nullptr;
    }

    SPI.end();
    SPI.begin(cfg_.pinSck, cfg_.pinMiso, cfg_.pinMosi, cfg_.pinSs);

    if (cfg_.pinRst >= 0) {
      pinMode(cfg_.pinRst, OUTPUT);
      digitalWrite(cfg_.pinRst, LOW);
      delay(5);
      digitalWrite(cfg_.pinRst, HIGH);
      delay(20);
    }

    nfc_ = new Adafruit_PN532(cfg_.pinSs);
    nfc_->begin();

    uint32_t version = nfc_->getFirmwareVersion();
    if (!version) {
      state_ = ReaderState::NotFound;
      lastErrorCode_ = 1;
      return false;
    }

    if (!nfc_->SAMConfig()) {
      state_ = ReaderState::Error;
      lastErrorCode_ = 2;
      return false;
    }

    nfc_->setPassiveActivationRetries(0x01);
    state_ = ReaderState::Ready;
    lastErrorCode_ = 0;
    return true;
  }

  ReaderConfig cfg_;
  Adafruit_PN532* nfc_ = nullptr;
  ReaderState state_ = ReaderState::Uninitialized;
  int lastErrorCode_ = 0;
  uint32_t lastPollMs_ = 0;
  uint32_t lastSeenMs_ = 0;
  uint32_t lastDeliveredMs_ = 0;
  uint32_t lastInitAttemptMs_ = 0;
  String currentUid_;
  String lastDeliveredUid_;
};

}  // namespace

const char* readerStateToString(ReaderState state) {
  switch (state) {
    case ReaderState::Ready: return "ready";
    case ReaderState::NotFound: return "not_found";
    case ReaderState::Error: return "error";
    case ReaderState::Uninitialized:
    default:
      return "uninitialized";
  }
}

ITagReader* createPn532SpiReader() {
  return new Pn532SpiReader();
}
