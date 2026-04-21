#pragma once

#include <Arduino.h>

enum class ReaderState : uint8_t {
  Uninitialized = 0,
  Ready = 1,
  NotFound = 2,
  Error = 3,
};

struct ReaderConfig {
  int pinSck = 12;
  int pinMiso = 13;
  int pinMosi = 11;
  int pinSs = 10;
  int pinRst = -1;
  uint32_t pollIntervalMs = 40;
  uint32_t removeAfterMs = 700;
  uint32_t duplicateSuppressMs = 250;
  uint32_t reinitIntervalMs = 15000;
};

struct TagReadResult {
  String uid;
  uint32_t timestampMs = 0;
};

class ITagReader {
 public:
  virtual ~ITagReader() = default;
  virtual bool begin(const ReaderConfig& config) = 0;
  virtual void loop() = 0;
  virtual bool poll(TagReadResult& out) = 0;
  virtual String readerName() const = 0;
  virtual String statusText() const = 0;
  virtual ReaderState state() const = 0;
  virtual String currentUid() const = 0;
  virtual uint32_t currentUidAgeMs() const = 0;
  virtual int lastErrorCode() const = 0;
};

const char* readerStateToString(ReaderState state);
ITagReader* createPn532SpiReader();
