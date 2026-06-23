// (c) Copyright 2026 Aaron Kimball
//
// Minimal desktop stand-in for Arduino.h, providing just enough of the API
// surface that src/eeprom-m95256.h and src/eeprom-wearlevel.h reference, so
// that the library can be compiled and unit tested with a plain x86_64 g++.
//
// digitalWrite() is wired to fakeSpiBusInstance() to frame chip-select
// transitions for the SpiEeprom<> driver tests; see fake_spi_bus.h.

#ifndef _TEST_HARNESS_ARDUINO_H
#define _TEST_HARNESS_ARDUINO_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdarg>

#include "fake_spi_bus.h"

constexpr int OUTPUT = 1;
constexpr int INPUT = 0;
constexpr int HIGH = 1;
constexpr int LOW = 0;
constexpr int MSBFIRST = 1;
constexpr int SPI_MODE0 = 0;

inline void pinMode(int /*pin*/, int /*mode*/) {}

inline void digitalWrite(int /*pin*/, int level) {
  if (fakeSpiBusInstance() == nullptr) {
    return;
  }
  if (level == LOW) {
    fakeSpiBusInstance()->beginTransaction();
  } else {
    fakeSpiBusInstance()->endTransaction();
  }
}

inline void delayMicroseconds(unsigned int) {}
inline void delayNanoseconds(unsigned int) {}

template <typename T> constexpr T min(T a, T b) { return a < b ? a : b; }
template <typename T> constexpr T max(T a, T b) { return a > b ? a : b; }

struct SerialClass {
  void println(const char *s) { std::printf("%s\n", s); }
  void printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
  }
};

inline SerialClass Serial;

#endif /* _TEST_HARNESS_ARDUINO_H */
