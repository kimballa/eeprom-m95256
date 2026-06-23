// (c) Copyright 2026 Aaron Kimball
//
// Minimal desktop stand-in for SPI.h, providing just enough of the API
// surface that src/eeprom-m95256.h references, so that the library can be
// compiled and unit tested with a plain x86_64 g++.
//
// SPIClass::transfer() is wired to fakeSpiBusInstance(); see fake_spi_bus.h.

#ifndef _TEST_HARNESS_SPI_H
#define _TEST_HARNESS_SPI_H

#include <cstdint>

#include "fake_spi_bus.h"

struct SPISettings {
  SPISettings(unsigned long /*clockHz*/, int /*bitOrder*/, int /*dataMode*/) {}
};

struct SPIClass {
  void beginTransaction(const SPISettings &) {}
  void endTransaction() {}

  uint8_t transfer(uint8_t out) {
    if (fakeSpiBusInstance() == nullptr) {
      return 0xFF;
    }
    return fakeSpiBusInstance()->transfer(out);
  }
};

inline SPIClass SPI;

#endif /* _TEST_HARNESS_SPI_H */
