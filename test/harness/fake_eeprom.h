// (c) Copyright 2026 Aaron Kimball
//
// In-memory stand-in for a GenericSpiEeprom, for testing the higher-level
// page/wear-leveling classes (EepromPageBitmap, DataMap, PageBackedData, ...)
// without going through the SPI wire protocol at all. Reads/writes operate
// directly on a RAM-backed byte array so test code can inspect or corrupt the
// "device" contents directly, to verify CRC/error-detection behavior.
//
// (For tests that need to exercise the real SPI wire protocol itself -- e.g.
// page-splitting and address-clamping in SpiEeprom<> -- see fake_spi_bus.h
// instead.)

#ifndef _FAKE_EEPROM_H
#define _FAKE_EEPROM_H

#include "eeprom-m95256.h"

#include <algorithm>
#include <cstring>
#include <vector>

class FakeEeprom : public GenericSpiEeprom {
public:
  explicit FakeEeprom(size_t sizeBytes) : _data(sizeBytes, 0) {}

  size_t read(void *buf, addr_t addr, size_t count) override {
    size_t avail = available(addr, count);
    memcpy(buf, _data.data() + addr, avail);
    return avail;
  }

  size_t write(void *buf, addr_t addr, size_t count) override {
    size_t avail = available(addr, count);
    memcpy(_data.data() + addr, buf, avail);
    return avail;
  }

  size_t size() const { return _data.size(); }
  uint8_t &byteAt(size_t i) { return _data.at(i); }

  void zeroAll() { std::fill(_data.begin(), _data.end(), 0); }

private:
  size_t available(addr_t addr, size_t count) const {
    if (addr >= _data.size()) {
      return 0;
    }
    return std::min(count, _data.size() - static_cast<size_t>(addr));
  }

  std::vector<uint8_t> _data;
};

#endif /* _FAKE_EEPROM_H */
