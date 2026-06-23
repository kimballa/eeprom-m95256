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
    if (_faultyByteSet && _faultyByte >= addr && _faultyByte < addr + avail) {
      // Simulate a bad cell: this byte never durably holds what was written.
      _data[_faultyByte] ^= 0xFF;
    }
    if (_writeDelayMillis > 0) {
      advanceFakeMillis(_writeDelayMillis);
    }
    return avail;
  }

  size_t size() const { return _data.size(); }
  uint8_t &byteAt(size_t i) { return _data.at(i); }

  void zeroAll() { std::fill(_data.begin(), _data.end(), 0); }

  /** Make every subsequent write() advance the (fake) clock by `ms` before
   * returning, to simulate a device whose write commit stalls. Tests use
   * this to exercise EepromPageManager::storeRecord()'s stall-triggered
   * relocation path without a real sleep. */
  void setWriteDelayMillis(unsigned long ms) { _writeDelayMillis = ms; }

  /** Make any write touching byte `addr` corrupt that byte afterward,
   * simulating a bad memory cell. Tests use this to make
   * PageBackedData::store()'s post-write readback deterministically
   * disagree with what was written. */
  void setFaultyByte(addr_t addr) {
    _faultyByte = addr;
    _faultyByteSet = true;
  }

private:
  size_t available(addr_t addr, size_t count) const {
    if (addr >= _data.size()) {
      return 0;
    }
    return std::min(count, _data.size() - static_cast<size_t>(addr));
  }

  std::vector<uint8_t> _data;
  unsigned long _writeDelayMillis = 0;
  addr_t _faultyByte = 0;
  bool _faultyByteSet = false;
};

#endif /* _FAKE_EEPROM_H */
