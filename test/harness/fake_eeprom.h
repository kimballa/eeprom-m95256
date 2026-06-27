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
  // Real EEPROMs read back as all-0xFF after a factory reset/erase, not
  // all-zero; match that so cold-start detection logic (e.g. CRC32 checks,
  // WearLevelingPageData's writeSequenceId discovery) is exercised the same
  // way it would be against real silicon.
  explicit FakeEeprom(size_t sizeBytes) : _data(sizeBytes, 0xFF) {}

  size_t read(void *buf, addr_t addr, size_t count) override {
    size_t avail = available(addr, count);
    memcpy(buf, _data.data() + addr, avail);
    return avail;
  }

  size_t write(void *buf, addr_t addr, size_t count) override {
    // Once the simulated power has been lost, the device is dead: no further
    // bytes ever reach the array, just as on a real MCU whose CPU has stopped.
    if (_poweredOff) {
      return 0;
    }

    _writeCount++;

    if (_powerLossArmed && _writeCount == _powerLossAtWrite) {
      // Simulate losing power partway through this page program: only the
      // first `_powerLossPartialBytes` bytes of this write durably land (0 for
      // a clean inter-write boundary, a fraction for a torn page), and every
      // subsequent write is a no-op. Reads still return whatever committed.
      size_t avail = available(addr, count);
      size_t partial = std::min(_powerLossPartialBytes, avail);
      memcpy(_data.data() + addr, buf, partial);
      _poweredOff = true;
      return partial;
    }

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

  /** Number of write() calls seen so far (each is a single page-level write
   * for the higher-level structures). Tests use this to enumerate every
   * possible interruption point of a multi-write operation. */
  size_t writeCount() const { return _writeCount; }
  void resetWriteCount() { _writeCount = 0; }

  /** Arm a simulated power loss: the `atWrite`-th subsequent write() call
   * (1-based) persists only its first `partialBytes` bytes, after which the
   * device is dead and all further writes are no-ops. `partialBytes == 0`
   * models power lost cleanly between two page writes; a value in
   * (0, pageSize) models a torn page program whose CRC will fail. */
  void armPowerLoss(size_t atWrite, size_t partialBytes = 0) {
    _powerLossArmed = true;
    _powerLossAtWrite = atWrite;
    _powerLossPartialBytes = partialBytes;
  }
  bool poweredOff() const { return _poweredOff; }

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

  /** Clear a previously-set faulty byte, so writes to it stick again. */
  void clearFaultyByte() { _faultyByteSet = false; }

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

  size_t _writeCount = 0;
  bool _powerLossArmed = false;
  size_t _powerLossAtWrite = 0;
  size_t _powerLossPartialBytes = 0;
  bool _poweredOff = false;
};

#endif /* _FAKE_EEPROM_H */
