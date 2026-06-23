// (c) Copyright 2026 Aaron Kimball
//
// Test-only emulation of an M95256 SPI EEPROM's wire protocol, just faithful
// enough to drive SpiEeprom<>'s real read()/write() page-splitting and
// chip-select framing logic end-to-end on a desktop build, with no hardware.
//
// This is NOT a stand-in for GenericSpiEeprom. It plugs in underneath the
// Arduino.h/SPI.h stubs (digitalWrite() for chip-select framing, SPI.transfer()
// for the byte stream) so that the *actual* SpiEeprom<bitSize, hwPageSizeBytes>
// template gets exercised exactly as it would be against real silicon.

#ifndef _FAKE_SPI_BUS_H
#define _FAKE_SPI_BUS_H

#include <cstddef>
#include <cstdint>
#include <vector>

// Re-declared here (matching the declaration in src/eeprom-m95256.h, defined
// in src/eeprom-m95256.cpp) rather than included, to avoid a circular
// include: Arduino.h (the test harness stand-in) includes this file, and
// eeprom-m95256.h includes Arduino.h.
extern volatile bool forceSuppressEepromWrite;

class FakeSpiBus {
public:
  explicit FakeSpiBus(size_t sizeBytes) : _mem(sizeBytes, 0) {}

  /** Begin a chip-select-asserted transaction: reset the byte-stream parser. */
  void beginTransaction() {
    _phase = Phase::Opcode;
    _addr = 0;
    _addrBytesLeft = 2;
    _countedThisWrite = false;
  }

  /** End the chip-select-asserted transaction. If this was a WRITE-opcode
   * transaction, arm the configured number of "busy" RDSR polls (see
   * setBusyPollsPerWrite()), simulating that the device hasn't yet
   * committed the page it was just sent. */
  void endTransaction() {
    if (_opcode == OPCODE_WRITE) {
      _busyPollsRemaining = _busyPollsPerWrite;
    }
    _phase = Phase::Idle;
  }

  /** Emulate one byte of full-duplex SPI transfer against the fake device. */
  uint8_t transfer(uint8_t out) {
    switch (_phase) {
    case Phase::Opcode:
      _opcode = out;
      _phase = (_opcode == OPCODE_READ || _opcode == OPCODE_WRITE) ? Phase::Address
                                                                     : Phase::Data;
      return 0xFF;

    case Phase::Address:
      _addr = static_cast<uint16_t>((_addr << 8) | out);
      if (--_addrBytesLeft == 0) {
        _phase = Phase::Data;
      }
      return 0xFF;

    case Phase::Data:
      return handleData(out);

    case Phase::Idle:
    default:
      return 0xFF;
    }
  }

  /** Directly inspect/corrupt the fake device's backing memory, for tests. */
  uint8_t &byteAt(size_t i) { return _mem.at(i); }
  size_t size() const { return _mem.size(); }

  /** Number of WRITE-opcode transactions completed since construction; lets
   * tests confirm a multi-page write took the expected number of distinct
   * page transactions. */
  size_t writeTransactionCount() const { return _writeTransactionCount; }

  /** After each completed WRITE-opcode transaction, make the next `n` RDSR
   * polls report the device as busy (WIP bit set) before reporting ready
   * again, simulating a real device's page-commit latency. Defaults to 0
   * (never busy), matching this fake's original always-ready behavior. */
  void setBusyPollsPerWrite(size_t n) { _busyPollsPerWrite = n; }

  /** Set the global `forceSuppressEepromWrite` flag as soon as the `n`'th
   * WRITE-opcode transaction completes, simulating a brownout/IRQ tripping
   * it mid-transfer (rather than a test setting it before the call even
   * begins, which only proves the *first* check works). 0 (the default)
   * disables this. */
  void setSuppressWriteAfterTransactions(size_t n) { _suppressAfterTransactions = n; }

private:
  enum class Phase { Idle, Opcode, Address, Data };

  static constexpr uint8_t OPCODE_WREN = 0x06;
  static constexpr uint8_t OPCODE_RDSR = 0x05;
  static constexpr uint8_t OPCODE_READ = 0x03;
  static constexpr uint8_t OPCODE_WRITE = 0x02;

  // Bit 0 of the M95256 status register: write-in-progress (see
  // EEPROM_STATUS_REG_WRITE_IN_PROGRESS in eeprom-m95256.h). Hardcoded here
  // rather than including that header, since this file deliberately models
  // only the wire protocol, not the higher-level driver's vocabulary.
  static constexpr uint8_t STATUS_REG_WIP_BIT = 0x01;

  uint8_t handleData(uint8_t out) {
    switch (_opcode) {
    case OPCODE_RDSR:
      if (_busyPollsRemaining > 0) {
        _busyPollsRemaining--;
        return STATUS_REG_WIP_BIT;
      }
      return 0x00;

    case OPCODE_READ: {
      uint8_t v = (_addr < _mem.size()) ? _mem[_addr] : 0xFF;
      _addr++;
      return v;
    }

    case OPCODE_WRITE:
      if (_addr < _mem.size()) {
        _mem[_addr] = out;
      }
      _addr++;
      if (!_countedThisWrite) {
        _writeTransactionCount++;
        _countedThisWrite = true;
        if (_suppressAfterTransactions > 0 && _writeTransactionCount >= _suppressAfterTransactions) {
          // Simulate a brownout/IRQ tripping the write-suppression flag
          // partway through a multi-page transfer, right after this page's
          // commit -- so the driver's next per-page loop iteration sees it.
          forceSuppressEepromWrite = true;
        }
      }
      return 0xFF;

    case OPCODE_WREN:
    default:
      return 0xFF;
    }
  }

  Phase _phase = Phase::Idle;
  uint8_t _opcode = 0;
  uint16_t _addr = 0;
  int _addrBytesLeft = 0;
  bool _countedThisWrite = false;
  size_t _writeTransactionCount = 0;
  size_t _busyPollsPerWrite = 0;
  size_t _busyPollsRemaining = 0;
  size_t _suppressAfterTransactions = 0;
  std::vector<uint8_t> _mem;
};

/** Process-wide fake bus that the Arduino.h/SPI.h stubs delegate to. */
inline FakeSpiBus *&fakeSpiBusInstance() {
  static FakeSpiBus *instance = nullptr;
  return instance;
}

/** (Re)create the fake bus with a fresh, zeroed backing memory of `sizeBytes`.
 * Call this before constructing/using an SpiEeprom<>-derived object in a test. */
inline FakeSpiBus &resetFakeSpiBus(size_t sizeBytes) {
  delete fakeSpiBusInstance();
  fakeSpiBusInstance() = new FakeSpiBus(sizeBytes);
  return *fakeSpiBusInstance();
}

#endif /* _FAKE_SPI_BUS_H */
