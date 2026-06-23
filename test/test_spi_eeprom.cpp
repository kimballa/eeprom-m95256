// (c) Copyright 2026 Aaron Kimball
//
// Tests for the real SpiEeprom<bitSize, hwPageSizeBytes>/EepromM95256 driver
// in src/eeprom-m95256.h, run against the FakeSpiBus emulation of the M95256
// wire protocol (see test/harness/fake_spi_bus.h). These exercise the actual
// page-splitting, multi-transaction write, and address-clamping logic that
// every higher-level class in this library ultimately depends on.

#include "doctest.h"

#include "eeprom-m95256.h"
#include "fake_spi_bus.h"

#include <cstring>
#include <vector>

namespace {
// 256 bytes of EEPROM (2048 bits), 64-byte hardware pages -> 4 pages. Small
// and fast, while still exercising real multi-page splits.
using TestEeprom = SpiEeprom<2048, 64>;
} // namespace

TEST_SUITE("SpiEeprom (real driver, via FakeSpiBus)") {

TEST_CASE("a write within a single page round-trips exactly via read()") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  uint8_t out[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
  CHECK(dev.write(out, 5, sizeof(out)) == sizeof(out));

  uint8_t in[10] = {0};
  CHECK(dev.read(in, 5, sizeof(in)) == sizeof(in));
  CHECK(memcmp(in, out, sizeof(out)) == 0);
}

TEST_CASE("a write spanning multiple hardware pages lands at the correct "
          "byte offsets and uses one SPI write transaction per page") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  // Start 10 bytes before a page boundary (page size 64) and write 100
  // bytes, so this straddles 3 pages: [54,64), [64,128), [128,154).
  std::vector<uint8_t> out(100);
  for (size_t i = 0; i < out.size(); i++) {
    out[i] = static_cast<uint8_t>(i);
  }

  CHECK(dev.write(out.data(), 54, out.size()) == out.size());
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 3);

  std::vector<uint8_t> in(100, 0);
  CHECK(dev.read(in.data(), 54, in.size()) == in.size());
  CHECK(in == out);
}

TEST_CASE("a write/read exactly at a page boundary uses exactly one page-write "
          "transaction") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  uint8_t out[64];
  for (size_t i = 0; i < sizeof(out); i++) {
    out[i] = static_cast<uint8_t>(0xA0 + i % 16);
  }
  CHECK(dev.write(out, 64, sizeof(out)) == sizeof(out));
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 1);

  uint8_t in[64] = {0};
  CHECK(dev.read(in, 64, sizeof(in)) == sizeof(in));
  CHECK(memcmp(in, out, sizeof(out)) == 0);
}

TEST_CASE("a write that would exceed the top of the address space is "
          "truncated, and only the in-range bytes are actually written") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  // Device is 256 bytes (addr 0..255). Try to write 20 bytes starting at 250
  // -> only 6 bytes (250..255) fit.
  uint8_t out[20];
  memset(out, 0x42, sizeof(out));
  size_t written = dev.write(out, 250, sizeof(out));
  CHECK(written == 6);

  uint8_t in[6] = {0};
  CHECK(dev.read(in, 250, sizeof(in)) == 6);
  for (uint8_t b : in) {
    CHECK(b == 0x42);
  }
}

TEST_CASE("a read that would exceed the top of the address space is truncated") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  uint8_t in[20] = {0};
  CHECK(dev.read(in, 250, sizeof(in)) == 6);
}

TEST_CASE("reads and writes starting beyond the address space are rejected outright") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  uint8_t buf[4] = {1, 2, 3, 4};
  CHECK(dev.write(buf, 256, sizeof(buf)) == 0);
  CHECK(dev.read(buf, 256, sizeof(buf)) == 0);
}

TEST_CASE("setting the global write-suppression flag aborts an in-progress "
          "multi-page write early") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  forceSuppressEepromWrite = true;
  std::vector<uint8_t> out(150, 0x55); // Would span 3 pages if allowed.
  size_t written = dev.write(out.data(), 0, out.size());
  forceSuppressEepromWrite = false; // Always restore global state.

  CHECK(written < out.size());
}

TEST_CASE("setting the global write-suppression flag before any write begins "
          "aborts it immediately, with zero bytes written") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  forceSuppressEepromWrite = true;
  uint8_t out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  size_t written = dev.write(out, 0, sizeof(out));
  forceSuppressEepromWrite = false;

  CHECK(written == 0);
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 0);
}

TEST_CASE("a multi-page write suppressed partway through (e.g. by a brownout IRQ "
          "firing mid-transfer) commits exactly the pages already in flight and "
          "stops before the next one") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  // Trip forceSuppressEepromWrite itself as soon as the 2nd page-write
  // transaction completes, simulating an IRQ firing mid-transfer rather than
  // a test just setting the flag before write() is even called.
  fakeSpiBusInstance()->setSuppressWriteAfterTransactions(2);

  std::vector<uint8_t> out(150, 0x55); // 3 pages: [0,64), [64,128), [128,150).
  size_t written = dev.write(out.data(), 0, out.size());
  forceSuppressEepromWrite = false; // Always restore global state.

  CHECK(written == 128); // Exactly the first 2 pages; the 3rd never began.
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 2);
}

TEST_CASE("isWriteInProgress()/waitForWriteComplete() reflect the device's WIP "
          "bit, and write() does not wait out the final page's own commit") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  fakeSpiBusInstance()->setBusyPollsPerWrite(2);
  uint8_t out[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  CHECK(dev.write(out, 0, sizeof(out)) == sizeof(out));

  // The single page committed by this write armed 2 busy polls, and write()
  // must not have waited them out itself.
  CHECK(dev.isWriteInProgress() == true);
  CHECK(dev.isWriteInProgress() == true);
  CHECK(dev.isWriteInProgress() == false); // Exactly 2 busy polls, then clear.

  // Re-arm and confirm waitForWriteComplete() actually loops until clear,
  // rather than returning early.
  fakeSpiBusInstance()->setBusyPollsPerWrite(4);
  CHECK(dev.write(out, 0, sizeof(out)) == sizeof(out));
  dev.waitForWriteComplete();
  CHECK(dev.isWriteInProgress() == false);
}

TEST_CASE("a multi-page write still lands all its data correctly even when the "
          "device reports busy between each page's commit") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();
  fakeSpiBusInstance()->setBusyPollsPerWrite(2);

  std::vector<uint8_t> out(150);
  for (size_t i = 0; i < out.size(); i++) {
    out[i] = static_cast<uint8_t>(i);
  }

  CHECK(dev.write(out.data(), 0, out.size()) == out.size());
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 3);

  std::vector<uint8_t> in(out.size(), 0);
  CHECK(dev.read(in.data(), 0, in.size()) == in.size());
  CHECK(in == out);
}

TEST_CASE("a single-byte write/read at the very last valid address round-trips, "
          "and one byte past it is out of range") {
  resetFakeSpiBus(256);
  TestEeprom dev(10);
  dev.setup();

  uint8_t out = 0x7E;
  CHECK(dev.write(&out, 255, 1) == 1); // Device is 256 bytes: addr 0..255.

  uint8_t in = 0;
  CHECK(dev.read(&in, 255, 1) == 1);
  CHECK(in == out);

  uint8_t buf[1];
  CHECK(dev.write(buf, 256, 1) == 0);
  CHECK(dev.read(buf, 256, 1) == 0);
}

TEST_CASE("EepromM95256 (the real 256Kbit/64-byte-page part) constructs, sets "
          "up, and reports its geometry correctly") {
  resetFakeSpiBus(256 * 1024 / 8);
  EepromM95256 dev(10);
  dev.setup();

  CHECK(dev.getSizeBytes() == 256 * 1024 / 8);
  CHECK(dev.getPageSize() == 64);
}

TEST_CASE("a differently-sized SpiEeprom<> instantiation (small EEPROM, small "
          "pages) still splits multi-page writes correctly") {
  // 128 bytes (1024 bits), 8-byte pages -> 16 pages: a much smaller, "odder"
  // geometry than the 64-byte-page configs used elsewhere in this file, to
  // confirm the driver's page-splitting logic is genuinely generic.
  using TinyPageEeprom = SpiEeprom<1024, 8>;
  resetFakeSpiBus(128);
  TinyPageEeprom dev(10);
  dev.setup();

  CHECK(dev.getSizeBytes() == 128);
  CHECK(dev.getPageSize() == 8);

  // Starting 3 bytes before a page boundary and writing 50 bytes spans 7
  // pages: [5,8), [8,16), [16,24), [24,32), [32,40), [40,48), [48,55).
  std::vector<uint8_t> out(50);
  for (size_t i = 0; i < out.size(); i++) {
    out[i] = static_cast<uint8_t>(0xC0 + i);
  }
  CHECK(dev.write(out.data(), 5, out.size()) == out.size());
  CHECK(fakeSpiBusInstance()->writeTransactionCount() == 7);

  std::vector<uint8_t> in(out.size(), 0);
  CHECK(dev.read(in.data(), 5, in.size()) == in.size());
  CHECK(in == out);
}

}
