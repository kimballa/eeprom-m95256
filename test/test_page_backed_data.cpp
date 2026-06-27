// (c) Copyright 2026 Aaron Kimball
//
// Tests for PageBackedData<T> in src/eeprom-wearlevel.h: single-page
// load/store with a CRC32 trailer, write verification via readback, and page
// reassignment.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

#include <cstring>

namespace {

constexpr size_t kPageSize = 32;

struct Widget {
  uint32_t id;
  int16_t value;
  char label[8];

  bool operator==(const Widget &o) const {
    return id == o.id && value == o.value && memcmp(label, o.label, sizeof(label)) == 0;
  }
};

} // namespace

TEST_SUITE("PageBackedData") {

TEST_CASE("getRecordId()/sizeInPages() satisfy the DataMappable contract") {
  FakeEeprom eeprom(4096);
  PageBackedData<Widget, kPageSize> pbd(/*recordId=*/5, eeprom);
  CHECK(pbd.getRecordId() == 5);
  CHECK(pbd.sizeInPages() == 1);
}

TEST_CASE("a freshly constructed PageBackedData defaults to page 0") {
  FakeEeprom eeprom(4096);
  PageBackedData<Widget, kPageSize> pbd(/*recordId=*/0, eeprom);
  CHECK(pbd.getPageNum() == 0);
}

TEST_CASE("store() then load() round-trips the data exactly") {
  FakeEeprom eeprom(4096);
  Widget original{42, -7, "hello"};

  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);
  writer.setPageNum(2);
  CHECK(writer.store() == true);

  // A separate object, same page: simulates reloading after a reboot.
  PageBackedData<Widget, kPageSize> reader(/*recordId=*/0, eeprom);
  reader.setPageNum(2);
  CHECK(reader.load() == true);
  CHECK(reader.data == original);
}

TEST_CASE("store() fails when the eeprom is too small to hold data + CRC32 trailer") {
  Widget original{1, 2, "abc"};
  FakeEeprom tinyEeprom(sizeof(Widget)); // Too small for data + CRC32 trailer.
  PageBackedData<Widget, kPageSize> truncatedWriter(original, /*recordId=*/0, tinyEeprom);
  CHECK(truncatedWriter.store() == false);
}

TEST_CASE("load() detects a corrupted data byte via CRC32 and leaves `data` untouched") {
  FakeEeprom eeprom(4096);
  Widget original{1, 100, "abc"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);
  writer.store();

  eeprom.byteAt(0) ^= 0xFF; // Corrupt the first byte of the stored Widget.

  Widget sentinel{999, -999, "zzzz"};
  PageBackedData<Widget, kPageSize> reader(sentinel, /*recordId=*/0, eeprom);
  CHECK(reader.load() == false);
  CHECK(reader.data == sentinel); // Unchanged: load() must not partially apply bad data.
}

TEST_CASE("load() detects a corrupted CRC trailer even if the data is untouched") {
  FakeEeprom eeprom(4096);
  Widget original{2, 5, "xy"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);
  writer.setPageNum(1);
  writer.store();

  // Corrupt a byte within the CRC32 trailer (immediately after sizeof(Widget)).
  eeprom.byteAt(kPageSize + sizeof(Widget)) ^= 0xFF;

  PageBackedData<Widget, kPageSize> reader(/*recordId=*/0, eeprom);
  reader.setPageNum(1);
  CHECK(reader.load() == false);
}

TEST_CASE("setPageNum()/getPageNum() move where store()/load() target without "
          "touching the previous page's contents") {
  FakeEeprom eeprom(4096);
  Widget v{7, 1, "a"};
  PageBackedData<Widget, kPageSize> pbd(v, /*recordId=*/0, eeprom);
  pbd.store(); // Defaults to page 0.

  pbd.setPageNum(3);
  CHECK(pbd.getPageNum() == 3);
  pbd.store();

  // The old page's bytes are still there, untouched by the move.
  PageBackedData<Widget, kPageSize> readOld(/*recordId=*/0, eeprom);
  CHECK(readOld.load() == true);
  CHECK(readOld.data == v);

  PageBackedData<Widget, kPageSize> readNew(/*recordId=*/0, eeprom);
  readNew.setPageNum(3);
  CHECK(readNew.load() == true);
  CHECK(readNew.data == v);
}

TEST_CASE("two records on different pages do not interfere with each other") {
  FakeEeprom eeprom(4096);
  Widget a{1, 10, "a"};
  Widget b{2, 20, "b"};

  PageBackedData<Widget, kPageSize> pa(a, /*recordId=*/0, eeprom);
  PageBackedData<Widget, kPageSize> pb(b, /*recordId=*/1, eeprom);
  pb.setPageNum(1);
  pa.store();
  pb.store();

  PageBackedData<Widget, kPageSize> ra(/*recordId=*/0, eeprom);
  PageBackedData<Widget, kPageSize> rb(/*recordId=*/1, eeprom);
  rb.setPageNum(1);
  CHECK(ra.load() == true);
  CHECK(rb.load() == true);
  CHECK(ra.data == a);
  CHECK(rb.data == b);
}

TEST_CASE("FakeEeprom models the WIP bit, reachable through a GenericSpiEeprom& "
          "(vtable wiring for isWriteInProgress())") {
  FakeEeprom eeprom(4096);
  GenericSpiEeprom &dev = eeprom; // Exercise the virtual dispatch path.

  CHECK(dev.isWriteInProgress() == false); // No config: ready immediately.

  eeprom.setWipPollsPerWrite(2);
  uint8_t payload[4] = {1, 2, 3, 4};
  dev.write(payload, 0, sizeof(payload));
  CHECK(dev.isWriteInProgress() == true);
  CHECK(dev.isWriteInProgress() == true);
  CHECK(dev.isWriteInProgress() == false); // Exactly 2 busy polls, then ready.
}

TEST_CASE("storeAsync()/verifyAsync() commit and round-trip when the write "
          "commits immediately") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  Widget original{42, -7, "async"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);
  writer.setPageNum(2);

  uint8_t buf[kPageSize];
  writer.storeAsync(buf);
  // No WIP polls armed, so the very first verifyAsync() sees the write done.
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);

  PageBackedData<Widget, kPageSize> reader(/*recordId=*/0, eeprom);
  reader.setPageNum(2);
  CHECK(reader.load() == true);
  CHECK(reader.data == original);
}

TEST_CASE("verifyAsync() reports IN_PROGRESS until the device's WIP bit clears, "
          "then SUCCESS") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  eeprom.setWipPollsPerWrite(2);
  Widget original{1, 2, "wip"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);

  uint8_t buf[kPageSize];
  writer.storeAsync(buf);
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::IN_PROGRESS);
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::IN_PROGRESS);
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);
  CHECK(isEepromWriteStalled == false);
}

TEST_CASE("verifyAsync() returns FAILED (no recovery at record level) when the "
          "readback does not match") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  Widget original{3, 4, "bad"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);
  writer.setPageNum(2);
  eeprom.setFaultyByte(2 * kPageSize); // Corrupt the page on every write to it.

  uint8_t buf[kPageSize];
  writer.storeAsync(buf);
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::FAILED);
  CHECK(isEepromWriteStalled == false); // A readback mismatch is not a stall.
}

TEST_CASE("verifyAsync() raises isEepromWriteStalled and FAILS when the WIP bit "
          "is stuck past the stall threshold") {
  isEepromWriteStalled = false;
  resetFakeMillis();
  FakeEeprom eeprom(4096);
  Widget original{5, 6, "stuck"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, eeprom);

  uint8_t buf[kPageSize];
  writer.storeAsync(buf);
  eeprom.setWipStuck(true);
  // Within the threshold it's still merely in progress.
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::IN_PROGRESS);
  CHECK(isEepromWriteStalled == false);

  advanceFakeMillis(WRITE_STALL_THRESHOLD_MILLIS + 1);
  CHECK(writer.verifyAsync(buf) == AsyncStoreStatus::FAILED);
  CHECK(isEepromWriteStalled == true);
}

TEST_CASE("an async store lands byte-for-byte identical bytes to a synchronous "
          "store of the same data") {
  isEepromWriteStalled = false;
  Widget original{77, -3, "same"};

  FakeEeprom syncEeprom(4096);
  PageBackedData<Widget, kPageSize> syncWriter(original, /*recordId=*/0, syncEeprom);
  syncWriter.setPageNum(2);
  CHECK(syncWriter.store() == true);

  FakeEeprom asyncEeprom(4096);
  PageBackedData<Widget, kPageSize> asyncWriter(original, /*recordId=*/0, asyncEeprom);
  asyncWriter.setPageNum(2);
  uint8_t buf[kPageSize];
  asyncWriter.storeAsync(buf);
  CHECK(asyncWriter.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);

  for (size_t i = 0; i < sizeof(Widget) + sizeof(uint32_t); i++) {
    CHECK(syncEeprom.byteAt(2 * kPageSize + i) ==
          asyncEeprom.byteAt(2 * kPageSize + i));
  }
}

}
