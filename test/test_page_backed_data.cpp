// (c) Copyright 2026 Aaron Kimball
//
// Tests for PageBackedData<T> in src/eeprom-wearlevel.h: single-page
// load/store with a CRC32 trailer, and page reassignment.

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
  PageBackedData<Widget, kPageSize> pbd(/*recordId=*/5, /*pageNum=*/0, eeprom);
  CHECK(pbd.getRecordId() == 5);
  CHECK(pbd.sizeInPages() == 1);
}

TEST_CASE("store() then load() round-trips the data exactly") {
  FakeEeprom eeprom(4096);
  Widget original{42, -7, "hello"};

  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, /*pageNum=*/2, eeprom);
  CHECK(writer.store() == sizeof(Widget) + sizeof(uint32_t));

  // A separate object, same page: simulates reloading after a reboot.
  PageBackedData<Widget, kPageSize> reader(/*recordId=*/0, /*pageNum=*/2, eeprom);
  CHECK(reader.load() == true);
  CHECK(reader.data == original);
}

TEST_CASE("load() detects a corrupted data byte via CRC32 and leaves `data` untouched") {
  FakeEeprom eeprom(4096);
  Widget original{1, 100, "abc"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, /*pageNum=*/0, eeprom);
  writer.store();

  eeprom.byteAt(0) ^= 0xFF; // Corrupt the first byte of the stored Widget.

  Widget sentinel{999, -999, "zzzz"};
  PageBackedData<Widget, kPageSize> reader(sentinel, /*recordId=*/0, /*pageNum=*/0, eeprom);
  CHECK(reader.load() == false);
  CHECK(reader.data == sentinel); // Unchanged: load() must not partially apply bad data.
}

TEST_CASE("load() detects a corrupted CRC trailer even if the data is untouched") {
  FakeEeprom eeprom(4096);
  Widget original{2, 5, "xy"};
  PageBackedData<Widget, kPageSize> writer(original, /*recordId=*/0, /*pageNum=*/1, eeprom);
  writer.store();

  // Corrupt a byte within the CRC32 trailer (immediately after sizeof(Widget)).
  eeprom.byteAt(kPageSize + sizeof(Widget)) ^= 0xFF;

  PageBackedData<Widget, kPageSize> reader(/*recordId=*/0, /*pageNum=*/1, eeprom);
  CHECK(reader.load() == false);
}

TEST_CASE("setPageNum()/getPageNum() move where store()/load() target without "
          "touching the previous page's contents") {
  FakeEeprom eeprom(4096);
  Widget v{7, 1, "a"};
  PageBackedData<Widget, kPageSize> pbd(v, /*recordId=*/0, /*pageNum=*/0, eeprom);
  pbd.store();

  pbd.setPageNum(3);
  CHECK(pbd.getPageNum() == 3);
  pbd.store();

  // The old page's bytes are still there, untouched by the move.
  PageBackedData<Widget, kPageSize> readOld(/*recordId=*/0, /*pageNum=*/0, eeprom);
  CHECK(readOld.load() == true);
  CHECK(readOld.data == v);

  PageBackedData<Widget, kPageSize> readNew(/*recordId=*/0, /*pageNum=*/3, eeprom);
  CHECK(readNew.load() == true);
  CHECK(readNew.data == v);
}

TEST_CASE("two records on different pages do not interfere with each other") {
  FakeEeprom eeprom(4096);
  Widget a{1, 10, "a"};
  Widget b{2, 20, "b"};

  PageBackedData<Widget, kPageSize> pa(a, /*recordId=*/0, /*pageNum=*/0, eeprom);
  PageBackedData<Widget, kPageSize> pb(b, /*recordId=*/1, /*pageNum=*/1, eeprom);
  pa.store();
  pb.store();

  PageBackedData<Widget, kPageSize> ra(/*recordId=*/0, /*pageNum=*/0, eeprom);
  PageBackedData<Widget, kPageSize> rb(/*recordId=*/1, /*pageNum=*/1, eeprom);
  CHECK(ra.load() == true);
  CHECK(rb.load() == true);
  CHECK(ra.data == a);
  CHECK(rb.data == b);
}

}
