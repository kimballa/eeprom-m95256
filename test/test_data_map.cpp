// (c) Copyright 2026 Aaron Kimball
//
// Tests for DataMap in src/eeprom-wearlevel.h: per-record start-page
// tracking, multi-page chunked serialization with per-chunk CRC32, and
// corruption detection on reload.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

#include <memory>
#include <vector>

namespace {

constexpr size_t kPageSize = 64; // _bytesPerMapPage == 60 for this page size.

class DummyRecord : public DataMappable {
public:
  explicit DummyRecord(addr_t id) : _id(id) {}
  size_t sizeInPages() const override { return 1; }
  addr_t getRecordId() const override { return _id; }
  addr_t getPageNum() const override { return _pageNum; }
  void setPageNum(addr_t newPageNum) override { _pageNum = newPageNum; }
  bool load() override { return true; }
  bool store() override { return true; }

private:
  addr_t _id;
  addr_t _pageNum = 0;
};

// Build `count` DummyRecord objects with ids 0..count-1, and a pointer array
// to hand to DataMap's constructor.
struct RecordSet {
  explicit RecordSet(size_t count) {
    records.reserve(count);
    for (size_t i = 0; i < count; i++) {
      records.push_back(std::make_unique<DummyRecord>(static_cast<DataMappable::addr_t>(i)));
    }
    for (auto &r : records) {
      ptrs.push_back(r.get());
    }
  }

  std::vector<std::unique_ptr<DummyRecord>> records;
  std::vector<const DataMappable *> ptrs;
};

} // namespace

TEST_SUITE("DataMap") {

TEST_CASE("setStartPage/getStartPage round-trip in memory without flushing") {
  RecordSet recs(3);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());

  CHECK(map.getStartPage(0) == 0);
  map.setStartPage(0, 6);
  map.setStartPage(1, 7);
  map.setStartPage(2, 42);

  CHECK(map.getStartPage(0) == 6);
  CHECK(map.getStartPage(1) == 7);
  CHECK(map.getStartPage(2) == 42);
}

TEST_CASE("flush() persists exactly what was set, surviving a simulated power cycle") {
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(3);
    DataMap<kPageSize> map(eeprom, 10, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 6);
    map.setStartPage(1, 7);
    map.setStartPage(2, 8);
    map.flush();
  }

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 10, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == true);
  CHECK(reloaded.getStartPage(0) == 6);
  CHECK(reloaded.getStartPage(1) == 7);
  CHECK(reloaded.getStartPage(2) == 8);
}

TEST_CASE("a zero-record DataMap needs no pages and flush/init are harmless no-ops") {
  RecordSet recs(0);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());
  map.flush();
  CHECK(map.init() == true);
}

TEST_CASE("a record count that exactly fills one chunk page uses exactly one page") {
  // _bytesPerMapPage == kPageSize - 4 == 60 bytes == 30 addr_t entries.
  constexpr size_t kRecordsPerPage = (kPageSize - 4) / sizeof(uint16_t);
  RecordSet recs(kRecordsPerPage);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());

  for (size_t i = 0; i < kRecordsPerPage; i++) {
    map.setStartPage(static_cast<DataMappable::addr_t>(i), static_cast<DataMappable::addr_t>(100 + i));
  }
  map.flush();

  // Byte just past the single page's data+CRC region must be untouched
  // (still factory-reset 0xFF), confirming the write didn't spill into a
  // second page.
  CHECK(eeprom.byteAt(kPageSize) == 0xFF);

  RecordSet reloadRecs(kRecordsPerPage);
  DataMap<kPageSize> reloaded(eeprom, 0, reloadRecs.ptrs.data(), reloadRecs.ptrs.size());
  CHECK(reloaded.init() == true);
  for (size_t i = 0; i < kRecordsPerPage; i++) {
    CHECK(reloaded.getStartPage(static_cast<DataMappable::addr_t>(i)) == 100 + i);
  }
}

TEST_CASE("a record count one more than fits in a page spans two chunk pages, "
          "with a shorter final page") {
  constexpr size_t kRecordsPerPage = (kPageSize - 4) / sizeof(uint16_t);
  constexpr size_t kCount = kRecordsPerPage + 1;
  RecordSet recs(kCount);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());

  for (size_t i = 0; i < kCount; i++) {
    map.setStartPage(static_cast<DataMappable::addr_t>(i), static_cast<DataMappable::addr_t>(200 + i));
  }
  map.flush();

  RecordSet reloadRecs(kCount);
  DataMap<kPageSize> reloaded(eeprom, 0, reloadRecs.ptrs.data(), reloadRecs.ptrs.size());
  CHECK(reloaded.init() == true);
  for (size_t i = 0; i < kCount; i++) {
    CHECK(reloaded.getStartPage(static_cast<DataMappable::addr_t>(i)) == 200 + i);
  }
}

TEST_CASE("corrupting one byte of a chunk's data is detected by init() via CRC32") {
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(3);
    DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 6);
    map.setStartPage(1, 7);
    map.setStartPage(2, 8);
    map.flush();
  }

  // Flip a bit in the serialized array's first entry, leaving the CRC32
  // trailer untouched -> mismatch.
  eeprom.byteAt(0) ^= 0x01;

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == false);
}

TEST_CASE("corruption in one chunk page does not prevent other chunk pages "
          "from loading correctly") {
  constexpr size_t kRecordsPerPage = (kPageSize - 4) / sizeof(uint16_t);
  constexpr size_t kCount = kRecordsPerPage + 2; // Spans two chunk pages.
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(kCount);
    DataMap<kPageSize> map(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());
    for (size_t i = 0; i < kCount; i++) {
      map.setStartPage(static_cast<DataMappable::addr_t>(i), static_cast<DataMappable::addr_t>(300 + i));
    }
    map.flush();
  }

  // Corrupt a byte that lives in the *second* chunk page only.
  eeprom.byteAt(kPageSize + 1) ^= 0xFF;

  RecordSet recs(kCount);
  DataMap<kPageSize> reloaded(eeprom, 0, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == false); // Overall corruption is reported...

  // ...but the untouched first chunk's records are still intact.
  for (size_t i = 0; i < kRecordsPerPage; i++) {
    CHECK(reloaded.getStartPage(static_cast<DataMappable::addr_t>(i)) == 300 + i);
  }
}

TEST_CASE("DataMap stored at a nonzero start page does not disturb earlier pages") {
  FakeEeprom eeprom(4096);
  // Put a sentinel byte right before the map's start page; flush() must not
  // touch it.
  RecordSet recs(2);
  DataMap<kPageSize> map(eeprom, 1, recs.ptrs.data(), recs.ptrs.size());
  eeprom.byteAt(kPageSize - 1) = 0xAB; // Last byte of page 0, just before page 1.

  map.setStartPage(0, 1);
  map.setStartPage(1, 2);
  map.flush();

  CHECK(eeprom.byteAt(kPageSize - 1) == 0xAB);
}

}
