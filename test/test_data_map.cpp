// (c) Copyright 2026 Aaron Kimball
//
// Tests for DataMap in src/eeprom-wearlevel.h: per-record start-page
// tracking, multi-page chunked serialization, dual-copy (A/B) crash-safe
// persistence, and corruption/torn-write handling.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

#include <memory>
#include <vector>

namespace {

// payloadPerPage == kPageSize - 8 == 56 bytes == 14 addr_t (uint32_t) entries.
constexpr size_t kPageSize = 64;
constexpr size_t kEntriesPerPage = (kPageSize - 8) / sizeof(uint32_t);

class DummyRecord : public DataMappable {
public:
  explicit DummyRecord(addr_t id) : _id(id) {}
  size_t sizeInPages() const override { return 1; }
  addr_t getRecordId() const override { return _id; }
  addr_t getPageNum() const override { return _pageNum; }
  void setPageNum(addr_t newPageNum) override { _pageNum = newPageNum; }
  bool load() override { return true; }
  bool store() override { return true; }
  bool verify() const override { return true; }

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
      records.push_back(
          std::make_unique<DummyRecord>(static_cast<DataMappable::addr_t>(i)));
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
  DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());

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
    DataMap<kPageSize> map(eeprom, 10, 11, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 6);
    map.setStartPage(1, 7);
    map.setStartPage(2, 8);
    CHECK(map.flush());
  }

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 10, 11, recs.ptrs.data(),
                             recs.ptrs.size());
  CHECK(reloaded.init() == true);
  CHECK(reloaded.getStartPage(0) == 6);
  CHECK(reloaded.getStartPage(1) == 7);
  CHECK(reloaded.getStartPage(2) == 8);
}

TEST_CASE("a zero-record DataMap needs no pages and flush/init are harmless no-ops") {
  RecordSet recs(0);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  CHECK(map.numMapPages() == 0);
  CHECK(map.flush());
  CHECK(map.init() == true);
}

TEST_CASE("a record count that exactly fills one chunk page uses exactly one page") {
  RecordSet recs(kEntriesPerPage);
  FakeEeprom eeprom(4096);
  DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  CHECK(map.numMapPages() == 1);

  for (size_t i = 0; i < kEntriesPerPage; i++) {
    map.setStartPage(static_cast<DataMappable::addr_t>(i),
                     static_cast<DataMappable::addr_t>(100 + i));
  }
  CHECK(map.flush());

  RecordSet reloadRecs(kEntriesPerPage);
  DataMap<kPageSize> reloaded(eeprom, 0, 1, reloadRecs.ptrs.data(),
                             reloadRecs.ptrs.size());
  CHECK(reloaded.init() == true);
  for (size_t i = 0; i < kEntriesPerPage; i++) {
    CHECK(reloaded.getStartPage(static_cast<DataMappable::addr_t>(i)) ==
          100 + i);
  }
}

TEST_CASE("a record count one more than fits in a page spans two chunk pages") {
  constexpr size_t kCount = kEntriesPerPage + 1;
  RecordSet recs(kCount);
  FakeEeprom eeprom(4096);
  // Two pages per copy: copy A occupies pages 0,1; copy B pages 2,3.
  DataMap<kPageSize> map(eeprom, 0, 2, recs.ptrs.data(), recs.ptrs.size());
  CHECK(map.numMapPages() == 2);

  for (size_t i = 0; i < kCount; i++) {
    map.setStartPage(static_cast<DataMappable::addr_t>(i),
                     static_cast<DataMappable::addr_t>(200 + i));
  }
  CHECK(map.flush());

  RecordSet reloadRecs(kCount);
  DataMap<kPageSize> reloaded(eeprom, 0, 2, reloadRecs.ptrs.data(),
                             reloadRecs.ptrs.size());
  CHECK(reloaded.init() == true);
  for (size_t i = 0; i < kCount; i++) {
    CHECK(reloaded.getStartPage(static_cast<DataMappable::addr_t>(i)) ==
          200 + i);
  }
}

TEST_CASE("corruption of one redundant copy still loads cleanly from the other") {
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(3);
    DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 6);
    map.setStartPage(1, 7);
    map.setStartPage(2, 8);
    map.formatBoth(); // Both copies written and valid.
  }

  // Corrupt a payload byte of copy A's page 0 (payload starts after the
  // 4-byte seqId preamble). Copy B is still intact.
  eeprom.byteAt(0 * kPageSize + 4) ^= 0xFF;

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == true); // Recovered from copy B.
  CHECK(reloaded.getStartPage(0) == 6);
  CHECK(reloaded.getStartPage(1) == 7);
  CHECK(reloaded.getStartPage(2) == 8);
}

TEST_CASE("corruption of BOTH redundant copies of a page is reported by init()") {
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(3);
    DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 6);
    map.formatBoth();
  }

  eeprom.byteAt(0 * kPageSize + 4) ^= 0x01; // copy A, page 0
  eeprom.byteAt(1 * kPageSize + 4) ^= 0x01; // copy B, page 0

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == false);
}

TEST_CASE("a torn flush leaves the previous committed values intact (A/B safety)") {
  FakeEeprom eeprom(4096);
  {
    RecordSet recs(3);
    DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
    map.setStartPage(0, 11);
    map.setStartPage(1, 22);
    map.setStartPage(2, 33);
    map.formatBoth(); // Baseline committed to both copies.

    // Change an entry, but lose power partway through writing the (single
    // dirty) page to its inactive slot.
    map.setStartPage(1, 99);
    eeprom.resetWriteCount();
    eeprom.armPowerLoss(/*atWrite=*/1, /*partialBytes=*/8);
    map.flush();
  }

  RecordSet recs(3);
  DataMap<kPageSize> reloaded(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  CHECK(reloaded.init() == true);
  CHECK(reloaded.getStartPage(0) == 11);
  CHECK(reloaded.getStartPage(1) == 22); // Reverted: the torn change was lost.
  CHECK(reloaded.getStartPage(2) == 33);
}

TEST_CASE("verify() reports ok for cleanly formatted copies and bad for a "
          "fully-corrupted page") {
  FakeEeprom eeprom(4096);
  RecordSet recs(3);
  DataMap<kPageSize> map(eeprom, 0, 1, recs.ptrs.data(), recs.ptrs.size());
  map.setStartPage(0, 6);
  map.formatBoth();
  CHECK(map.verify() == true);

  eeprom.byteAt(0 * kPageSize + 4) ^= 0x01;
  eeprom.byteAt(1 * kPageSize + 4) ^= 0x01;
  CHECK(map.verify() == false);
}

}
