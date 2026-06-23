// (c) Copyright 2026 Aaron Kimball
//
// Tests for EepromPageManager in src/eeprom-wearlevel.h: root-page
// formatting/validation, initial page placement for a set of DataMappable
// records, reload-time pageNum resync, and storeRecord()'s
// detect-and-relocate behavior on a failed (mismatched or stalled) write.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

#include <cstring>
#include <memory>
#include <set>
#include <vector>

namespace {

constexpr size_t kPageSize = 32;

struct Widget {
  uint32_t id;
  int32_t value;

  bool operator==(const Widget &o) const { return id == o.id && value == o.value; }
};

using Record = PageBackedData<Widget, kPageSize>;

/** Build `count` Record objects with ids 0..count-1, all backed by `eeprom`,
 * plus a DataMappable* pointer array to hand to EepromPageManager's ctor. */
struct RecordSet {
  RecordSet(GenericSpiEeprom &eeprom, size_t count) {
    records.reserve(count);
    for (size_t i = 0; i < count; i++) {
      records.push_back(std::make_unique<Record>(static_cast<Record::addr_t>(i), eeprom));
    }
    for (auto &r : records) {
      ptrs.push_back(r.get());
    }
  }

  std::vector<std::unique_ptr<Record>> records;
  std::vector<DataMappable *> ptrs;
};

/** Directly craft and write a root page payload (with a correct CRC32
 * trailer) bypassing EepromPageManager, so tests can probe
 * checkRootPageValid()'s individual failure modes independently. */
void writeRawRootPage(FakeEeprom &eeprom, const EepromRootPageData &rootData) {
  uint8_t buf[sizeof(EepromRootPageData) + sizeof(uint32_t)];
  memcpy(buf, &rootData, sizeof(rootData));
  uint32_t crc = computeCrc32(buf, sizeof(rootData));
  memcpy(buf + sizeof(rootData), &crc, sizeof(crc));
  eeprom.write(buf, 0, sizeof(buf));
}

/** Number of pages needed for `numMapPages` to be 1 in all these tests: a
 * handful of single-page records easily fit in one chunk page. With
 * kPageSize=32, _bytesPerMapPage=28, so up to 14 records fit in one map
 * page -- well above what any test below needs. firstDataPageNum is
 * therefore always DATA_MAP_START_PAGE_NUM (3) + 1 = 4. */
constexpr size_t kFirstDataPageNum = 4;

template <size_t numPages> using Mgr = EepromPageManager<numPages * kPageSize * 8, kPageSize>;

} // namespace

TEST_SUITE("EepromPageManager") {

TEST_CASE("ROOT_PAGE_MAGIC matches the specified constant") {
  CHECK(Mgr<8>::ROOT_PAGE_MAGIC == 0x0571AC94);
  CHECK(Mgr<8>::SYSTEM_FORMAT_VERSION == 1);
}

TEST_CASE("formatNew() makes the root page valid and reports it via checkRootPageValid()") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/7);

  CHECK(mgr.formatNew() == true);
  CHECK(mgr.checkRootPageValid() == Mgr<kNumPages>::ROOT_PAGE_OK);
}

TEST_CASE("formatNew() assigns each record a unique page at/after firstDataPageNum, "
          "marked in-use, and reflected in the data map") {
  constexpr size_t kNumPages = kFirstDataPageNum + 5;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 5);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 7);

  CHECK(mgr.formatNew() == true);
  CHECK(mgr.firstDataPageNum() == kFirstDataPageNum);

  std::set<size_t> seenPages;
  for (auto *rec : recs.ptrs) {
    size_t p = rec->getPageNum();
    CHECK(p >= mgr.firstDataPageNum());
    CHECK(mgr.usage().isPageInUse(p));
    CHECK(seenPages.insert(p).second); // No two records share a page.
    CHECK(mgr.dataMap().getStartPage(rec->getRecordId()) == p);
  }
}

TEST_CASE("formatNew() reserves the root/deadlist/usage/data-map pages as in-use") {
  constexpr size_t kNumPages = kFirstDataPageNum + 1;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.formatNew() == true);
  for (size_t p = 0; p < mgr.firstDataPageNum(); p++) {
    CHECK(mgr.usage().isPageInUse(p));
  }
}

TEST_CASE("formatNew() returns false when there isn't enough free space for every record") {
  // One page short of what 5 single-page records need.
  constexpr size_t kNumPages = kFirstDataPageNum + 4;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 5);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.formatNew() == false);
}

TEST_CASE("checkRootPageValid()/init() report ROOT_PAGE_ERR_LOAD_FAILED on a "
          "never-formatted (factory-reset, all-0xFF) EEPROM") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.checkRootPageValid() == Mgr<kNumPages>::ROOT_PAGE_ERR_LOAD_FAILED);
  CHECK(mgr.init() == Mgr<kNumPages>::ROOT_PAGE_ERR_LOAD_FAILED);
}

TEST_CASE("checkRootPageValid() reports ROOT_PAGE_ERR_BAD_MAGIC for a structurally "
          "valid page with the wrong magic number") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 9);

  EepromRootPageData rootData{};
  rootData.magic = 0xDEADBEEF;
  rootData.systemVersion = Mgr<kNumPages>::SYSTEM_FORMAT_VERSION;
  rootData.appVersion = 9;
  writeRawRootPage(eeprom, rootData);

  CHECK(mgr.checkRootPageValid() == Mgr<kNumPages>::ROOT_PAGE_ERR_BAD_MAGIC);
}

TEST_CASE("checkRootPageValid() reports ROOT_PAGE_ERR_BAD_SYSTEM_VERSION for an "
          "unrecognized system format version") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 9);

  EepromRootPageData rootData{};
  rootData.magic = Mgr<kNumPages>::ROOT_PAGE_MAGIC;
  rootData.systemVersion = Mgr<kNumPages>::SYSTEM_FORMAT_VERSION + 1;
  rootData.appVersion = 9;
  writeRawRootPage(eeprom, rootData);

  CHECK(mgr.checkRootPageValid() == Mgr<kNumPages>::ROOT_PAGE_ERR_BAD_SYSTEM_VERSION);
}

TEST_CASE("checkRootPageValid()/init() report ROOT_PAGE_ERR_BAD_APP_VERSION when the "
          "stored app version doesn't match what the application expects") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  {
    RecordSet recs(eeprom, 2);
    Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/5);
    CHECK(mgr.formatNew() == true);
  }

  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/6);
  CHECK(mgr.checkRootPageValid() == Mgr<kNumPages>::ROOT_PAGE_ERR_BAD_APP_VERSION);
  CHECK(mgr.init() == Mgr<kNumPages>::ROOT_PAGE_ERR_BAD_APP_VERSION);
}

TEST_CASE("init() reports ERR_DATA_MAP_CORRUPT when a data-map page's CRC32 doesn't match") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  {
    RecordSet recs(eeprom, 2);
    Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
    CHECK(mgr.formatNew() == true);
  }

  // Flip a bit inside the data map's serialized page without fixing up its CRC32.
  eeprom.byteAt(Mgr<kNumPages>::DATA_MAP_START_PAGE_NUM * kPageSize) ^= 0x01;

  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.init() == Mgr<kNumPages>::ERR_DATA_MAP_CORRUPT);
}

TEST_CASE("init() resyncs each known record's pageNum from the reloaded data map, "
          "surviving a simulated power cycle, without loading record data") {
  constexpr size_t kNumPages = kFirstDataPageNum + 3;
  FakeEeprom eeprom(kNumPages * kPageSize);
  std::vector<Record::addr_t> assignedPages;
  {
    RecordSet recs(eeprom, 3);
    Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 42);
    CHECK(mgr.formatNew() == true);
    for (auto *rec : recs.ptrs) {
      assignedPages.push_back(rec->getPageNum());
    }
  }

  // A fresh set of record objects, as if after a reboot: pageNum defaults to 0.
  RecordSet recs(eeprom, 3);
  for (auto *rec : recs.ptrs) {
    CHECK(rec->getPageNum() == 0);
  }

  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 42);
  CHECK(mgr.init() == Mgr<kNumPages>::ROOT_PAGE_OK);

  for (size_t i = 0; i < recs.ptrs.size(); i++) {
    CHECK(recs.ptrs[i]->getPageNum() == assignedPages[i]);
    // init() does not auto-load record data: it's still default-constructed.
    CHECK(recs.records[i]->data == Widget{});
  }
}

TEST_CASE("storeRecord() persists data without relocating on a clean write") {
  constexpr size_t kNumPages = kFirstDataPageNum + 2;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 2);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{11, 22};
  size_t originalPage = rec.getPageNum();

  CHECK(mgr.storeRecord(rec) == true);
  CHECK(rec.getPageNum() == originalPage); // No relocation occurred.
  CHECK(mgr.deadList().isPageLive(originalPage));

  Record reloaded(rec.getRecordId(), eeprom);
  reloaded.setPageNum(rec.getPageNum());
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == rec.data);
}

TEST_CASE("storeRecord() relocates a record when the write fails its readback verification") {
  constexpr size_t kNumPages = kFirstDataPageNum + 3;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{1, 2};
  size_t oldPage = rec.getPageNum();

  // Simulate a bad cell: any write touching the old page's first byte gets
  // silently corrupted, so PageBackedData::store()'s readback will disagree.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == true); // Succeeds after relocating elsewhere.

  size_t newPage = rec.getPageNum();
  CHECK(newPage != oldPage);
  CHECK(mgr.deadList().isPageDead(oldPage));
  CHECK(mgr.usage().isPageFree(oldPage));
  CHECK(mgr.usage().isPageInUse(newPage));
  CHECK(mgr.dataMap().getStartPage(rec.getRecordId()) == newPage);

  Record reloaded(rec.getRecordId(), eeprom);
  reloaded.setPageNum(static_cast<Record::addr_t>(newPage));
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == rec.data);
}

TEST_CASE("storeRecord() relocates a record when the write stalls past the threshold") {
  constexpr size_t kNumPages = kFirstDataPageNum + 3;
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{3, 4};
  size_t oldPage = rec.getPageNum();

  resetFakeMillis();
  eeprom.setWriteDelayMillis(Mgr<kNumPages>::WRITE_STALL_THRESHOLD_MILLIS + 5);

  CHECK(mgr.storeRecord(rec) == true);
  CHECK(rec.getPageNum() != oldPage);
  CHECK(mgr.deadList().isPageDead(oldPage));

  Record reloaded(rec.getRecordId(), eeprom);
  reloaded.setPageNum(rec.getPageNum());
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == rec.data);
}

TEST_CASE("storeRecord() relocation skips over pages already marked dead") {
  constexpr size_t kNumPages = kFirstDataPageNum + 3; // data pages: 4, 5, 6.
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  size_t oldPage = rec.getPageNum();
  CHECK(oldPage == kFirstDataPageNum); // The only record, so it gets the first data page.

  mgr.deadList().markPageDead(kFirstDataPageNum + 1); // Pre-mark the next page dead.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == true);
  CHECK(rec.getPageNum() == kFirstDataPageNum + 2); // +1 was skipped because it's dead.
}

TEST_CASE("storeRecord() returns false when no free page is available to relocate to") {
  constexpr size_t kNumPages = kFirstDataPageNum + 1; // Exactly one data page, no spares.
  FakeEeprom eeprom(kNumPages * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<kNumPages> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  size_t oldPage = rec.getPageNum();
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == false);
  // The old page is still marked dead, even though relocation itself failed.
  CHECK(mgr.deadList().isPageDead(oldPage));
}

TEST_CASE("a large EEPROM whose deadlist/usage list each span multiple hardware "
          "pages places the usage list and data map correctly, with no overlap "
          "between system structures") {
  // 1 MiB EEPROM, 64-byte pages -> 16384 total pages. An EepromPageBitmap
  // covering 16384 pages needs a (16384/8)=2048-byte table, which spans
  // ceil(2048/64)=32 hardware pages -- not the single page that fits a
  // small test EEPROM's deadlist/usage list.
  constexpr size_t kBitSize = 1024UL * 1024 * 8;
  constexpr size_t kHwPageSize = 64;
  using BigMgr = EepromPageManager<kBitSize, kHwPageSize>;
  using BigRecord = PageBackedData<Widget, kHwPageSize>;

  CHECK(BigMgr::ROOT_PAGE_NUM == 0);
  CHECK(BigMgr::DEAD_LIST_PAGE_NUM == 1);
  CHECK(BigMgr::USAGE_LIST_PAGE_NUM == 33);     // 1 + 32 deadlist pages.
  CHECK(BigMgr::DATA_MAP_START_PAGE_NUM == 65); // 33 + 32 usage-list pages.

  FakeEeprom eeprom(1024UL * 1024);

  auto makeRecords = [&eeprom]() {
    std::vector<std::unique_ptr<BigRecord>> records;
    std::vector<DataMappable *> ptrs;
    for (size_t i = 0; i < 3; i++) {
      records.push_back(std::make_unique<BigRecord>(static_cast<BigRecord::addr_t>(i), eeprom));
      ptrs.push_back(records.back().get());
    }
    return std::make_pair(std::move(records), std::move(ptrs));
  };

  auto [records, ptrs] = makeRecords();
  BigMgr mgr(eeprom, ptrs.data(), ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);
  CHECK(mgr.firstDataPageNum() >= BigMgr::DATA_MAP_START_PAGE_NUM);
  CHECK(mgr.checkRootPageValid() == BigMgr::ROOT_PAGE_OK);

  std::set<size_t> seenPages;
  for (auto *rec : ptrs) {
    size_t p = rec->getPageNum();
    CHECK(p >= mgr.firstDataPageNum());
    CHECK(seenPages.insert(p).second); // No two records share a page.
  }

  // Exercise pages far out in the deadlist/usage bitmaps -- ones that live in
  // a later hardware page of each multi-page table -- and confirm they
  // round-trip across a simulated reboot without disturbing the root page,
  // each other, or the data map.
  constexpr size_t kFarPage = 16000;
  mgr.deadList().markPageDead(kFarPage);
  mgr.usage().markPageInUse(kFarPage + 1);

  auto [reloadRecords, reloadPtrs] = makeRecords();
  BigMgr reloaded(eeprom, reloadPtrs.data(), reloadPtrs.size(), 1);
  CHECK(reloaded.init() == BigMgr::ROOT_PAGE_OK);
  CHECK(reloaded.deadList().isPageDead(kFarPage));
  CHECK(reloaded.usage().isPageInUse(kFarPage + 1));

  for (size_t i = 0; i < ptrs.size(); i++) {
    CHECK(reloadPtrs[i]->getPageNum() == ptrs[i]->getPageNum());
  }
}

}
