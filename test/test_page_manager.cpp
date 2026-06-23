// (c) Copyright 2026 Aaron Kimball
//
// Tests for EepromPageManager in src/eeprom-wearlevel.h: root-page
// formatting/validation, initial page placement, reload-time pageNum resync,
// storeRecord()'s detect-and-relocate behavior (with the crash-safe write
// ordering and don't-free-the-old-page policy), the slow-write advisory path,
// verifyAll(), and power-loss recovery at every interruption point of
// formatNew() and of a relocation.

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
 * checkRootPageValid()'s individual failure modes independently. The root page
 * is a single-copy PageBackedData at hardware page 0. */
void writeRawRootPage(FakeEeprom &eeprom, const EepromRootPageData &rootData) {
  uint8_t buf[sizeof(EepromRootPageData) + sizeof(uint32_t)];
  memcpy(buf, &rootData, sizeof(rootData));
  uint32_t crc = computeCrc32(buf, sizeof(rootData));
  memcpy(buf + sizeof(rootData), &crc, sizeof(crc));
  eeprom.write(buf, 0, sizeof(buf));
}

// A roomy EEPROM (plenty of pages past firstDataPageNum) for the small tests.
constexpr size_t kBigEnoughPages = 40;
template <size_t numPages>
using Mgr = EepromPageManager<numPages * kPageSize * 8, kPageSize>;
using TestMgr = Mgr<kBigEnoughPages>;

} // namespace

TEST_SUITE("EepromPageManager") {

TEST_CASE("ROOT_PAGE_MAGIC matches the specified constant") {
  CHECK(TestMgr::ROOT_PAGE_MAGIC == 0x0571AC94);
  CHECK(TestMgr::SYSTEM_FORMAT_VERSION == 1);
}

TEST_CASE("formatNew() makes the root page valid and reports it via checkRootPageValid()") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/7);

  CHECK(mgr.formatNew() == true);
  CHECK(mgr.checkRootPageValid() == TestMgr::ROOT_PAGE_OK);
}

TEST_CASE("formatNew() assigns each record a unique page at/after firstDataPageNum, "
          "marked in-use, and reflected in the data map") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 5);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 7);

  CHECK(mgr.formatNew() == true);

  std::set<size_t> seenPages;
  for (auto *rec : recs.ptrs) {
    size_t p = rec->getPageNum();
    CHECK(p >= mgr.firstDataPageNum());
    CHECK(mgr.usage().isPageInUse(p));
    CHECK(seenPages.insert(p).second); // No two records share a page.
    CHECK(mgr.dataMap().getStartPage(rec->getRecordId()) == p);
  }
}

TEST_CASE("formatNew() reserves every system page (root + both A/B copies) as in-use") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 1);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.formatNew() == true);
  for (size_t p = 0; p < mgr.firstDataPageNum(); p++) {
    CHECK(mgr.usage().isPageInUse(p));
  }
}

TEST_CASE("formatNew() returns false when there isn't enough free space for every record") {
  // firstDataPageNum is 7 for this config; with 4 data pages (11 total) there
  // is not room for 5 single-page records.
  FakeEeprom eeprom(11 * kPageSize);
  RecordSet recs(eeprom, 5);
  Mgr<11> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.firstDataPageNum() == 7);
  CHECK(mgr.formatNew() == false);
}

TEST_CASE("checkRootPageValid()/init() report ROOT_PAGE_ERR_LOAD_FAILED on a "
          "never-formatted (factory-reset, all-0xFF) EEPROM") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);

  CHECK(mgr.checkRootPageValid() == TestMgr::ROOT_PAGE_ERR_LOAD_FAILED);
  CHECK(mgr.init() == TestMgr::ROOT_PAGE_ERR_LOAD_FAILED);
}

TEST_CASE("checkRootPageValid() reports ROOT_PAGE_ERR_BAD_MAGIC for a structurally "
          "valid page with the wrong magic number") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 9);

  EepromRootPageData rootData{};
  rootData.magic = 0xDEADBEEF;
  rootData.systemVersion = TestMgr::SYSTEM_FORMAT_VERSION;
  rootData.appVersion = 9;
  writeRawRootPage(eeprom, rootData);

  CHECK(mgr.checkRootPageValid() == TestMgr::ROOT_PAGE_ERR_BAD_MAGIC);
}

TEST_CASE("checkRootPageValid() reports ROOT_PAGE_ERR_BAD_SYSTEM_VERSION for an "
          "unrecognized system format version") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 9);

  EepromRootPageData rootData{};
  rootData.magic = TestMgr::ROOT_PAGE_MAGIC;
  rootData.systemVersion = TestMgr::SYSTEM_FORMAT_VERSION + 1;
  rootData.appVersion = 9;
  writeRawRootPage(eeprom, rootData);

  CHECK(mgr.checkRootPageValid() == TestMgr::ROOT_PAGE_ERR_BAD_SYSTEM_VERSION);
}

TEST_CASE("checkRootPageValid()/init() report ROOT_PAGE_ERR_BAD_APP_VERSION when the "
          "stored app version doesn't match what the application expects") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  {
    RecordSet recs(eeprom, 2);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/5);
    CHECK(mgr.formatNew() == true);
  }

  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), /*appFormatVersion=*/6);
  CHECK(mgr.checkRootPageValid() == TestMgr::ROOT_PAGE_ERR_BAD_APP_VERSION);
  CHECK(mgr.init() == TestMgr::ROOT_PAGE_ERR_BAD_APP_VERSION);
}

TEST_CASE("init() recovers a data map whose ONE redundant copy is corrupt, but "
          "reports ERR_DATA_MAP_CORRUPT when BOTH copies of a page are corrupt") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  {
    RecordSet recs(eeprom, 2);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
    CHECK(mgr.formatNew() == true);
  }

  // Corrupt copy A of the data map's first page; copy B still validates.
  eeprom.byteAt(TestMgr::DATA_MAP_START_PAGE_NUM * kPageSize + 4) ^= 0x01;
  {
    RecordSet recs(eeprom, 2);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
    CHECK(mgr.init() == TestMgr::ROOT_PAGE_OK); // Recovered from copy B.
  }

  // Now corrupt copy B's first page too: no valid copy remains. Copy B of the
  // map is the last system region, ending just before the first data page.
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  size_t mapBStart = mgr.firstDataPageNum() - mgr.dataMap().numMapPages();
  eeprom.byteAt(mapBStart * kPageSize + 4) ^= 0x01;
  CHECK(mgr.init() == TestMgr::ERR_DATA_MAP_CORRUPT);
}

TEST_CASE("init() reports ERR_BITMAP_CORRUPT when both copies of the deadlist are bad") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  {
    RecordSet recs(eeprom, 2);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
    CHECK(mgr.formatNew() == true);
  }

  // Copy A of the deadlist is at DEAD_LIST_PAGE_NUM; copy B starts right after
  // copy A of the data map.
  size_t deadB = TestMgr::DATA_MAP_START_PAGE_NUM + 1; // 1 map page in this config.
  eeprom.byteAt(TestMgr::DEAD_LIST_PAGE_NUM * kPageSize + 4) ^= 0x01;
  eeprom.byteAt(deadB * kPageSize + 4) ^= 0x01;

  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.init() == TestMgr::ERR_BITMAP_CORRUPT);
}

TEST_CASE("init() resyncs each known record's pageNum from the reloaded data map, "
          "surviving a simulated power cycle, without loading record data") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  std::vector<Record::addr_t> assignedPages;
  {
    RecordSet recs(eeprom, 3);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 42);
    CHECK(mgr.formatNew() == true);
    for (auto *rec : recs.ptrs) {
      assignedPages.push_back(rec->getPageNum());
    }
  }

  RecordSet recs(eeprom, 3);
  for (auto *rec : recs.ptrs) {
    CHECK(rec->getPageNum() == 0);
  }

  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 42);
  CHECK(mgr.init() == TestMgr::ROOT_PAGE_OK);

  for (size_t i = 0; i < recs.ptrs.size(); i++) {
    CHECK(recs.ptrs[i]->getPageNum() == assignedPages[i]);
    CHECK(recs.records[i]->data == Widget{}); // init() does not auto-load data.
  }
}

TEST_CASE("storeRecord() persists data without relocating on a clean write") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{11, 22};
  size_t originalPage = rec.getPageNum();

  CHECK(mgr.storeRecord(rec) == true);
  CHECK(rec.getPageNum() == originalPage);
  CHECK(mgr.deadList().isPageLive(originalPage));

  Record reloaded(rec.getRecordId(), eeprom);
  reloaded.setPageNum(rec.getPageNum());
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == rec.data);
}

TEST_CASE("storeRecord() relocates on a failed readback, marking the old page dead "
          "but leaving it in-use (never freed)") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 1);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{1, 2};
  size_t oldPage = rec.getPageNum();

  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == true); // Succeeds after relocating elsewhere.

  size_t newPage = rec.getPageNum();
  CHECK(newPage != oldPage);
  CHECK(mgr.deadList().isPageDead(oldPage));
  CHECK(mgr.usage().isPageInUse(oldPage)); // NOT freed: a dead page stays reserved.
  CHECK(mgr.usage().isPageInUse(newPage));
  CHECK(mgr.dataMap().getStartPage(rec.getRecordId()) == newPage);

  Record reloaded(rec.getRecordId(), eeprom);
  reloaded.setPageNum(static_cast<Record::addr_t>(newPage));
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == rec.data);
}

TEST_CASE("storeRecord() does NOT relocate on a merely-slow (but verifiable) write") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 1);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  rec.data = Widget{3, 4};
  size_t oldPage = rec.getPageNum();

  resetFakeMillis();
  eeprom.setWriteDelayMillis(TestMgr::WRITE_STALL_THRESHOLD_MILLIS + 5);

  CHECK(mgr.storeRecord(rec) == true);
  // The slow write was correct, so the advisory readback passes and the
  // record stays put -- no page is condemned.
  CHECK(rec.getPageNum() == oldPage);
  CHECK(mgr.deadList().isPageLive(oldPage));
}

TEST_CASE("storeRecord() relocation skips over pages already marked dead") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 1);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  size_t oldPage = rec.getPageNum();
  CHECK(oldPage == mgr.firstDataPageNum());

  mgr.deadList().markPageDead(oldPage + 1); // Pre-mark the next page dead.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == true);
  CHECK(rec.getPageNum() == oldPage + 2); // +1 was skipped because it's dead.
}

TEST_CASE("storeRecord() returns false when no free page is available to relocate to") {
  // Exactly one data page (firstDataPageNum == 7, total 8 pages).
  FakeEeprom eeprom(8 * kPageSize);
  RecordSet recs(eeprom, 1);
  Mgr<8> mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  Record &rec = *recs.records[0];
  size_t oldPage = rec.getPageNum();
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>(oldPage * kPageSize));

  CHECK(mgr.storeRecord(rec) == false);
  CHECK(mgr.deadList().isPageDead(oldPage));
}

TEST_CASE("verifyAll() passes for a fully written store, and flags a corrupted "
          "record and a corrupted data map") {
  FakeEeprom eeprom(kBigEnoughPages * kPageSize);
  RecordSet recs(eeprom, 2);
  TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
  CHECK(mgr.formatNew() == true);

  recs.records[0]->data = Widget{1, 1};
  recs.records[1]->data = Widget{2, 2};
  CHECK(mgr.storeRecord(*recs.records[0]) == true);
  CHECK(mgr.storeRecord(*recs.records[1]) == true);

  EepromVerifyResult good = mgr.verifyAll();
  CHECK(good.ok);
  CHECK(good.numBadRecords == 0);

  // Corrupt record 0's data page.
  eeprom.byteAt(recs.records[0]->getPageNum() * kPageSize) ^= 0xFF;
  EepromVerifyResult bad = mgr.verifyAll();
  CHECK_FALSE(bad.ok);
  CHECK(bad.numBadRecords == 1);
  CHECK(bad.dataMapOk); // Map itself is still fine.

  // Corrupt both copies of the data map's first page too.
  size_t mapBStart = mgr.firstDataPageNum() - mgr.dataMap().numMapPages();
  eeprom.byteAt(TestMgr::DATA_MAP_START_PAGE_NUM * kPageSize + 4) ^= 0x01;
  eeprom.byteAt(mapBStart * kPageSize + 4) ^= 0x01;
  EepromVerifyResult worse = mgr.verifyAll();
  CHECK_FALSE(worse.dataMapOk);
}

TEST_CASE("a large EEPROM whose deadlist/usage list each span multiple hardware "
          "pages lays out both A and B copies without overlap") {
  constexpr size_t kBitSize = 1024UL * 1024 * 8;
  constexpr size_t kHwPageSize = 64;
  using BigMgr = EepromPageManager<kBitSize, kHwPageSize>;
  using BigRecord = PageBackedData<Widget, kHwPageSize>;

  // 16384 pages -> 2048-byte bitmap table -> payload 56 -> 37 pages per copy.
  CHECK(BigMgr::DEAD_LIST_PAGE_NUM == 1);
  CHECK(BigMgr::USAGE_LIST_PAGE_NUM == 38);      // 1 + 37 deadlist pages.
  CHECK(BigMgr::DATA_MAP_START_PAGE_NUM == 75);  // 38 + 37 usage pages.

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
    CHECK(seenPages.insert(p).second);
  }

  // Exercise pages far out in the multi-page bitmaps and confirm they
  // round-trip across a reboot.
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

TEST_CASE("formatNew() is crash-safe: a power loss at ANY write leaves the EEPROM "
          "either fully formatted or unformatted, never half-valid-but-trusted") {
  // Reference: a clean format, to know the deterministic placement and the
  // total number of page writes involved.
  std::vector<Record::addr_t> refPages;
  size_t totalWrites = 0;
  {
    FakeEeprom eeprom(kBigEnoughPages * kPageSize);
    RecordSet recs(eeprom, 3);
    TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
    eeprom.resetWriteCount();
    CHECK(mgr.formatNew() == true);
    totalWrites = eeprom.writeCount();
    for (auto *rec : recs.ptrs) {
      refPages.push_back(rec->getPageNum());
    }
  }
  CHECK(totalWrites > 0);

  for (size_t k = 1; k <= totalWrites + 1; k++) {
    for (size_t partial : {size_t(0), size_t(8)}) {
      FakeEeprom eeprom(kBigEnoughPages * kPageSize);
      {
        RecordSet recs(eeprom, 3);
        TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
        eeprom.resetWriteCount();
        eeprom.armPowerLoss(k, partial);
        mgr.formatNew(); // May complete, return false, or be cut short.
      }

      // Reboot and inspect.
      RecordSet recs(eeprom, 3);
      TestMgr mgr(eeprom, recs.ptrs.data(), recs.ptrs.size(), 1);
      int status = mgr.init();
      if (status == TestMgr::ROOT_PAGE_OK) {
        // Root written last => a valid root implies a complete, consistent
        // format: structures verify and placement matches the clean run.
        EepromVerifyResult v = mgr.verifyAll();
        CHECK(v.deadListOk);
        CHECK(v.usageOk);
        CHECK(v.dataMapOk);
        for (size_t i = 0; i < recs.ptrs.size(); i++) {
          CHECK(recs.ptrs[i]->getPageNum() == refPages[i]);
        }
      }
      // Otherwise the device simply looks unformatted -- acceptable.
    }
  }
}

TEST_CASE("a record's data is never lost when a worn-out ring's relocation is "
          "interrupted by a power loss at ANY write") {
  // A WearLevelingPageData ring keeps its current value in a still-valid slot
  // even once it's worn out, so (unlike a bad-cell-triggered relocation) the
  // old copy is intact when relocation begins. Crash at any write during the
  // relocating storeRecord() and confirm the record reloads SOME valid value
  // (old or new) -- never neither -- with the metadata self-consistent.
  constexpr size_t kRingSize = 4;
  using Ring = WearLevelingPageData<Widget, kPageSize, kRingSize>;

  const Widget vOld{10, 100};
  const Widget vNew{20, 200};

  // Find how many writes the relocating storeRecord() takes (clean run).
  size_t relocateWrites = 0;
  {
    FakeEeprom eeprom(kBigEnoughPages * kPageSize);
    Ring ring(/*recordId=*/0, eeprom);
    DataMappable *records[] = {&ring};
    TestMgr mgr(eeprom, records, 1, 1);
    ring.setDeadPageOracle(mgr.deadList());
    CHECK(mgr.formatNew() == true);
    ring.data = vOld;
    CHECK(mgr.storeRecord(ring) == true);
    size_t base = ring.getPageNum();
    mgr.deadList().markPageDead(base + 1);
    mgr.deadList().markPageDead(base + 3); // Half dead: next store relocates.
    ring.data = vNew;
    eeprom.resetWriteCount();
    CHECK(mgr.storeRecord(ring) == true);
    relocateWrites = eeprom.writeCount();
  }
  CHECK(relocateWrites > 0);

  for (size_t k = 1; k <= relocateWrites; k++) {
    for (size_t partial : {size_t(0), size_t(8)}) {
      FakeEeprom eeprom(kBigEnoughPages * kPageSize);
      size_t oldBase = 0;
      {
        Ring ring(/*recordId=*/0, eeprom);
        DataMappable *records[] = {&ring};
        TestMgr mgr(eeprom, records, 1, 1);
        ring.setDeadPageOracle(mgr.deadList());
        CHECK(mgr.formatNew() == true);
        ring.data = vOld;
        CHECK(mgr.storeRecord(ring) == true);
        oldBase = ring.getPageNum();
        mgr.deadList().markPageDead(oldBase + 1);
        mgr.deadList().markPageDead(oldBase + 3);

        ring.data = vNew;
        eeprom.resetWriteCount();
        eeprom.armPowerLoss(k, partial);
        mgr.storeRecord(ring); // Interrupted somewhere in here.
      }

      // Reboot: reload the manager and the ring, then load the record.
      Ring ring(/*recordId=*/0, eeprom);
      DataMappable *records[] = {&ring};
      TestMgr mgr(eeprom, records, 1, 1);
      ring.setDeadPageOracle(mgr.deadList());
      CHECK(mgr.init() == TestMgr::ROOT_PAGE_OK);

      // Metadata must always be self-consistent after the interruption.
      CHECK(mgr.deadList().verify());
      CHECK(mgr.usage().verify());
      CHECK(mgr.dataMap().verify());

      // The record must reload SOME valid value -- old or new, never lost.
      CHECK(ring.load() == true);
      CHECK((ring.data == vOld || ring.data == vNew));
    }
  }
}

}
