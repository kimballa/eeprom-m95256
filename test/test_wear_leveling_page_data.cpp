// (c) Copyright 2026 Aaron Kimball
//
// Tests for WearLevelingPageData<T, hwPageSizeBytes, k> in
// src/eeprom-wearlevel.h: round-robin slot rotation, writeSequenceId-based
// "most recent slot" discovery on load() (including the UINT32_MAX
// wraparound disambiguation), per-slot dead-marking/skipping on a failed
// write, giving up on a half-dead ring, and that giving-up integrates with
// EepromPageManager::storeRecord()'s generic relocate-on-failure behavior.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

#include <cstring>

namespace {

constexpr size_t kPageSize = 32;
constexpr size_t kRingSize = 4; // Small k so tests can drive a full wrap quickly.

struct Widget {
  uint32_t id;
  int32_t value;

  bool operator==(const Widget &o) const { return id == o.id && value == o.value; }
};

using Ring = WearLevelingPageData<Widget, kPageSize, kRingSize>;
using DeadList = WearLevelDeadList<64 * kPageSize * 8, kPageSize>;
// A 64-page deadlist needs an 8-byte table, which fits one serialized page per
// copy (payload kPageSize-8 == 24 bytes); place copy B one page past copy A.
constexpr size_t kDeadListCopyA = 0;
constexpr size_t kDeadListCopyB = 1;

} // namespace

TEST_SUITE("WearLevelingPageData") {

TEST_CASE("sizeInPages()/getRecordId() satisfy the DataMappable contract, and NUM_SLOTS "
          "matches k") {
  FakeEeprom eeprom(4096);
  Ring ring(/*recordId=*/3, eeprom);
  CHECK(ring.sizeInPages() == kRingSize);
  CHECK(ring.getRecordId() == 3);
  CHECK(Ring::NUM_SLOTS == kRingSize);
}

TEST_CASE("a freshly placed (never-written) ring has no valid record") {
  FakeEeprom eeprom(4096);
  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);

  CHECK(ring.load() == false);
  CHECK(ring.currentWriteSeqId() == 0);
}

TEST_CASE("the first store() lands at slot 0 with writeSequenceId 1") {
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  ring.data = Widget{1, 2};
  CHECK(ring.store() == true);
  CHECK(ring.currentSlot() == 0);
  CHECK(ring.currentWriteSeqId() == 1);
  CHECK(ring.currentPageNum() == 4);
}

TEST_CASE("store() advances round-robin through all k slots in order, with "
          "writeSequenceId incrementing each time, then wraps back to slot 0") {
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  for (size_t i = 0; i < kRingSize * 2; i++) {
    ring.data = Widget{static_cast<uint32_t>(i), 0};
    CHECK(ring.store() == true);
    CHECK(ring.currentSlot() == i % kRingSize);
    CHECK(ring.currentWriteSeqId() == i + 1);
  }
}

TEST_CASE("setPageNum() resets the ring's slot/sequence history") {
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);
  ring.data = Widget{1, 1};
  ring.store();
  ring.store();
  CHECK(ring.currentWriteSeqId() == 2);

  ring.setPageNum(8); // Simulate a relocation to a fresh ring elsewhere.
  CHECK(ring.currentWriteSeqId() == 0);
  CHECK(ring.currentSlot() == 0);

  ring.data = Widget{9, 9};
  CHECK(ring.store() == true);
  CHECK(ring.currentSlot() == 0);
  CHECK(ring.currentWriteSeqId() == 1); // The new ring starts its own history at 1.
}

TEST_CASE("load() after a reboot finds the most-recently-written slot, even with "
          "several other CRC-valid (but older) slots present in the ring") {
  FakeEeprom eeprom(4096);
  Widget last{};
  {
    DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
    deadList.formatNew();

    Ring writer(/*recordId=*/0, eeprom);
    writer.setPageNum(4);
    writer.setDeadPageOracle(deadList);
    // k+2 stores: every slot has a valid CRC, and the ring has wrapped once.
    for (size_t i = 0; i < kRingSize + 2; i++) {
      writer.data = Widget{static_cast<uint32_t>(i), static_cast<int32_t>(i * 10)};
      CHECK(writer.store() == true);
    }
    last = writer.data;
  }

  Ring reader(/*recordId=*/0, eeprom);
  reader.setPageNum(4);
  CHECK(reader.load() == true);
  CHECK(reader.data == last);
  CHECK(reader.currentWriteSeqId() == kRingSize + 2);
}

TEST_CASE("load() correctly disambiguates a writeSequenceId that has wrapped from "
          "UINT32_MAX back around to small values") {
  FakeEeprom eeprom(4096);
  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);

  // Slot 0 holds the literal pre-wrap maximum; slots 1..3 still hold older,
  // merely-large pre-wrap values (as a real ring would, since each slot is
  // exactly `k` writes apart). None of this requires k actual writes to set
  // up -- craft it directly, since driving 4 billion real store() calls
  // isn't practical in a test.
  auto writeRaw = [&](size_t slot, uint32_t seq, const Widget &w) {
    uint8_t buf[sizeof(uint32_t) + sizeof(Widget) + sizeof(uint32_t)];
    memcpy(buf, &seq, sizeof(seq));
    memcpy(buf + sizeof(seq), &w, sizeof(w));
    uint32_t crc = computeCrc32(buf, sizeof(seq) + sizeof(w));
    memcpy(buf + sizeof(seq) + sizeof(w), &crc, sizeof(crc));
    eeprom.write(buf, static_cast<FakeEeprom::addr_t>((4 + slot) * kPageSize), sizeof(buf));
  };

  writeRaw(0, UINT32_MAX, Widget{100, 100});       // The exact pre-wrap max.
  writeRaw(1, UINT32_MAX - 1, Widget{99, 99});      // Still pre-wrap.
  writeRaw(2, UINT32_MAX - 2, Widget{98, 98});      // Still pre-wrap.
  writeRaw(3, 1, Widget{1, 1});                     // Just wrapped: the real winner.

  CHECK(ring.load() == true);
  CHECK(ring.data == (Widget{1, 1}));
  CHECK(ring.currentSlot() == 3);
  CHECK(ring.currentWriteSeqId() == 1);
}

TEST_CASE("load() disambiguates a wrap even when the literal UINT32_MAX slot has "
          "already been overwritten, leaving only near-max pre-wrap values") {
  // When dead-slot skipping means slots aren't overwritten in strict ring
  // order, the slot that held the exact UINT32_MAX can be recycled before the
  // other pre-wrap slots. The wrap heuristic keys off the whole top-k band
  // (any seq within k of UINT32_MAX), not just the literal max, so it still
  // recognizes this as a wrap and picks the small post-wrap winner.
  FakeEeprom eeprom(4096);
  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);

  auto writeRaw = [&](size_t slot, uint32_t seq, const Widget &w) {
    uint8_t buf[sizeof(uint32_t) + sizeof(Widget) + sizeof(uint32_t)];
    memcpy(buf, &seq, sizeof(seq));
    memcpy(buf + sizeof(seq), &w, sizeof(w));
    uint32_t crc = computeCrc32(buf, sizeof(seq) + sizeof(w));
    memcpy(buf + sizeof(seq) + sizeof(w), &crc, sizeof(crc));
    eeprom.write(buf, static_cast<FakeEeprom::addr_t>((4 + slot) * kPageSize),
                 sizeof(buf));
  };

  // No slot holds the literal UINT32_MAX, but three hold near-max pre-wrap
  // values; slot 1 holds the post-wrap winner (seq 2).
  writeRaw(0, UINT32_MAX - 1, Widget{91, 91});
  writeRaw(1, 2, Widget{2, 2}); // The real winner.
  writeRaw(2, UINT32_MAX - 2, Widget{92, 92});
  writeRaw(3, UINT32_MAX - 3, Widget{93, 93});

  CHECK(ring.load() == true);
  CHECK(ring.data == (Widget{2, 2}));
  CHECK(ring.currentSlot() == 1);
  CHECK(ring.currentWriteSeqId() == 2);
}

TEST_CASE("after a reboot, load() correctly finds the latest slot when only some of "
          "the ring has ever been written (a never-written slot reads back as a "
          "factory-reset, all-0xFF page, just like real EEPROM hardware)") {
  FakeEeprom eeprom(4096);

  {
    DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
    deadList.formatNew();

    Ring writer(/*recordId=*/0, eeprom);
    writer.setPageNum(4);
    writer.setDeadPageOracle(deadList);

    // Only write 2 of the ring's 4 slots; slots 2 and 3 stay untouched, so
    // they still read back as FakeEeprom's factory-reset 0xFF fill --
    // writeSequenceId==UINT32_MAX, CRC32 trailer==0xFFFFFFFF too. That CRC
    // won't validate against an all-0xFF payload, so those slots are
    // correctly excluded as candidates despite their raw writeSequenceId
    // looking huge; nothing here should trip the wraparound heuristic.
    CHECK(eeprom.byteAt(static_cast<size_t>((4 + 2) * kPageSize)) == 0xFF);

    writer.data = Widget{5, 5};
    CHECK(writer.store() == true); // slot 0, seq 1
    writer.data = Widget{6, 6};
    CHECK(writer.store() == true); // slot 1, seq 2
  } // writer goes out of scope: simulates a power cycle.

  Ring reader(/*recordId=*/0, eeprom);
  reader.setPageNum(4);
  CHECK(reader.load() == true);
  CHECK(reader.data == (Widget{6, 6}));
  CHECK(reader.currentSlot() == 1);
  CHECK(reader.currentWriteSeqId() == 2);
}

TEST_CASE("a write that fails its readback verification is marked dead, and the "
          "ring transparently retries the next live slot") {
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  ring.data = Widget{1, 1};
  CHECK(ring.store() == true); // slot 0, seq 1.
  CHECK(ring.currentSlot() == 0);

  // Make the page backing slot 1 unreliable.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>((4 + 1) * kPageSize));

  ring.data = Widget{2, 2};
  CHECK(ring.store() == true); // Skips dead slot 1, lands on slot 2.
  CHECK(ring.currentSlot() == 2);
  CHECK(ring.currentWriteSeqId() == 2);
  CHECK(deadList.isPageDead(4 + 1) == true);
  CHECK(deadList.isPageDead(4 + 0) == false);
  CHECK(deadList.isPageDead(4 + 2) == false);

  // Slot 1 stays excluded from the rotation forever afterward.
  for (int i = 0; i < 6; i++) {
    ring.data = Widget{static_cast<uint32_t>(10 + i), 0};
    CHECK(ring.store() == true);
    CHECK(ring.currentSlot() != 1);
  }
  CHECK(deadList.isPageDead(4 + 1) == true);
}

TEST_CASE("store() gives up without writing anywhere once half or more of the "
          "ring's slots are already dead") {
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();
  deadList.markPageDead(4 + 0);
  deadList.markPageDead(4 + 2); // 2 of 4 slots dead == half.

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  ring.data = Widget{1, 1};
  CHECK(ring.store() == false);
  CHECK(ring.currentWriteSeqId() == 0); // Nothing was ever written.
}

TEST_CASE("EepromPageManager::storeRecord() relocates a WearLevelingPageData ring "
          "wholesale once enough of its slots have failed verification") {
  constexpr size_t kNumPages = 16;
  FakeEeprom eeprom(kNumPages * kPageSize);
  using Mgr = EepromPageManager<kNumPages * kPageSize * 8, kPageSize>;

  Ring ring(/*recordId=*/0, eeprom);
  DataMappable *records[] = {&ring};
  Mgr mgr(eeprom, records, 1, /*appFormatVersion=*/1);
  ring.setDeadPageOracle(mgr.deadList());

  CHECK(mgr.formatNew() == true);
  size_t originalBase = ring.getPageNum();
  CHECK(ring.sizeInPages() == kRingSize);

  ring.data = Widget{1, 1};
  CHECK(mgr.storeRecord(ring) == true); // slot 0, seq 1.
  CHECK(ring.currentSlot() == 0);

  // Kill slot 1: the ring quietly retries and lands on slot 2.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>((originalBase + 1) * kPageSize));
  ring.data = Widget{2, 2};
  CHECK(mgr.storeRecord(ring) == true);
  CHECK(ring.currentSlot() == 2);
  CHECK(ring.getPageNum() == originalBase); // Still the same ring: not relocated yet.

  // Now kill slot 3 too: that's half the ring (2 of 4) dead, so the ring's
  // own store() gives up, and EepromPageManager::storeRecord() must
  // relocate the whole thing elsewhere.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>((originalBase + 3) * kPageSize));
  ring.data = Widget{3, 3};
  CHECK(mgr.storeRecord(ring) == true);

  size_t newBase = ring.getPageNum();
  CHECK(newBase != originalBase);
  CHECK(ring.currentSlot() == 0);     // Fresh ring: history starts over.
  CHECK(ring.currentWriteSeqId() == 1);

  // All 4 pages of the abandoned ring are dead -- including the 2 that were
  // never individually marked dead, since the whole region was given up on.
  for (size_t j = 0; j < kRingSize; j++) {
    CHECK(mgr.deadList().isPageDead(originalBase + j) == true);
  }
  for (size_t j = 0; j < kRingSize; j++) {
    CHECK(mgr.usage().isPageInUse(newBase + j) == true);
  }
  CHECK(mgr.dataMap().getStartPage(ring.getRecordId()) == newBase);

  // The data really did survive the relocation.
  Ring reloaded(/*recordId=*/0, eeprom);
  reloaded.setPageNum(static_cast<Ring::addr_t>(newBase));
  CHECK(reloaded.load() == true);
  CHECK(reloaded.data == (Widget{3, 3}));
}

TEST_CASE("storeAsync() writes exactly one slot and does not yet commit "
          "currentSlot/currentWriteSeqId") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);
  eeprom.setWipPollsPerWrite(2); // Keep the write "in flight" across polls.

  size_t writesBefore = eeprom.writeCount();
  ring.data = Widget{1, 2};
  uint8_t buf[kPageSize];
  ring.storeAsync(buf);

  // Exactly one physical page write was issued, and nothing is committed yet.
  CHECK(eeprom.writeCount() == writesBefore + 1);
  CHECK(ring.currentWriteSeqId() == 0);

  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::IN_PROGRESS);
  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::IN_PROGRESS);
  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);

  // Now committed at slot 0, seq 1, and a reboot can find it.
  CHECK(ring.currentSlot() == 0);
  CHECK(ring.currentWriteSeqId() == 1);

  Ring reader(/*recordId=*/0, eeprom);
  reader.setPageNum(4);
  CHECK(reader.load() == true);
  CHECK(reader.data == (Widget{1, 2}));
}

TEST_CASE("async stores advance round-robin across slots, exactly like store()") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  uint8_t buf[kPageSize];
  for (size_t i = 0; i < kRingSize * 2; i++) {
    ring.data = Widget{static_cast<uint32_t>(i), 0};
    ring.storeAsync(buf);
    CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);
    CHECK(ring.currentSlot() == i % kRingSize);
    CHECK(ring.currentWriteSeqId() == i + 1);
  }
}

TEST_CASE("verifyAsync() recovery: a bad async slot is marked dead and the next "
          "live slot synchronously takes the write") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  uint8_t buf[kPageSize];
  ring.data = Widget{1, 1};
  ring.storeAsync(buf); // slot 0
  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::SUCCESS);
  CHECK(ring.currentSlot() == 0);

  // Slot 1's page is unreliable; the async write to it won't verify.
  eeprom.setFaultyByte(static_cast<FakeEeprom::addr_t>((4 + 1) * kPageSize));
  ring.data = Widget{2, 2};
  ring.storeAsync(buf); // targets slot 1
  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::SUCCESS); // recovers to slot 2

  CHECK(ring.currentSlot() == 2);
  CHECK(ring.currentWriteSeqId() == 2);
  CHECK(deadList.isPageDead(4 + 1) == true);

  Ring reader(/*recordId=*/0, eeprom);
  reader.setPageNum(4);
  CHECK(reader.load() == true);
  CHECK(reader.data == (Widget{2, 2}));
}

TEST_CASE("storeAsync() on a worn-out ring issues no write and verifyAsync() "
          "reports FAILED") {
  isEepromWriteStalled = false;
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();
  deadList.markPageDead(4 + 0);
  deadList.markPageDead(4 + 2); // half the ring dead.

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  uint8_t buf[kPageSize];
  size_t writesBefore = eeprom.writeCount();
  ring.data = Widget{1, 1};
  ring.storeAsync(buf);
  CHECK(eeprom.writeCount() == writesBefore); // Nothing was written.
  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::FAILED);
  CHECK(ring.currentWriteSeqId() == 0);
}

TEST_CASE("verifyAsync() raises isEepromWriteStalled and FAILS on a stuck WIP "
          "bit during an async slot write") {
  isEepromWriteStalled = false;
  resetFakeMillis();
  FakeEeprom eeprom(4096);
  DeadList deadList(eeprom, kDeadListCopyA, kDeadListCopyB);
  deadList.formatNew();

  Ring ring(/*recordId=*/0, eeprom);
  ring.setPageNum(4);
  ring.setDeadPageOracle(deadList);

  uint8_t buf[kPageSize];
  ring.data = Widget{1, 1};
  ring.storeAsync(buf);
  eeprom.setWipStuck(true);
  advanceFakeMillis(WRITE_STALL_THRESHOLD_MILLIS + 1);

  CHECK(ring.verifyAsync(buf) == AsyncStoreStatus::FAILED);
  CHECK(isEepromWriteStalled == true);
}

}
