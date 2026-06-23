// (c) Copyright 2026 Aaron Kimball
//
// Tests for EepromPageBitmap and its subclasses (WearLevelDeadList,
// PageUsageBitmap) in src/eeprom-wearlevel.h, including the crash-safe
// dual-copy (A/B) persistence and the span helpers.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

namespace {

// 8 pages exactly (128 bytes / 16-byte pages). The page must be larger than
// the seqId + CRC32 overhead (8 bytes) the dual-copy store adds, so the
// smallest practical page here is 16. The 1-byte page table fits in a single
// serialized hardware page per copy.
constexpr size_t kCleanBitSize = 1024;
constexpr size_t kCleanPageSize = 16;
// Copy B is placed one serialized page past copy A (each is a single page).
constexpr size_t kCleanCopyA = 0;
constexpr size_t kCleanCopyB = 1;

// 70 pages (1120 bytes / 16-byte pages): NOT a multiple of 8, to exercise the
// ceiling-division sizing of the page table, and large enough (a 9-byte table,
// payload 8 bytes/page) that each copy spans TWO serialized hardware pages.
constexpr size_t kOddBitSize = 70 * 16 * 8;
constexpr size_t kOddPageSize = 16;
constexpr size_t kOddCopyA = 0; // serialized pages 0,1
constexpr size_t kOddCopyB = 2; // serialized pages 2,3

} // namespace

TEST_SUITE("WearLevelDeadList") {

TEST_CASE("pages are live by default and can be marked dead independently") {
  FakeEeprom eeprom(4096);
  WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(eeprom, kCleanCopyA,
                                                           kCleanCopyB);
  deadList.formatNew();

  for (size_t i = 0; i < 8; i++) {
    CHECK(deadList.isPageLive(i));
    CHECK_FALSE(deadList.isPageDead(i));
  }

  deadList.markPageDead(3);
  CHECK(deadList.isPageDead(3));
  CHECK_FALSE(deadList.isPageLive(3));

  // Neighbors are unaffected.
  CHECK(deadList.isPageLive(2));
  CHECK(deadList.isPageLive(4));
}

TEST_CASE("marking dead pages persists across a simulated power cycle") {
  FakeEeprom eeprom(4096);
  {
    WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(
        eeprom, kCleanCopyA, kCleanCopyB);
    deadList.formatNew();
    deadList.markPageDead(0);
    deadList.markPageDead(7);
  }

  // A brand new object, same backing EEPROM: simulates a reboot.
  WearLevelDeadList<kCleanBitSize, kCleanPageSize> reloaded(eeprom, kCleanCopyA,
                                                           kCleanCopyB);
  CHECK(reloaded.init());

  CHECK(reloaded.isPageDead(0));
  CHECK(reloaded.isPageDead(7));
  for (size_t i = 1; i < 7; i++) {
    CHECK(reloaded.isPageLive(i));
  }
}

TEST_CASE("init() on a never-formatted (all-0xFF) region reports invalid") {
  FakeEeprom eeprom(4096);
  WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(eeprom, kCleanCopyA,
                                                           kCleanCopyB);
  CHECK_FALSE(deadList.init());
}

TEST_CASE("markSpanDead marks a contiguous run with a single flush") {
  FakeEeprom eeprom(4096);
  WearLevelDeadList<kOddBitSize, kOddPageSize> deadList(eeprom, kOddCopyA,
                                                       kOddCopyB);
  deadList.formatNew();

  deadList.markSpanDead(10, 5); // pages 10..14
  for (size_t i = 10; i < 15; i++) {
    CHECK(deadList.isPageDead(i));
  }
  CHECK(deadList.isPageLive(9));
  CHECK(deadList.isPageLive(15));

  WearLevelDeadList<kOddBitSize, kOddPageSize> reloaded(eeprom, kOddCopyA,
                                                       kOddCopyB);
  CHECK(reloaded.init());
  for (size_t i = 10; i < 15; i++) {
    CHECK(reloaded.isPageDead(i));
  }
}

TEST_CASE("a torn flush (power loss mid-write) leaves the previous committed "
          "state intact thanks to the redundant A/B copies") {
  FakeEeprom eeprom(4096);
  {
    WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(
        eeprom, kCleanCopyA, kCleanCopyB);
    deadList.formatNew();
    deadList.markPageDead(3); // Committed cleanly.

    // Now lose power partway through the next flush (a torn page program).
    eeprom.resetWriteCount();
    eeprom.armPowerLoss(/*atWrite=*/1, /*partialBytes=*/4);
    deadList.markPageDead(5); // This write tears; nothing more commits.
  }

  WearLevelDeadList<kCleanBitSize, kCleanPageSize> reloaded(eeprom, kCleanCopyA,
                                                           kCleanCopyB);
  CHECK(reloaded.init());
  CHECK(reloaded.isPageDead(3));  // The earlier, committed change survived.
  CHECK(reloaded.isPageLive(5));  // The torn change did not corrupt anything.
}

}

TEST_SUITE("PageUsageBitmap") {

TEST_CASE("pages are empty by default and markPageInUse/markPageFree round-trip") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, kCleanCopyA,
                                                      kCleanCopyB);
  usage.formatNew();

  CHECK(usage.isPageFree(5));
  usage.markPageInUse(5);
  CHECK(usage.isPageInUse(5));
  CHECK_FALSE(usage.isPageFree(5));

  usage.markPageFree(5);
  CHECK(usage.isPageFree(5));
  CHECK_FALSE(usage.isPageInUse(5));
}

TEST_CASE("findNextFreePageNum finds the first free bit at or after the start") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, kCleanCopyA,
                                                      kCleanCopyB);
  usage.formatNew(); // 8 pages, all free.

  CHECK(usage.findNextFreePageNum(0) == 0);

  usage.markPageInUse(0);
  usage.markPageInUse(1);
  usage.markPageInUse(2);
  CHECK(usage.findNextFreePageNum(0) == 3);

  // Asking to start mid-byte, at an already-occupied page, skips forward.
  CHECK(usage.findNextFreePageNum(1) == 3);
  // Asking to start exactly at a free page returns that page.
  CHECK(usage.findNextFreePageNum(3) == 3);
}

TEST_CASE("findNextFreePageNum returns ERR_NO_FREE_PAGES when the table is full") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, kCleanCopyA,
                                                      kCleanCopyB);
  usage.formatNew();

  usage.markSpanInUse(0, 8);
  CHECK(usage.findNextFreePageNum(0) == ERR_NO_FREE_PAGES);
}

TEST_CASE("findNextFreePageNum skips entirely-full bytes before a free one") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, kOddCopyA, kOddCopyB);
  usage.formatNew(); // 70 pages.

  usage.markSpanInUse(0, 64); // Fill the first eight whole bytes.
  CHECK(usage.findNextFreePageNum(0) == 64);
  CHECK(usage.findNextFreePageNum(64) == 64);
}

TEST_CASE("a page count that is not a multiple of 8 is handled without "
          "corrupting neighboring bits (regression for ceil-division sizing)") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, kOddCopyA, kOddCopyB);
  usage.formatNew(); // 70 pages -> 9-byte table; pages 64..69 share a byte.

  // The last page (69) lives in the ninth, partially-used byte. Setting it
  // must not corrupt bits belonging to pages 64-68.
  usage.markPageInUse(69);
  CHECK(usage.isPageInUse(69));
  for (size_t i = 64; i < 69; i++) {
    CHECK(usage.isPageFree(i));
  }

  // findNextFreePageNum must terminate cleanly (not walk off the end of the
  // 70-page table) when searching from the last byte-aligned chunk.
  usage.markSpanInUse(0, 69);
  CHECK(usage.findNextFreePageNum(0) == ERR_NO_FREE_PAGES);
}

TEST_CASE("usage bitmap state persists across a power cycle, including for an "
          "odd page count whose table spans multiple serialized pages") {
  FakeEeprom eeprom(4096);
  {
    PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, kOddCopyA,
                                                    kOddCopyB);
    usage.formatNew();
    usage.markPageInUse(0);
    usage.markPageInUse(69); // Exercise the partial trailing byte (2nd page).
  }

  PageUsageBitmap<kOddBitSize, kOddPageSize> reloaded(eeprom, kOddCopyA,
                                                     kOddCopyB);
  CHECK(reloaded.init());

  CHECK(reloaded.isPageInUse(0));
  CHECK(reloaded.isPageInUse(69));
  CHECK(reloaded.isPageFree(1));
  CHECK(reloaded.isPageFree(68));
}

TEST_CASE("markSpanInUse with delayFlush stages without writing until flush()") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, kCleanCopyA,
                                                      kCleanCopyB);
  usage.formatNew();

  usage.markSpanInUse(0, 4, /*delayFlush=*/true);
  // In-RAM view reflects the change immediately.
  CHECK(usage.isPageInUse(0));
  CHECK(usage.isPageInUse(3));

  // But a reload before flush() sees nothing (the staged bits weren't written).
  {
    PageUsageBitmap<kCleanBitSize, kCleanPageSize> peek(eeprom, kCleanCopyA,
                                                       kCleanCopyB);
    CHECK(peek.init());
    CHECK(peek.isPageFree(0));
    CHECK(peek.isPageFree(3));
  }

  CHECK(usage.flush());
  {
    PageUsageBitmap<kCleanBitSize, kCleanPageSize> peek(eeprom, kCleanCopyA,
                                                       kCleanCopyB);
    CHECK(peek.init());
    CHECK(peek.isPageInUse(0));
    CHECK(peek.isPageInUse(3));
  }
}

}
