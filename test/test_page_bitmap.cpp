// (c) Copyright 2026 Aaron Kimball
//
// Tests for EepromPageBitmap and its subclasses (WearLevelDeadList,
// PageUsageBitmap) in src/eeprom-wearlevel.h.

#include "doctest.h"

#include "eeprom-wearlevel.h"
#include "fake_eeprom.h"

namespace {

// bitSize=512 bits -> 64-byte EEPROM, pageSize=8 bytes -> 8 pages exactly:
// a clean, power-of-2, multiple-of-8 case.
constexpr size_t kCleanBitSize = 512;
constexpr size_t kCleanPageSize = 8;

// bitSize chosen so numPages = eepromSizeBytes / hwPageSizeBytes is NOT a
// multiple of 8, to exercise the ceiling-division sizing of _pageTableSize
// and make sure findNextFreePageNum() never walks off the end of _bits.
// 20 pages (160 bytes / 8-byte pages) -> pageTableSize must round up to 3
// bytes (24 page-bits) rather than truncate to 2 (16 page-bits).
constexpr size_t kOddBitSize = 160 * 8;
constexpr size_t kOddPageSize = 8;

} // namespace

TEST_SUITE("WearLevelDeadList") {

TEST_CASE("pages are live by default and can be marked dead independently") {
  FakeEeprom eeprom(4096);
  WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(eeprom, 0);
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
    WearLevelDeadList<kCleanBitSize, kCleanPageSize> deadList(eeprom, 0);
    deadList.formatNew();
    deadList.markPageDead(0);
    deadList.markPageDead(7);
  }

  // A brand new object, same backing EEPROM: simulates a reboot.
  WearLevelDeadList<kCleanBitSize, kCleanPageSize> reloaded(eeprom, 0);
  reloaded.init();

  CHECK(reloaded.isPageDead(0));
  CHECK(reloaded.isPageDead(7));
  for (size_t i = 1; i < 7; i++) {
    CHECK(reloaded.isPageLive(i));
  }
}

}

TEST_SUITE("PageUsageBitmap") {

TEST_CASE("pages are empty by default and markPageInUse/markPageFree round-trip") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, 0);
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
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, 0);
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
  PageUsageBitmap<kCleanBitSize, kCleanPageSize> usage(eeprom, 0);
  usage.formatNew();

  for (size_t i = 0; i < 8; i++) {
    usage.markPageInUse(i);
  }

  CHECK(usage.findNextFreePageNum(0) == ERR_NO_FREE_PAGES);
}

TEST_CASE("findNextFreePageNum skips entirely-full bytes before a free one") {
  FakeEeprom eeprom(4096);
  // 4 pages per byte-span isn't real, but with kCleanPageSize the table is
  // exactly one byte (8 pages); use the odd-size config to get >1 byte of
  // table so we can exercise the byte-skipping ("pageByte == 0xFF") path.
  PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, 0);
  usage.formatNew(); // 20 pages: bytes for pages [0-7], [8-15], [16-19].

  for (size_t i = 0; i < 16; i++) {
    usage.markPageInUse(i); // Fill the first two whole bytes.
  }

  CHECK(usage.findNextFreePageNum(0) == 16);
  CHECK(usage.findNextFreePageNum(16) == 16);
}

TEST_CASE("a page count that is not a multiple of 8 is handled without "
          "corrupting neighboring bits (regression for ceil-division sizing)") {
  FakeEeprom eeprom(4096);
  PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, 0);
  usage.formatNew(); // 20 pages -> needs 3 bytes of table, not 2.

  // The last page (19) lives in the third, partially-used byte. Setting it
  // must not corrupt bits belonging to pages 16-18, and must not require
  // writing past the allocated table.
  usage.markPageInUse(19);
  CHECK(usage.isPageInUse(19));
  CHECK(usage.isPageFree(16));
  CHECK(usage.isPageFree(17));
  CHECK(usage.isPageFree(18));

  // findNextFreePageNum must terminate cleanly (not walk off the end of the
  // 20-page table) when searching from the last byte-aligned chunk.
  for (size_t i = 0; i < 19; i++) {
    usage.markPageInUse(i);
  }
  CHECK(usage.findNextFreePageNum(0) == ERR_NO_FREE_PAGES);
}

TEST_CASE("usage bitmap state persists across a simulated power cycle, "
          "including for an odd (non-multiple-of-8) page count") {
  FakeEeprom eeprom(4096);
  {
    PageUsageBitmap<kOddBitSize, kOddPageSize> usage(eeprom, 0);
    usage.formatNew();
    usage.markPageInUse(0);
    usage.markPageInUse(19); // Exercise the partial trailing byte.
  }

  PageUsageBitmap<kOddBitSize, kOddPageSize> reloaded(eeprom, 0);
  reloaded.init();

  CHECK(reloaded.isPageInUse(0));
  CHECK(reloaded.isPageInUse(19));
  CHECK(reloaded.isPageFree(1));
  CHECK(reloaded.isPageFree(18));
}

}
