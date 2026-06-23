// (c) Copyright 2026 Aaron Kimball
//
// Tests for computeCrc32() in src/eeprom-wearlevel.h.

#include "doctest.h"

#include "eeprom-wearlevel.h"

#include <cstring>

TEST_SUITE("computeCrc32") {

TEST_CASE("matches the standard CRC32 (zlib/PNG) test vectors") {
  CHECK(computeCrc32(reinterpret_cast<const uint8_t *>(""), 0) == 0x00000000u);

  const char *a = "a";
  CHECK(computeCrc32(reinterpret_cast<const uint8_t *>(a), 1) == 0xE8B7BE43u);

  const char *check = "123456789";
  CHECK(computeCrc32(reinterpret_cast<const uint8_t *>(check), 9) == 0xCBF43926u);

  const char *quick = "The quick brown fox jumps over the lazy dog";
  CHECK(computeCrc32(reinterpret_cast<const uint8_t *>(quick), strlen(quick)) ==
        0x414FA339u);
}

TEST_CASE("is deterministic and sensitive to every byte") {
  uint8_t buf[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  uint32_t crc1 = computeCrc32(buf, sizeof(buf));
  uint32_t crc2 = computeCrc32(buf, sizeof(buf));
  CHECK(crc1 == crc2);

  for (size_t i = 0; i < sizeof(buf); i++) {
    uint8_t flipped[8];
    memcpy(flipped, buf, sizeof(buf));
    flipped[i] ^= 0x01; // Flip one bit of one byte.
    CHECK(computeCrc32(flipped, sizeof(flipped)) != crc1);
  }
}

TEST_CASE("distinguishes data from its own CRC32 swapped in place") {
  // Sanity check against a classic off-by-one: appending a buffer's own CRC
  // and recomputing over a different length must not coincidentally match.
  uint8_t buf[4] = {0xDE, 0xAD, 0xBE, 0xEF};
  uint32_t crc = computeCrc32(buf, sizeof(buf));

  uint8_t withCrc[8];
  memcpy(withCrc, buf, sizeof(buf));
  memcpy(withCrc + sizeof(buf), &crc, sizeof(crc));

  CHECK(computeCrc32(withCrc, sizeof(withCrc)) != crc);
}

}
