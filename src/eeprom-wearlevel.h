// (c) Copyright 2026 Aaron Kimball
//
// Classes for managing wear level-sensitive writes to an EEPROM
// to maximize its lifetime.

#ifndef _EEPROM_WEAR_LEVEL_H
#define _EEPROM_WEAR_LEVEL_H

#include "eeprom-m95256.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

/**
 * A bitmap where each bit in the bitmap corresponds to one page of the EEPROM.
 * The values of 0 and 1 are application-specific to the client of the bitmap.
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes> class EepromPageBitmap {
public:
  using addr_t = uint16_t;

  /** Create a bitmap object, stored in the specified EEPROM at address
   * `addr`.
   */
  EepromPageBitmap(GenericSpiEeprom &eeprom, addr_t addr)
      : _eeprom(eeprom), _addr(addr){};

  /** Set up the bitmap by loading it from EEPROM. */
  void init() { _eeprom.read(_bits, _addr, _pageTableSize); };

  /** Set up the bitmap for a new EEPROM by zeroing the whole table.  */
  void formatNew() { memset(_bits, 0, _pageTableSize); };

protected:
  /** Return true if the page identified by pageNum is marked with a 1 in the
   * bitmap. */
  bool isPageSet(size_t pageNum) const {
    size_t pageByte = pageNum / 8;
    uint8_t pageBitFlag = 1 << (pageNum & 7);
    uint8_t isSet = _bits[pageByte] & pageBitFlag;
    return isSet != 0;
  }

  /** Mark the specified page with a 1. */
  void setPageBit(size_t pageNum) {
    size_t pageByte = pageNum / 8;
    uint8_t pageBitFlag = 1 << (pageNum & 7);
    _bits[pageByte] |= pageBitFlag;
    flush();
  };

  /** Mark the specified page with a 0. */
  void clearPageBit(size_t pageNum) {
    size_t pageByte = pageNum / 8;
    uint8_t pageBitFlag = 1 << (pageNum & 7);
    _bits[pageByte] &= ~pageBitFlag;
    flush();
  };

  /** Return the entire byte (8 page span) of the bitmap containing the bit for
   * pageNum. */
  uint8_t getPageByte(size_t pageNum) {
    size_t pageByte = pageNum / 8;
    return _bits[pageByte];
  }

  static constexpr size_t _eepromSizeBytes = bitSize / 8;
  static constexpr size_t _numPages = _eepromSizeBytes / hwPageSizeBytes;
  static constexpr size_t _pageTableSize = (_numPages + 7) / 8;

private:
  /** Write the bitmap back to the EEPROM. */
  void flush() { _eeprom.write(_bits, _addr, _pageTableSize); };

  /** The EEPROM we are managing (and stored on). */
  GenericSpiEeprom &_eeprom;

  /** The address in the EEPROM where this bitmap can be found. */
  const addr_t _addr;

  /** The actual data bitmap. */
  uint8_t _bits[_pageTableSize];
};

/**
 * A bitmap indicating which pages are *dead*.
 *
 * Each bit in this array represents one page in the EEPROM. A zero in bit
 * position `i` means the i'th page is valid. A one in the position means the
 * page is marked as 'dead' and the page allocator should not direct any data to
 * be stored there.
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes>
class WearLevelDeadList : public EepromPageBitmap<bitSize, hwPageSizeBytes> {
  using Base = EepromPageBitmap<bitSize, hwPageSizeBytes>;

public:
  using typename Base::addr_t;

  /** Create a deadlist object, stored in the specified EEPROM at address
   * `deadListAddr`.
   */
  WearLevelDeadList(GenericSpiEeprom &eeprom, addr_t deadListAddr)
      : Base(eeprom, deadListAddr){};

  /** Return true if the page identified by pageNum is marked dead and should
   * not be used for storage. */
  bool isPageDead(size_t pageNum) const { return Base::isPageSet(pageNum); }

  bool isPageLive(size_t pageNum) const { return !Base::isPageSet(pageNum); };

  /** Mark the specified page as no longer accessible. */
  void markPageDead(size_t pageNum) { Base::setPageBit(pageNum); };
};

constexpr size_t ERR_NO_FREE_PAGES = SIZE_MAX;

/**
 * A bitmap indicating which pages are in use for data.
 *
 * Each bit in this array represents one page in the EEPROM. A zero in bit
 * position `i` means the i'th page is empty. A one in the position means the
 * page is in use for data already and the page allocator cannot direct a new
 * data record to be stored there.
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes>
class PageUsageBitmap : public EepromPageBitmap<bitSize, hwPageSizeBytes> {
  using Base = EepromPageBitmap<bitSize, hwPageSizeBytes>;

public:
  using typename Base::addr_t;

  /** Create a page usage object, stored in the specified EEPROM at address
   * `pageUsageAddr`.
   */
  PageUsageBitmap(GenericSpiEeprom &eeprom, addr_t pageUsageAddr)
      : Base(eeprom, pageUsageAddr){};

  /** Return true if the page identified by pageNum is marked in-use and should
   * not be used for storage of a new object. */
  bool isPageInUse(size_t pageNum) const { return Base::isPageSet(pageNum); }

  bool isPageFree(size_t pageNum) const { return !Base::isPageSet(pageNum); };

  /** Mark the specified page as allocated for storing a record. */
  void markPageInUse(size_t pageNum) { Base::setPageBit(pageNum); };

  /** Mark the specified page as free; no longer storing a record. */
  void markPageFree(size_t pageNum) { Base::clearPageBit(pageNum); };

  /**
   * Return the page number of the first free page with pageNum >= startPageNum.
   *
   * Returns ERR_NO_FREE_PAGES if this condition cannot be satisfied.
   */
  size_t findNextFreePageNum(size_t startPageNum) {
    for (size_t i = (startPageNum / 8) * 8; i < Base::_numPages; i += 8) {
      // Get the byte for the 8-page-wide region of the bitmap containing
      // startPageNum.
      uint8_t pageByte = Base::getPageByte(i);

      if (pageByte == 0xFF) {
        // All 8 pages are full. Keep moving.
        continue;
      } else {
        // There is at least one free page in this region.
        // Look at each bit in the byte. If any bit is zero, that's our first
        // free page.

        // We started our iterator 'i' at the 8-aligned value at or just below
        // startPageNum so that it would always be aligned to the start of the
        // retrieved byte. Thus, 'i + j' is the page number we are actually
        // representing, and for the first byte read, it may be lower than
        // startPageNum. In that case, skip it.
        for (size_t j = 0; j < 8; j++) {
          if (i + j >= startPageNum && i + j < Base::_numPages &&
              (pageByte & (1 << j)) == 0) {
            return i + j; // Found a free page bit.
          }
        }
      }
    }

    // Couldn't find a free page bit before exhausting the array.
    return ERR_NO_FREE_PAGES;
  }
};

/**
 * Compute the standard (zlib/PNG, polynomial 0xEDB88320) CRC32 checksum over
 * `len` bytes starting at `data`.
 */
inline uint32_t computeCrc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
  }
  return ~crc;
}

/**
 * Interface for a record whose location can be tracked by a `DataMap`. The
 * DataMap persists the hardware page at which each record's data begins; it
 * does not persist the record's size.
 */
class DataMappable {
public:
  using addr_t = uint16_t;

  /** The number of complete hardware pages this record occupies. */
  virtual size_t sizeInPages() const = 0;

  /**
   * This record's unique id, in the range 0..n-1 for the n records tracked by
   * its DataMap. This also serves as the record's index within the DataMap's
   * backing array. The caller is responsible for ensuring ids are unique.
   */
  virtual addr_t getRecordId() const = 0;
};

/**
 * Tracks, for each of n `DataMappable` records, the hardware page at which
 * that record's data currently begins.
 *
 * This is persisted as an array of n page numbers (2 bytes each), indexed by
 * record id, to the EEPROM starting at a fixed page. The array is serialized
 * across as many hardware pages as required: each page holds up to
 * `(hwPageSizeBytes - 4)` bytes of array data immediately followed by a CRC32
 * of that page's data. The final page may hold fewer array bytes, but is
 * still immediately followed by its own CRC32. The size of each record
 * (`sizeInPages()`) is not itself persisted; it is purely an in-program
 * convenience available to the records' allocator.
 */
template <size_t hwPageSizeBytes> class DataMap {
public:
  using addr_t = uint16_t;

  /**
   * Create a DataMap tracking `numRecords` records, to be serialized to
   * `eeprom` starting at hardware page `mapStartPageNum`.
   */
  DataMap(GenericSpiEeprom &eeprom, addr_t mapStartPageNum,
          const DataMappable *const *records, size_t numRecords)
      : _eeprom(eeprom), _mapStartPageNum(mapStartPageNum), _records(records),
        _numRecords(numRecords),
        _numMapPages(((_numRecords * sizeof(addr_t)) + _bytesPerMapPage - 1) /
                     _bytesPerMapPage),
        _startPages(new addr_t[numRecords]) {
    memset(_startPages, 0, _numRecords * sizeof(addr_t));
  };

  ~DataMap() { delete[] _startPages; };

  /**
   * Set up the DataMap by loading the serialized start-page array from the
   * EEPROM, verifying each page's CRC32 along the way.
   *
   * Returns false (and still loads as much as it can) if any page's CRC32
   * does not match its stored data, so the caller can decide how to react to
   * detected corruption instead of silently trusting it.
   */
  bool init() {
    size_t remaining = _numRecords * sizeof(addr_t);
    uint8_t *dst = reinterpret_cast<uint8_t *>(_startPages);
    bool allValid = true;

    for (size_t mapPage = 0; mapPage < _numMapPages; mapPage++) {
      size_t thisPageBytes = min(remaining, _bytesPerMapPage);
      uint8_t buf[hwPageSizeBytes];
      _eeprom.read(buf, _pageAddr(mapPage), thisPageBytes + sizeof(uint32_t));

      uint32_t storedCrc;
      memcpy(&storedCrc, buf + thisPageBytes, sizeof(uint32_t));
      if (computeCrc32(buf, thisPageBytes) != storedCrc) {
        Serial.println("ERROR: DataMap page CRC32 mismatch.");
        allValid = false;
      }

      memcpy(dst, buf, thisPageBytes);
      dst += thisPageBytes;
      remaining -= thisPageBytes;
    }

    return allValid;
  };

  /** Return the page number at which the record identified by `recordId`
   * currently starts, as stored in the backing array. */
  addr_t getStartPage(addr_t recordId) const { return _startPages[recordId]; }

  /** Set, in memory only, the start page for the record identified by
   * `recordId`. Call flush() to persist this change to the EEPROM. */
  void setStartPage(addr_t recordId, addr_t startPage) {
    _startPages[recordId] = startPage;
  };

  /** Write the entire start-page array back to the EEPROM, one hardware page
   * at a time, each followed by a freshly computed CRC32. */
  void flush() {
    size_t remaining = _numRecords * sizeof(addr_t);
    const uint8_t *src = reinterpret_cast<const uint8_t *>(_startPages);

    for (size_t mapPage = 0; mapPage < _numMapPages; mapPage++) {
      size_t thisPageBytes = min(remaining, _bytesPerMapPage);
      uint8_t buf[hwPageSizeBytes];
      memcpy(buf, src, thisPageBytes);

      uint32_t crc = computeCrc32(buf, thisPageBytes);
      memcpy(buf + thisPageBytes, &crc, sizeof(uint32_t));

      _eeprom.write(buf, _pageAddr(mapPage), thisPageBytes + sizeof(uint32_t));

      src += thisPageBytes;
      remaining -= thisPageBytes;
    }
  };

private:
  /** Bytes of array data held by each serialized DataMap page (the rest of
   * the page is the trailing CRC32). */
  static constexpr size_t _bytesPerMapPage = hwPageSizeBytes - sizeof(uint32_t);

  /** Byte address of the `mapPage`'th hardware page of the serialized map. */
  addr_t _pageAddr(size_t mapPage) const {
    return (_mapStartPageNum + mapPage) * hwPageSizeBytes;
  }

  /** The EEPROM this DataMap is persisted to. */
  GenericSpiEeprom &_eeprom;

  /** The hardware page at which the serialized DataMap begins. */
  const addr_t _mapStartPageNum;

  /** The records this DataMap is responsible for tracking. */
  const DataMappable *const *_records;

  /** Number of records tracked (and length of the backing array). */
  const size_t _numRecords;

  /** Number of hardware pages needed to serialize the backing array. */
  const size_t _numMapPages;

  /** In-memory copy of the start-page array, indexed by record id. */
  addr_t *_startPages;
};

/**
 * A container for a fixed-size data element `T`, paired with a CRC32,
 * starting at some hardware page within an EEPROM.
 *
 * This is similar to `EepromBacked<T>` in eeprom-m95256.h, but where
 * `EepromBacked` is pinned to a fixed byte address for the lifetime of the
 * object, a `PageBackedData<T>`'s page may be reassigned (e.g. by a
 * wear-leveling allocator via a `DataMap`). `T`, plus its CRC32 trailer, must
 * fit within a single hardware page; every load/store is one page-sized
 * transaction.
 */
template <typename T, size_t hwPageSizeBytes>
class PageBackedData : public DataMappable {
  static_assert(sizeof(T) + sizeof(uint32_t) <= hwPageSizeBytes,
                "T plus its CRC32 trailer must fit within one hardware page.");

public:
  using addr_t = uint16_t;

  PageBackedData(addr_t recordId, addr_t pageNum, GenericSpiEeprom &eeprom)
      : data(), recordId(recordId), _pageNum(pageNum), _eeprom(eeprom){};
  PageBackedData(const T &initial, addr_t recordId, addr_t pageNum,
                 GenericSpiEeprom &eeprom)
      : data(initial), recordId(recordId), _pageNum(pageNum), _eeprom(eeprom){};

  /** PageBackedData always occupies exactly one hardware page. */
  size_t sizeInPages() const override { return 1; };

  /** This record's unique id within its DataMap. */
  addr_t getRecordId() const override { return recordId; };

  /** The hardware page this record is currently stored at. */
  addr_t getPageNum() const { return _pageNum; };

  /** Reassign the hardware page this record is stored at (e.g. when relocated
   * by a wear-leveling allocator). Does not move the stored data; call
   * store() afterward to write `data` to the new page. */
  void setPageNum(addr_t newPageNum) { _pageNum = newPageNum; };

  /**
   * Load the stored copy of `data` from the current page. Verifies the
   * CRC32 trailer; if it does not match, returns false and leaves `data`
   * unmodified.
   */
  bool load() {
    uint8_t buf[sizeof(T) + sizeof(uint32_t)];
    size_t n = _eeprom.read(buf, _pageNum * hwPageSizeBytes, sizeof(buf));
    if (n != sizeof(buf)) {
      return false;
    }

    uint32_t storedCrc;
    memcpy(&storedCrc, buf + sizeof(T), sizeof(uint32_t));
    if (computeCrc32(buf, sizeof(T)) != storedCrc) {
      return false;
    }

    memcpy(&data, buf, sizeof(T));
    return true;
  };

  /**
   * Store `data` to the current page, along with a freshly computed CRC32
   * trailer, in a single page-sized write transaction.
   *
   * Returns the number of bytes written.
   */
  size_t store() {
    uint8_t buf[sizeof(T) + sizeof(uint32_t)];
    memcpy(buf, &data, sizeof(T));

    uint32_t crc = computeCrc32(buf, sizeof(T));
    memcpy(buf + sizeof(T), &crc, sizeof(uint32_t));

    return _eeprom.write(buf, _pageNum * hwPageSizeBytes, sizeof(buf));
  };

  /** Local copy of the EEPROM-backed data. */
  T data;

  /** This record's unique id within its DataMap. */
  const addr_t recordId;

private:
  /** The hardware page currently backing this record. */
  addr_t _pageNum;

  /** The EEPROM backing this data. */
  GenericSpiEeprom &_eeprom;
};

#endif /* _EEPROM_WEAR_LEVEL_H */
