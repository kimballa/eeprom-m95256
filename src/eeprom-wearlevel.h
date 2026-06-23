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
 * Minimal interface a deadlist must expose for a `WearLevelingPageData<T,
 * hwPageSizeBytes, k>` ring to consult and update, without needing to know
 * the enclosing EEPROM's total `bitSize` (and thus without becoming a
 * template parameter of `WearLevelingPageData` itself).
 */
class DeadPageOracle {
public:
  virtual ~DeadPageOracle() = default;

  /** Return true if the page identified by pageNum is marked dead and should
   * not be used for storage. */
  virtual bool isPageDead(size_t pageNum) const = 0;

  /** Mark the specified page as no longer accessible. */
  virtual void markPageDead(size_t pageNum) = 0;
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
class WearLevelDeadList : public EepromPageBitmap<bitSize, hwPageSizeBytes>,
                          public DeadPageOracle {
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
  bool isPageDead(size_t pageNum) const override { return Base::isPageSet(pageNum); }

  bool isPageLive(size_t pageNum) const { return !Base::isPageSet(pageNum); };

  /** Mark the specified page as no longer accessible. */
  void markPageDead(size_t pageNum) override { Base::setPageBit(pageNum); };
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

  virtual ~DataMappable() = default;

  /** The number of complete hardware pages this record occupies. */
  virtual size_t sizeInPages() const = 0;

  /**
   * This record's unique id, in the range 0..n-1 for the n records tracked by
   * its DataMap. This also serves as the record's index within the DataMap's
   * backing array. The caller is responsible for ensuring ids are unique.
   */
  virtual addr_t getRecordId() const = 0;

  /** The hardware page this record is currently stored at. */
  virtual addr_t getPageNum() const = 0;

  /** Reassign the hardware page this record is stored at (e.g. when relocated
   * by a wear-leveling allocator). Does not move the stored data; call
   * store() afterward to write the data to the new page. */
  virtual void setPageNum(addr_t newPageNum) = 0;

  /** Load this record's data from its current page. Returns false (data left
   * unmodified) if the stored copy fails its integrity check. */
  virtual bool load() = 0;

  /** Persist this record's in-memory data to its current page. Returns false
   * if the write could not be verified (e.g. a readback mismatch). */
  virtual bool store() = 0;
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

  /** Number of hardware pages needed to serialize the backing array. */
  size_t numMapPages() const { return _numMapPages; }

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

  /** The page this record is stored at starts out zeroed; a `DataMap` (or an
   * `EepromPageManager`) is responsible for assigning the real page via
   * setPageNum() before this is used for storage. */
  PageBackedData(addr_t recordId, GenericSpiEeprom &eeprom)
      : data(), recordId(recordId), _pageNum(0), _eeprom(eeprom){};
  PageBackedData(const T &initial, addr_t recordId, GenericSpiEeprom &eeprom)
      : data(initial), recordId(recordId), _pageNum(0), _eeprom(eeprom){};

  /** PageBackedData always occupies exactly one hardware page. */
  size_t sizeInPages() const override { return 1; };

  /** This record's unique id within its DataMap. */
  addr_t getRecordId() const override { return recordId; };

  /** The hardware page this record is currently stored at. */
  addr_t getPageNum() const override { return _pageNum; };

  /** Reassign the hardware page this record is stored at (e.g. when relocated
   * by a wear-leveling allocator). Does not move the stored data; call
   * store() afterward to write `data` to the new page. */
  void setPageNum(addr_t newPageNum) override { _pageNum = newPageNum; };

  /**
   * Load the stored copy of `data` from the current page. Verifies the
   * CRC32 trailer; if it does not match, returns false and leaves `data`
   * unmodified.
   */
  bool load() override {
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
   * trailer, in a single page-sized write transaction. Immediately reads the
   * page back and confirms it matches what was written (both byte-for-byte
   * and via CRC32), so a caller can detect a bad write/page instead of
   * silently trusting it.
   *
   * Returns true iff the write was verified.
   */
  bool store() override {
    uint8_t buf[sizeof(T) + sizeof(uint32_t)];
    memcpy(buf, &data, sizeof(T));

    uint32_t crc = computeCrc32(buf, sizeof(T));
    memcpy(buf + sizeof(T), &crc, sizeof(uint32_t));

    size_t written = _eeprom.write(buf, _pageNum * hwPageSizeBytes, sizeof(buf));
    if (written != sizeof(buf)) {
      return false;
    }

    uint8_t readBack[sizeof(T) + sizeof(uint32_t)];
    size_t readCount =
        _eeprom.read(readBack, _pageNum * hwPageSizeBytes, sizeof(readBack));
    if (readCount != sizeof(readBack)) {
      return false;
    }
    if (memcmp(buf, readBack, sizeof(readBack)) != 0) {
      return false;
    }

    uint32_t readBackCrc;
    memcpy(&readBackCrc, readBack + sizeof(T), sizeof(uint32_t));
    return computeCrc32(readBack, sizeof(T)) == readBackCrc;
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

/**
 * A container for a fixed-size data element `T`, proactively wear-leveled
 * across a small ring of `k` hardware pages, for data that's written far
 * more often than ordinary `PageBackedData<T>` records can tolerate.
 *
 * `T`'s home is a contiguous run of `k` hardware pages (`sizeInPages()==k`),
 * assigned as a single block by an `EepromPageManager`/`DataMap` exactly as
 * for any other `DataMappable`. Each `store()` call writes to the *next*
 * page in the ring rather than overwriting the same page every time, so no
 * single physical page absorbs more than 1/k of the total write traffic.
 *
 * Each ring slot's hardware page holds:
 *   [writeSequenceId: uint32_t][T][CRC32 of the above]
 * The writeSequenceId increments by one on every store() (wrapping from
 * UINT32_MAX back to 1, never to 0, since 0 is never written and thus can't
 * collide with a legitimately-stored value). On load(), every slot is read
 * and CRC-checked; the most-recently-written *valid* slot is identified as
 * the one with the largest writeSequenceId -- except where that would be
 * ambiguous because the counter has wrapped partway around the ring (some
 * slots still hold large pre-wrap values, others already hold small
 * post-wrap values): that case is detected by seeing literal UINT32_MAX
 * among the candidates alongside another candidate well below it (more than
 * `k` less), and resolved by considering only candidates whose
 * writeSequenceId is `<= k` (which, by construction, is exactly the set of
 * slots already written since the wrap).
 *
 * If a slot fails its post-write readback verification, it is marked dead
 * in the supplied `DeadPageOracle` (typically an `EepromPageManager`'s
 * `WearLevelDeadList`) and skipped from then on; store() tries the next live
 * slot in the ring instead of failing outright. If marking a slot dead would
 * bring the ring to half (or more) dead slots, this ring is considered worn
 * out: store() returns false without writing anywhere, signaling the caller
 * (e.g. `EepromPageManager::storeRecord()`) to relocate this record's
 * `sizeInPages()`-page block entirely, the same way it would for any other
 * `DataMappable` whose write failed.
 *
 * @param T the fixed-size data element to store.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 * @param k the number of hardware pages in the round-robin ring. Defaults to
 *   16; larger values spread writes (and thus wear) more thinly but cost
 *   more EEPROM space and a longer worst-case load()/store() scan.
 */
template <typename T, size_t hwPageSizeBytes, size_t k = 16>
class WearLevelingPageData : public DataMappable {
  static_assert(k >= 1, "k must be at least 1.");
  static_assert(2 * sizeof(uint32_t) + sizeof(T) <= hwPageSizeBytes,
                "writeSequenceId + T + its CRC32 trailer must fit within one "
                "hardware page.");

public:
  using addr_t = uint16_t;

  /** Number of hardware pages in the round-robin ring. */
  static constexpr size_t NUM_SLOTS = k;

  /** The ring's base page starts out zeroed, and the ring starts out with no
   * known valid record (as if freshly formatted); a `DataMap` (or an
   * `EepromPageManager`) is responsible for assigning the real base page via
   * setPageNum() before this is used for storage, and load() is responsible
   * for discovering whatever the most recently written slot actually is.
   *
   * The deadlist isn't taken here, but via setDeadPageOracle(): an
   * `EepromPageManager`'s deadlist doesn't exist until the manager itself is
   * constructed, but this object must already exist (so its address can go
   * into the manager's `records` array) *before* that constructor runs.
   * Until setDeadPageOracle() is called, this ring behaves as if it has no
   * dead slots. */
  WearLevelingPageData(addr_t recordId, GenericSpiEeprom &eeprom)
      : data(), recordId(recordId), _basePage(0), _currentSlot(0), _currentSeq(0),
        _eeprom(eeprom), _deadList(nullptr){};
  WearLevelingPageData(const T &initial, addr_t recordId, GenericSpiEeprom &eeprom)
      : data(initial), recordId(recordId), _basePage(0), _currentSlot(0), _currentSeq(0),
        _eeprom(eeprom), _deadList(nullptr){};

  /** A WearLevelingPageData record occupies its entire ring of `k` pages. */
  size_t sizeInPages() const override { return k; };

  /** This record's unique id within its DataMap. */
  addr_t getRecordId() const override { return recordId; };

  /** Bind (or rebind) the deadlist this ring consults before writing to a
   * slot, and updates when a slot's write fails verification. Typically
   * called once, right after constructing the `EepromPageManager` that owns
   * the deadlist (see the constructor comment for why this can't happen at
   * construction time instead). */
  void setDeadPageOracle(DeadPageOracle &deadList) { _deadList = &deadList; };

  /** The base hardware page of the ring (i.e. its first of `k` pages). */
  addr_t getPageNum() const override { return _basePage; };

  /** Reassign the ring's base page (e.g. when relocated wholesale by
   * `EepromPageManager::storeRecord()` after too many of this ring's slots
   * went dead). Forgets any previously known slot/sequence, since a new
   * ring's history starts fresh; call store() afterward to write `data`
   * there for the first time. */
  void setPageNum(addr_t newBasePage) override {
    _basePage = newBasePage;
    _currentSlot = 0;
    _currentSeq = 0;
  };

  /** Which of the `k` slots currently holds the most-recently-written data,
   * per the last successful load() or store(). Meaningless until one of
   * those has succeeded at least once. */
  size_t currentSlot() const { return _currentSlot; };

  /** The hardware page backing `currentSlot()`. */
  addr_t currentPageNum() const { return static_cast<addr_t>(_basePage + _currentSlot); };

  /** The writeSequenceId last read (via load()) or written (via store())
   * into the current slot, or 0 if no valid record has been found yet. */
  uint32_t currentWriteSeqId() const { return _currentSeq; };

  /**
   * Scan every slot in the ring, identify the most recently written valid
   * one (per the writeSequenceId/wraparound rules described above), and load
   * `data` from it.
   *
   * Returns false (leaving `data` unmodified, and forgetting any previously
   * known slot/sequence) if no slot in the ring holds a CRC-valid record --
   * e.g. a never-yet-written ring.
   */
  bool load() override {
    uint32_t rawSeq[k];
    bool validSlot[k];
    bool sawMax = false;
    bool sawLow = false;

    for (size_t i = 0; i < k; i++) {
      uint8_t buf[_slotSize];
      validSlot[i] = _readSlot(i, buf) && _crcValid(buf);
      if (!validSlot[i]) {
        continue;
      }

      memcpy(&rawSeq[i], buf, sizeof(uint32_t));
      if (rawSeq[i] == UINT32_MAX) {
        sawMax = true;
      }
      if (rawSeq[i] < UINT32_MAX - k) {
        sawLow = true;
      }
    }

    bool wrapped = sawMax && sawLow;
    bool found = false;
    size_t bestSlot = 0;
    uint32_t bestSeq = 0;

    for (size_t i = 0; i < k; i++) {
      if (!validSlot[i] || (wrapped && rawSeq[i] > k)) {
        continue;
      }
      if (!found || rawSeq[i] > bestSeq) {
        found = true;
        bestSeq = rawSeq[i];
        bestSlot = i;
      }
    }

    if (!found && wrapped) {
      // Defensive fallback: the wrap heuristic excluded every candidate,
      // which shouldn't happen for a ring written only by this class.
      // Reconsider without the restriction rather than reporting failure.
      for (size_t i = 0; i < k; i++) {
        if (!validSlot[i]) {
          continue;
        }
        if (!found || rawSeq[i] > bestSeq) {
          found = true;
          bestSeq = rawSeq[i];
          bestSlot = i;
        }
      }
    }

    if (!found) {
      _currentSeq = 0;
      return false;
    }

    uint8_t buf[_slotSize];
    _readSlot(bestSlot, buf);
    memcpy(&data, buf + sizeof(uint32_t), sizeof(T));

    _currentSlot = bestSlot;
    _currentSeq = bestSeq;
    return true;
  };

  /**
   * Write `data` to the next slot in the round-robin ring (the live slot
   * after `currentSlot()`, or slot 0 if no valid record is known yet), with
   * a freshly incremented writeSequenceId and CRC32 trailer, then read it
   * back to verify the write.
   *
   * If that slot's write can't be verified, it is marked dead and the next
   * live slot is tried instead, continuing around the ring. Returns false,
   * without writing anywhere, if half or more of the ring's slots are
   * already dead (this ring is considered worn out -- the caller should
   * relocate this record to a fresh ring elsewhere) or if every remaining
   * live slot's write fails verification.
   */
  bool store() override {
    if (_deadSlotCount() * 2 >= k) {
      return false;
    }

    uint32_t nextSeq;
    size_t startSlot;
    if (_currentSeq == 0) {
      nextSeq = 1;
      startSlot = 0;
    } else {
      nextSeq = (_currentSeq == UINT32_MAX) ? 1 : _currentSeq + 1;
      startSlot = (_currentSlot + 1) % k;
    }

    for (size_t attempt = 0; attempt < k; attempt++) {
      size_t slot = (startSlot + attempt) % k;
      if (_deadList != nullptr && _deadList->isPageDead(_basePage + slot)) {
        continue;
      }

      if (_writeSlot(slot, nextSeq)) {
        _currentSlot = slot;
        _currentSeq = nextSeq;
        return true;
      }

      if (_deadList != nullptr) {
        _deadList->markPageDead(_basePage + slot);
        if (_deadSlotCount() * 2 >= k) {
          return false;
        }
      }
    }

    return false;
  };

  /** Local copy of the EEPROM-backed data. */
  T data;

  /** This record's unique id within its DataMap. */
  const addr_t recordId;

private:
  /** Bytes occupied by one ring slot: writeSequenceId + T + CRC32. */
  static constexpr size_t _slotSize = 2 * sizeof(uint32_t) + sizeof(T);

  /** Byte address of the `slot`'th hardware page in the ring. */
  addr_t _pageAddr(size_t slot) const {
    return static_cast<addr_t>((_basePage + slot) * hwPageSizeBytes);
  };

  /** Number of this ring's `k` slots currently marked dead. Always 0 until
   * setDeadPageOracle() has been called. */
  size_t _deadSlotCount() const {
    if (_deadList == nullptr) {
      return 0;
    }
    size_t count = 0;
    for (size_t i = 0; i < k; i++) {
      if (_deadList->isPageDead(_basePage + i)) {
        count++;
      }
    }
    return count;
  };

  /** Read slot `i`'s raw bytes (writeSequenceId + T + CRC32) into `buf`.
   * Returns false if the read was short. */
  bool _readSlot(size_t i, uint8_t buf[_slotSize]) const {
    return _eeprom.read(buf, _pageAddr(i), _slotSize) == _slotSize;
  };

  /** Returns true iff `buf` (a full slot read via `_readSlot()`) has a
   * CRC32 trailer matching its writeSequenceId+T payload. */
  static bool _crcValid(const uint8_t buf[_slotSize]) {
    uint32_t storedCrc;
    memcpy(&storedCrc, buf + sizeof(uint32_t) + sizeof(T), sizeof(uint32_t));
    return computeCrc32(buf, sizeof(uint32_t) + sizeof(T)) == storedCrc;
  };

  /**
   * Write `data` (tagged with `seq`) to ring slot `slot`, then read it back
   * and confirm it matches byte-for-byte and via CRC32, exactly as
   * `PageBackedData::store()` does for its single page.
   */
  bool _writeSlot(size_t slot, uint32_t seq) {
    uint8_t buf[_slotSize];
    memcpy(buf, &seq, sizeof(seq));
    memcpy(buf + sizeof(seq), &data, sizeof(T));

    uint32_t crc = computeCrc32(buf, sizeof(seq) + sizeof(T));
    memcpy(buf + sizeof(seq) + sizeof(T), &crc, sizeof(crc));

    addr_t pageAddr = _pageAddr(slot);
    size_t written = _eeprom.write(buf, pageAddr, _slotSize);
    if (written != _slotSize) {
      return false;
    }

    uint8_t readBack[_slotSize];
    if (!_readSlot(slot, readBack)) {
      return false;
    }
    if (memcmp(buf, readBack, _slotSize) != 0) {
      return false;
    }

    return _crcValid(readBack);
  };

  /** The base hardware page of the ring; the ring spans
   * `[_basePage, _basePage + k)`. */
  addr_t _basePage;

  /** Which slot (offset from `_basePage`) currently holds the
   * most-recently-written data, per the last load()/store(). */
  size_t _currentSlot;

  /** The writeSequenceId of `_currentSlot`'s data, or 0 if no valid record
   * has been found/written yet (in which case the next store() starts the
   * ring over at slot 0, sequence 1). */
  uint32_t _currentSeq;

  /** The EEPROM backing this ring. */
  GenericSpiEeprom &_eeprom;

  /** Where dead slots are recorded, and consulted to skip them. Null until
   * setDeadPageOracle() is called. */
  DeadPageOracle *_deadList;
};

/**
 * The fixed-format "superblock" persisted at hardware page 0 of an EEPROM
 * managed by `EepromPageManager`. Verified via `PageBackedData`'s usual
 * CRC32 trailer.
 */
struct EepromRootPageData {
  /** Set to `EepromPageManager<...>::ROOT_PAGE_MAGIC` once formatted. */
  uint32_t magic;

  /** The on-disk layout version of the page-manager system itself (deadlist
   * / usage list / data map placement), independent of the application's own
   * record format. */
  uint32_t systemVersion;

  /** The application-supplied format version of the records stored above the
   * system pages. Lets an application detect that it's looking at an EEPROM
   * formatted by an incompatible build of itself. */
  uint32_t appVersion;

  /** Reserved for future use; always zero. */
  uint8_t reserved[4];
};

/**
 * Top-level coordinator for wear-leveled EEPROM storage.
 *
 * Lays out a fixed set of "system" pages at the start of the EEPROM:
 *   - page 0: the root page (`EepromRootPageData`).
 *   - page 1: the `WearLevelDeadList`.
 *   - page 2: the `PageUsageBitmap` (free/used list).
 *   - page 3..3+numMapPages-1: the `DataMap`.
 * Application data for the supplied `DataMappable` records begins
 * immediately after the data map (see `firstDataPageNum()`).
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes> class EepromPageManager {
  /** Total number of hardware pages in the managed EEPROM. */
  static constexpr size_t _numPages = (bitSize / 8) / hwPageSizeBytes;

  /**
   * Number of hardware pages needed to serialize a single `EepromPageBitmap`
   * (the deadlist or the usage list) covering all `_numPages` pages of this
   * EEPROM. For a small EEPROM this is 1, but for a large one (e.g. 1 MiB at
   * 64 bytes/page -> 16384 pages -> a 2048-byte bitmap) it can be several
   * pages. The deadlist and usage list are always the same size as each
   * other, since both cover the same `_numPages`.
   */
  static constexpr size_t _bitmapTableSizeBytes = (_numPages + 7) / 8;
  static constexpr size_t _bitmapNumPages =
      (_bitmapTableSizeBytes + hwPageSizeBytes - 1) / hwPageSizeBytes;

public:
  using addr_t = uint16_t;

  /**
   * The hardware page number at which each system structure is rooted. The
   * deadlist and usage list placements account for either bitmap spanning
   * more than one hardware page; the data map always immediately follows
   * the usage list.
   */
  static constexpr size_t ROOT_PAGE_NUM = 0;
  static constexpr size_t DEAD_LIST_PAGE_NUM = ROOT_PAGE_NUM + 1;
  static constexpr size_t USAGE_LIST_PAGE_NUM = DEAD_LIST_PAGE_NUM + _bitmapNumPages;
  static constexpr size_t DATA_MAP_START_PAGE_NUM = USAGE_LIST_PAGE_NUM + _bitmapNumPages;

  /** Magic number proving the root page has been formatted by this system. */
  static constexpr uint32_t ROOT_PAGE_MAGIC = 0x0571AC94;

  /** The page-manager system's own on-disk format version. */
  static constexpr uint32_t SYSTEM_FORMAT_VERSION = 1;

  /** How long (in ms) a single `storeRecord()` write+readback may take before
   * it's treated as a stalled/failed write, prompting relocation. The M95256
   * datasheet specifies a 5ms max write time; this allows generous margin. */
  static constexpr unsigned long WRITE_STALL_THRESHOLD_MILLIS = 20;

  /** Root page is valid: magic, system version, and app version all check out. */
  static constexpr int ROOT_PAGE_OK = 0;
  /** The root page failed to load (read error or CRC32 mismatch) -- the
   * EEPROM has likely never been formatted, or is corrupt. */
  static constexpr int ROOT_PAGE_ERR_LOAD_FAILED = -1;
  /** The root page loaded cleanly but its magic number doesn't match -- this
   * EEPROM was never formatted by an EepromPageManager. */
  static constexpr int ROOT_PAGE_ERR_BAD_MAGIC = -2;
  /** The root page's system format version isn't one this build understands. */
  static constexpr int ROOT_PAGE_ERR_BAD_SYSTEM_VERSION = -3;
  /** The root page's application format version doesn't match what the
   * application asked for in its EepromPageManager constructor. */
  static constexpr int ROOT_PAGE_ERR_BAD_APP_VERSION = -4;
  /** The data map failed to load (CRC32 mismatch on one or more of its pages). */
  static constexpr int ERR_DATA_MAP_CORRUPT = -5;

  /**
   * Create a page manager for `numRecords` `DataMappable` records, backed by
   * `eeprom`. `appFormatVersion` is the application's own data format
   * version, persisted to (and checked against) the root page.
   */
  EepromPageManager(GenericSpiEeprom &eeprom, DataMappable *const *records,
                     size_t numRecords, uint32_t appFormatVersion)
      : _eeprom(eeprom),
        _deadList(eeprom, DEAD_LIST_PAGE_NUM * hwPageSizeBytes),
        _usage(eeprom, USAGE_LIST_PAGE_NUM * hwPageSizeBytes),
        _dataMap(eeprom, DATA_MAP_START_PAGE_NUM, records, numRecords),
        _rootPage(/*recordId=*/0, eeprom), _records(records),
        _numRecords(numRecords), _appFormatVersion(appFormatVersion),
        _firstDataPageNum(DATA_MAP_START_PAGE_NUM + _dataMap.numMapPages()){};

  /**
   * Format a brand-new EEPROM: writes the root page, formats the deadlist
   * and usage list, reserves the system pages (root/deadlist/usage/map) as
   * in-use, and assigns each known `DataMappable` an initial run of free
   * pages (sized via `sizeInPages()`), recording the assignment in both the
   * usage list and the data map.
   *
   * Returns false if there isn't enough free space to place every record
   * (records placed before the failure remain assigned).
   */
  bool formatNew() {
    _deadList.formatNew();
    _usage.formatNew();

    for (size_t p = 0; p < _firstDataPageNum; p++) {
      _usage.markPageInUse(p);
    }

    _rootPage.data.magic = ROOT_PAGE_MAGIC;
    _rootPage.data.systemVersion = SYSTEM_FORMAT_VERSION;
    _rootPage.data.appVersion = _appFormatVersion;
    memset(_rootPage.data.reserved, 0, sizeof(_rootPage.data.reserved));
    _rootPage.store();

    size_t nextSearch = _firstDataPageNum;
    for (size_t i = 0; i < _numRecords; i++) {
      DataMappable *record = _records[i];
      size_t numPages = record->sizeInPages();
      size_t startPage = _findFreePageRun(nextSearch, numPages);
      if (startPage == ERR_NO_FREE_PAGES) {
        return false;
      }

      for (size_t j = 0; j < numPages; j++) {
        _usage.markPageInUse(startPage + j);
      }

      _dataMap.setStartPage(record->getRecordId(), static_cast<addr_t>(startPage));
      record->setPageNum(static_cast<addr_t>(startPage));
      nextSearch = startPage + numPages;
    }
    _dataMap.flush();
    return true;
  };

  /**
   * Load the root page, deadlist, usage list, and data map from the EEPROM.
   * Does *not* load any individual record's data -- callers are responsible
   * for calling load() on whichever records they actually need.
   *
   * Updates each known `DataMappable`'s cached page number from the freshly
   * loaded data map.
   *
   * Returns ROOT_PAGE_OK on success, or the first applicable
   * ROOT_PAGE_ERR_... / ERR_DATA_MAP_CORRUPT code otherwise.
   */
  int init() {
    int rootStatus = checkRootPageValid();
    if (rootStatus != ROOT_PAGE_OK) {
      return rootStatus;
    }

    _deadList.init();
    _usage.init();
    if (!_dataMap.init()) {
      return ERR_DATA_MAP_CORRUPT;
    }

    for (size_t i = 0; i < _numRecords; i++) {
      _records[i]->setPageNum(_dataMap.getStartPage(_records[i]->getRecordId()));
    }

    return ROOT_PAGE_OK;
  };

  /**
   * Load the root page and confirm it is valid: readable (CRC32-clean), has
   * the expected magic number, and matches both the system and
   * application-level format versions this manager expects.
   *
   * Returns ROOT_PAGE_OK, or the first applicable ROOT_PAGE_ERR_* code.
   */
  int checkRootPageValid() {
    if (!_rootPage.load()) {
      return ROOT_PAGE_ERR_LOAD_FAILED;
    }
    if (_rootPage.data.magic != ROOT_PAGE_MAGIC) {
      return ROOT_PAGE_ERR_BAD_MAGIC;
    }
    if (_rootPage.data.systemVersion != SYSTEM_FORMAT_VERSION) {
      return ROOT_PAGE_ERR_BAD_SYSTEM_VERSION;
    }
    if (_rootPage.data.appVersion != _appFormatVersion) {
      return ROOT_PAGE_ERR_BAD_APP_VERSION;
    }
    return ROOT_PAGE_OK;
  };

  /**
   * Persist `record` to its currently assigned page(s).
   *
   * If the write cannot be verified -- either store() reports a readback
   * mismatch, or the operation took implausibly long
   * (> WRITE_STALL_THRESHOLD_MILLIS, well past the device's rated max write
   * time) -- the old page(s) are marked dead in the deadlist and freed in
   * the usage list, a fresh run of live, free pages is chosen by consulting
   * the usage list and deadlist, the data map and usage list are updated and
   * flushed, and the record is written again at its new home.
   *
   * Returns true iff `record` ends up durably (and verifiably) stored
   * somewhere.
   */
  bool storeRecord(DataMappable &record) {
    unsigned long startMillis = millis();
    bool ok = record.store();
    unsigned long elapsedMillis = millis() - startMillis;

    if (ok && elapsedMillis <= WRITE_STALL_THRESHOLD_MILLIS) {
      return true;
    }

    return _relocateAndStore(record);
  };

  /** Direct access to the deadlist, e.g. for tests or diagnostics. */
  WearLevelDeadList<bitSize, hwPageSizeBytes> &deadList() { return _deadList; };

  /** Direct access to the usage (free/used) list, e.g. for tests or
   * diagnostics. */
  PageUsageBitmap<bitSize, hwPageSizeBytes> &usage() { return _usage; };

  /** Direct access to the data map, e.g. for tests or diagnostics. */
  DataMap<hwPageSizeBytes> &dataMap() { return _dataMap; };

  /** The first hardware page available for application data; everything
   * before this is reserved for the root page, deadlist, usage list, and
   * data map. */
  size_t firstDataPageNum() const { return _firstDataPageNum; };

private:
  /**
   * Find the first run of `numPages` consecutive pages, at or after
   * `startPageNum`, that are all simultaneously free (per the usage list)
   * and live (per the deadlist), without running off the end of the
   * EEPROM. Returns ERR_NO_FREE_PAGES if no such run exists.
   */
  size_t _findFreePageRun(size_t startPageNum, size_t numPages) {
    size_t candidate = startPageNum;
    while (true) {
      candidate = _usage.findNextFreePageNum(candidate);
      if (candidate == ERR_NO_FREE_PAGES) {
        return ERR_NO_FREE_PAGES;
      }

      size_t j = 0;
      for (; j < numPages; j++) {
        size_t p = candidate + j;
        if (p >= _numPages || _usage.isPageInUse(p) || _deadList.isPageDead(p)) {
          break;
        }
      }

      if (j == numPages) {
        return candidate;
      }

      // The run starting at `candidate` is blocked at offset `j` (either off
      // the end of the EEPROM, already in-use, or dead). Resume the search
      // just past the blocking page.
      candidate = candidate + j + 1;
    }
  };

  /**
   * Relocate `record` to a fresh run of pages after a failed store(), then
   * retry the write at the new location.
   */
  bool _relocateAndStore(DataMappable &record) {
    size_t numPages = record.sizeInPages();
    addr_t oldPageNum = record.getPageNum();

    for (size_t j = 0; j < numPages; j++) {
      _deadList.markPageDead(oldPageNum + j);
      _usage.markPageFree(oldPageNum + j);
    }

    size_t newPageNum = _findFreePageRun(_firstDataPageNum, numPages);
    if (newPageNum == ERR_NO_FREE_PAGES) {
      return false;
    }

    for (size_t j = 0; j < numPages; j++) {
      _usage.markPageInUse(newPageNum + j);
    }

    record.setPageNum(static_cast<addr_t>(newPageNum));
    _dataMap.setStartPage(record.getRecordId(), static_cast<addr_t>(newPageNum));
    _dataMap.flush();

    return record.store();
  };

  /** The EEPROM this page manager governs. */
  GenericSpiEeprom &_eeprom;

  /** Tracks pages that have failed a verified write and must never be
   * allocated again. */
  WearLevelDeadList<bitSize, hwPageSizeBytes> _deadList;

  /** Tracks which pages currently hold live data. */
  PageUsageBitmap<bitSize, hwPageSizeBytes> _usage;

  /** Tracks the current start page of every known record. */
  DataMap<hwPageSizeBytes> _dataMap;

  /** The fixed-location (page 0) superblock. */
  PageBackedData<EepromRootPageData, hwPageSizeBytes> _rootPage;

  /** The records this page manager is responsible for placing/relocating. */
  DataMappable *const *_records;

  /** Number of records in `_records`. */
  const size_t _numRecords;

  /** The application's own data format version, checked against the root
   * page on init(). */
  const uint32_t _appFormatVersion;

  /** First hardware page available for application data. */
  const size_t _firstDataPageNum;
};

#endif /* _EEPROM_WEAR_LEVEL_H */
