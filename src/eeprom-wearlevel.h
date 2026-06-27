// (c) Copyright 2026 Aaron Kimball
//
// Classes for managing wear level-sensitive writes to an EEPROM
// to maximize its lifetime.

#ifndef _EEPROM_WEAR_LEVEL_H
#define _EEPROM_WEAR_LEVEL_H

#include "eeprom-m95256.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

// The low-level driver (eeprom-m95256.h) addresses the device with a 16-bit
// word, matching the M95256's two-byte on-wire address; that header is
// deployed on real hardware and is intentionally left at uint16_t. The
// higher-level page-management code here uses a wider page/address type so its
// arithmetic never overflows and its persisted structures aren't pinned to a
// 16-bit world. Byte addresses handed to the driver are narrowed at the call
// boundary; for the M95256 every such address fits in 16 bits.
using eeprom_addr_t = GenericSpiEeprom::addr_t;

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
 * Crash-safe persistence engine for a fixed-size in-RAM byte buffer, shared by
 * the page bitmaps and the data map so that all three behave identically.
 *
 * ## Data Buffer
 *
 * The client of RedundantPagedStore allocates the in-RAM byte buffer, specifies
 * the buffer address to the RedundantPagedStore constructor, and retains its
 * own pointer to the buffer. The client directly manipulates the data through
 * that pointer, rather than going through RedundantPagedStore. The client is
 * responsible for using markDirty() / markDirtyByteRange() after making any
 * updates to the data in the buffer, and for calling flushDirty() when
 * necessary.
 *
 * ## Serialization
 *
 * The buffer is serialized across `numPages()` hardware pages. Two redundant
 * copies are kept: copy A starts at hardware page `copyAStartPage`, copy B at
 * `copyBStartPage`. Each logical table page occupies one hardware page in each
 * copy, laid out as:
 *
 *   [seqId : uint32_t][N payload bytes][CRC32 of seqId+payload]
 *
 * N is the number of bytes per EEPROM hardware page - 2 * sizeof(uint32_t).
 *
 * Every table page is *independently* versioned. On load() each page is taken
 * from whichever of its two slots is CRC-valid and has the higher seqId, so a
 * torn write to one slot never destroys that page's last good copy, and a
 * power loss between page writes simply leaves some pages a generation behind
 * (still self-consistent, since the structures here -- bitmaps and a map of
 * independent per-record pointers -- have no cross-page invariant).
 *
 * An update writes only the dirty pages, each to that page's currently
 * *inactive* (older) slot, then bumps that page's seqId. The active slot (the
 * last durable copy) is never touched during the write, which is what makes a
 * mid-write power loss recoverable.
 *
 * Worked example, a 3-page table. formatBoth() writes both slots of every page
 * and marks slot A active throughout, so the live composite reads A,A,A:
 *
 *     page:      0     1     2
 *     copy A:  [v0*] [v1*] [v2*]      (* = active slot, the one load() reads)
 *     copy B:  [v0 ] [v1 ] [v2 ]
 *
 * Now update only the middle page (page 1 -> v3). Its inactive slot is B, so
 * v3 is written there and page 1's active pointer flips to B; pages 0 and 2
 * are untouched. The live composite now reads A,B,A:
 *
 *     copy A:  [v0*] [v1 ] [v2*]      (page 1's copy A is now its stale backup)
 *     copy B:  [v0 ] [v3*] [v2 ]
 *
 * A crash midway through writing v3 just leaves the old v1 in copy A still
 * active and valid. The next update of page 1 would write back to slot A,
 * returning to A,A,A -- each page ping-pongs between its two slots.
 *
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t hwPageSizeBytes> class RedundantPagedStore {
  static_assert(
      hwPageSizeBytes > 2 * sizeof(uint32_t),
      "hardware page must be larger than the seqId + CRC32 overhead.");

public:
  /** Per-page overhead: a leading seqId and a trailing CRC32. */
  static constexpr size_t PER_PAGE_OVERHEAD = 2 * sizeof(uint32_t);
  /** Payload bytes carried by each hardware page (rest is overhead). */
  static constexpr size_t PAYLOAD_PER_PAGE =
      hwPageSizeBytes - PER_PAGE_OVERHEAD;

  RedundantPagedStore(GenericSpiEeprom &eeprom, uint8_t *buf, size_t bufLen,
                      uint32_t copyAStartPage, uint32_t copyBStartPage)
      : _eeprom(eeprom), _buf(buf), _bufLen(bufLen),
        _numPages((bufLen + PAYLOAD_PER_PAGE - 1) / PAYLOAD_PER_PAGE),
        _copyA(copyAStartPage), _copyB(copyBStartPage),
        _seq(_numPages ? new uint32_t[_numPages] : nullptr),
        _active(_numPages ? new uint8_t[_numPages] : nullptr),
        _dirty(_numPages ? new bool[_numPages] : nullptr) {
    if (_numPages) {
      memset(_seq, 0, sizeof(uint32_t) * _numPages);
      memset(_active, 0, sizeof(uint8_t) * _numPages);
      memset(_dirty, 0, sizeof(bool) * _numPages);
    }
  }

  RedundantPagedStore(RedundantPagedStore<hwPageSizeBytes> &&store)
      : _eeprom(store._eeprom), _buf(store._buf), _bufLen(store._bufLen),
        _numPages(store._numPages), _copyA(store._copyA), _copyB(store._copyB),
        _seq(store._seq), _active(store._active), _dirty(store._dirty) {
    // Transfer ownership of variable-length state arrays; don't allocate new
    // ones and free the old ones.
    store._seq = nullptr;
    store._active = nullptr;
    store._dirty = nullptr;
  }

  ~RedundantPagedStore() {
    if (_seq) {
      delete[] _seq;
    }

    if (_active) {
      delete[] _active;
    }

    if (_dirty) {
      delete[] _dirty;
    }
  }

  /** Number of hardware pages each copy spans. */
  size_t numPages() const { return _numPages; }

  /**
   * Load the buffer from the EEPROM, taking each page from its newest valid
   * slot. Returns false if any page had no valid slot in either copy (e.g. a
   * never-formatted, all-0xFF region).
   */
  bool load() {
    bool allOk = true;
    for (size_t p = 0; p < _numPages; p++) {
      uint8_t pa[PAYLOAD_PER_PAGE];
      uint8_t pb[PAYLOAD_PER_PAGE];
      uint32_t sa = 0, sb = 0;
      bool va = _readSlot(p, 0, pa, sa);
      bool vb = _readSlot(p, 1, pb, sb);

      size_t off = p * PAYLOAD_PER_PAGE;
      size_t len = std::min(PAYLOAD_PER_PAGE, _bufLen - off);

      _dirty[p] = false;
      if (va && (!vb || sa >= sb)) {
        memcpy(_buf + off, pa, len);
        _seq[p] = sa;
        _active[p] = 0;
      } else if (vb) {
        memcpy(_buf + off, pb, len);
        _seq[p] = sb;
        _active[p] = 1;
      } else {
        _seq[p] = 0;
        _active[p] = 0;
        allOk = false;
      }
    }
    return allOk;
  }

  /** Write both copies in full from the current buffer contents, starting a
   * fresh generation. Used when formatting so later single-page updates have
   * two valid copies to alternate between. */
  void formatBoth() {
    for (size_t p = 0; p < _numPages; p++) {
      _writeSlot(p, 0, 1);
      _writeSlot(p, 1, 1);
      _seq[p] = 1;
      _active[p] = 0;
      _dirty[p] = false;
    }
  }

  /** Mark the table page at index `pageIndex` as needing to be written on the
   * next flush. */
  void markDirty(size_t pageIndex) {
    if (pageIndex < _numPages) {
      _dirty[pageIndex] = true;
    }
  }

  /** Mark dirty every table page touched by buffer bytes [firstByteIdx,
   * lastByteIdx]. */
  void markDirtyByteRange(size_t firstByteIdx, size_t lastByteIdx) {
    size_t firstPage = firstByteIdx / PAYLOAD_PER_PAGE;
    size_t lastPage = lastByteIdx / PAYLOAD_PER_PAGE;
    for (size_t p = firstPage; p <= lastPage && p < _numPages; p++) {
      _dirty[p] = true;
    }
  }

  /**
   * Write every dirty page to its inactive slot (crash-safely, never touching
   * the active slot), bump that page's seqId, and clear its dirty flag.
   * Returns false if any page's write could not be verified.
   *
   * This is best-effort and report-once: even on a write failure the page's
   * dirty flag is cleared (and its active slot left pointing at the last good
   * copy), so the failure surfaces ONLY via this call's `false` return -- a
   * later flushDirty() will see nothing dirty and return true. Callers must
   * therefore treat a `false` return as terminal rather than retryable. For
   * the system structures this engine backs, that's the right contract: their
   * pages live at fixed addresses with no spare location, so a failed
   * _writeSlot() is an unrecoverable bad-cell fault, not something a retry
   * could fix. A caller that wants to abandon the in-RAM change on failure
   * (e.g. EepromPageManager's relocation commit) must also roll its own
   * in-RAM state back, rather than relying on the dropped dirty flag.
   */
  bool flushDirty() {
    bool ok = true;
    for (size_t p = 0; p < _numPages; p++) {
      if (!_dirty[p]) {
        continue;
      }
      // Page A is '0', Page B is '1'; inactive page flag = 1 - _active
      uint8_t inactive = static_cast<uint8_t>(1 - _active[p]);
      uint32_t newSeq = _seq[p] + 1;
      if (_writeSlot(p, inactive, newSeq)) {
        _active[p] = inactive;
        _seq[p] = newSeq;
      } else {
        ok = false;
      }
      _dirty[p] = false;
    }
    return ok;
  }

  /** Mark all pages dirty and flush them. */
  bool flushAll() {
    for (size_t p = 0; p < _numPages; p++) {
      _dirty[p] = true;
    }
    return flushDirty();
  }

  /** Read both copies and confirm every page has at least one CRC-valid slot.
   * Does not modify the in-RAM buffer. */
  bool verify() const {
    for (size_t p = 0; p < _numPages; p++) {
      uint8_t pa[PAYLOAD_PER_PAGE];
      uint8_t pb[PAYLOAD_PER_PAGE];
      uint32_t sa = 0, sb = 0;
      if (!_readSlot(p, 0, pa, sa) && !_readSlot(p, 1, pb, sb)) {
        return false;
      }
    }
    return true;
  }

private:
  /** Byte address of table page `pageIndex` within copy `slot` (0=A, 1=B). */
  eeprom_addr_t _slotAddr(size_t pageIndex, uint8_t slot) const {
    uint32_t physPage = (slot ? _copyB : _copyA) + pageIndex;
    return static_cast<eeprom_addr_t>(physPage * hwPageSizeBytes);
  }

  /** Read one slot; on a CRC-valid read, copy its payload into `payloadOut`
   * and its seqId into `seqOut`, and return true. */
  bool _readSlot(size_t pageIndex, uint8_t slot,
                 uint8_t payloadOut[PAYLOAD_PER_PAGE], uint32_t &seqOut) const {
    size_t off = pageIndex * PAYLOAD_PER_PAGE;
    size_t len = std::min(PAYLOAD_PER_PAGE, _bufLen - off);
    // Total bytes to read includes payload plus CRC32 plus writeSequenceId.
    size_t total = sizeof(uint32_t) + len + sizeof(uint32_t);

    uint8_t img[hwPageSizeBytes];
    if (_eeprom.read(img, _slotAddr(pageIndex, slot), total) != total) {
      return false;
    }
    // Data format of the page image is [writeSeqId:4][payload:N][CRC32:4]
    uint32_t storedCrc;
    memcpy(&storedCrc, img + sizeof(uint32_t) + len, sizeof(uint32_t));
    if (computeCrc32(img, sizeof(uint32_t) + len) != storedCrc) {
      return false;
    }
    memcpy(&seqOut, img, sizeof(uint32_t));
    memcpy(payloadOut, img + sizeof(uint32_t), len);
    return true;
  }

  /** Build and write table page `pageIndex` into copy `slot` with sequence id
   * `seq`, then read it back and confirm it landed byte-for-byte. */
  bool _writeSlot(size_t pageIndex, uint8_t slot, uint32_t seq) {
    size_t off = pageIndex * PAYLOAD_PER_PAGE;
    size_t len = std::min(PAYLOAD_PER_PAGE, _bufLen - off);
    size_t total = sizeof(uint32_t) + len + sizeof(uint32_t);

    // Data format of the page image is [writeSeqId:4][payload:N][CRC32:4]
    uint8_t img[hwPageSizeBytes];
    uint8_t *cursor = img;
    memcpy(cursor, &seq, sizeof(uint32_t));
    cursor += sizeof(uint32_t);
    memcpy(cursor, _buf + off, len);
    cursor += len;
    uint32_t crc = computeCrc32(img, sizeof(uint32_t) + len);
    memcpy(cursor, &crc, sizeof(uint32_t));

    eeprom_addr_t addr = _slotAddr(pageIndex, slot);
    if (_eeprom.write(img, addr, total) != total) {
      return false;
    }
    uint8_t readBack[hwPageSizeBytes];
    if (_eeprom.read(readBack, addr, total) != total) {
      return false;
    }
    return memcmp(img, readBack, total) == 0;
  }

  GenericSpiEeprom &_eeprom;
  /**
   * Arbitrary data buffer backed by EEPROM and managed by this data structure.
   *
   * Each serialized copy may span multiple pages of EEPROM and have CRC32 and
   * sequence id metadata interspersed with the real data, but the data is
   * reconstituted as a single logical sequential array and stored here in _buf.
   *
   * The client of RedundantPagedStore has its own pointer to the same address
   * as _buf and directly manipulates the data. The client is responsible for
   * using markDirty() / markDirtyByteRange() after making any updates to the
   * data in the buffer.
   */
  uint8_t *_buf;
  /** Number of bytes in _buf. */
  const size_t _bufLen;
  /** Number of EEPROM pages occupied by one copy of the data. */
  const size_t _numPages;
  /** Sequential page number in the EEPROM where copy A begins.  */
  const uint32_t _copyA;
  /** Sequential page number in the EEPROM where copy B begins.  */
  const uint32_t _copyB;

  /** Newest known seqId of each table page. */
  uint32_t *_seq;
  /** Which slot (0=A, 1=B) currently holds each page's newest data. */
  uint8_t *_active;
  /** Which pages have unwritten in-RAM changes. */
  bool *_dirty;
};

/**
 * A bitmap where each bit in the bitmap corresponds to one page of the EEPROM.
 * The values of 0 and 1 are application-specific to the client of the bitmap.
 *
 * Persistence is crash-safe and dual-copy via an embedded
 * `RedundantPagedStore`. Updates flush only the hardware page(s) actually made
 * dirty; the span helpers can defer flushing so that marking a contiguous run
 * of pages costs a single flush instead of one per bit.
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes> class EepromPageBitmap {
public:
  using addr_t = uint32_t;

  /** Create a bitmap object whose two redundant copies are stored starting at
   * hardware pages `copyAStartPage` and `copyBStartPage`. */
  EepromPageBitmap(GenericSpiEeprom &eeprom, addr_t copyAStartPage,
                   addr_t copyBStartPage)
      : _store(eeprom, _bits, _pageTableSize, copyAStartPage, copyBStartPage) {}

  /** Set up the bitmap by loading it from EEPROM. Returns false if neither
   * copy of some page validated. */
  bool init() { return _store.load(); }

  /** Zero the whole table in RAM only (no EEPROM write). Pair with
   * formatBoth() once any initial bits have been staged. */
  void clearAll() { memset(_bits, 0, _pageTableSize); }

  /** Write both redundant copies in full from the current in-RAM table, so
   * later single-page updates have two valid copies to alternate between. */
  void formatBoth() { _store.formatBoth(); }

  /** Set up the bitmap for a new EEPROM by zeroing the whole table and writing
   * both copies. */
  void formatNew() {
    clearAll();
    formatBoth();
  }

  /** Flush any pending (dirty) changes to the EEPROM. */
  bool flush() { return _store.flushDirty(); }

  /** Force-write every page of both-copy state from the current buffer. */
  bool flushAll() { return _store.flushAll(); }

  /** Read both copies back and confirm each page validates. */
  bool verify() const { return _store.verify(); }

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
  void setPageBit(size_t pageNum, bool delayFlush = false) {
    setSpan(pageNum, 1, delayFlush);
  }

  /** Mark the specified page with a 0. */
  void clearPageBit(size_t pageNum, bool delayFlush = false) {
    clearSpan(pageNum, 1, delayFlush);
  }

  /** Mark `nPages` pages starting at `startPage` with a 1. With
   * delayFlush=true the change is staged in RAM (and the affected hardware
   * pages noted as dirty) without writing; call flush() afterward. */
  void setSpan(size_t startPage, size_t nPages, bool delayFlush = false) {
    if (nPages == 0) {
      return;
    }
    for (size_t i = 0; i < nPages; i++) {
      size_t pageNum = startPage + i;
      _bits[pageNum / 8] |= static_cast<uint8_t>(1 << (pageNum & 7));
    }
    _store.markDirtyByteRange(startPage / 8, (startPage + nPages - 1) / 8);
    if (!delayFlush) {
      _store.flushDirty();
    }
  }

  /** Mark `nPages` pages starting at `startPage` with a 0. */
  void clearSpan(size_t startPage, size_t nPages, bool delayFlush = false) {
    if (nPages == 0) {
      return;
    }
    for (size_t i = 0; i < nPages; i++) {
      size_t pageNum = startPage + i;
      _bits[pageNum / 8] &= static_cast<uint8_t>(~(1 << (pageNum & 7)));
    }
    _store.markDirtyByteRange(startPage / 8, (startPage + nPages - 1) / 8);
    if (!delayFlush) {
      _store.flushDirty();
    }
  }

  /** Return the entire byte (8 page span) of the bitmap containing the bit for
   * pageNum. */
  uint8_t getPageByte(size_t pageNum) const { return _bits[pageNum / 8]; }

  static constexpr size_t _eepromSizeBytes = bitSize / 8;
  static constexpr size_t _numPages = _eepromSizeBytes / hwPageSizeBytes;
  static constexpr size_t _pageTableSize = (_numPages + 7) / 8;

private:
  /** The actual data bitmap. Declared before `_store`, which references it. */
  uint8_t _bits[_pageTableSize];

  /** Crash-safe dual-copy persistence for `_bits`. */
  RedundantPagedStore<hwPageSizeBytes> _store;
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

  /** Create a deadlist object whose redundant copies start at the given two
   * hardware pages. */
  WearLevelDeadList(GenericSpiEeprom &eeprom, addr_t copyAStartPage,
                    addr_t copyBStartPage)
      : Base(eeprom, copyAStartPage, copyBStartPage) {}

  /** Return true if the page identified by pageNum is marked dead and should
   * not be used for storage. */
  bool isPageDead(size_t pageNum) const override {
    return Base::isPageSet(pageNum);
  }

  bool isPageLive(size_t pageNum) const { return !Base::isPageSet(pageNum); }

  /** Mark the specified page as no longer accessible. */
  void markPageDead(size_t pageNum) override { Base::setPageBit(pageNum); }

  /** Mark a contiguous run of `nPages` pages dead in one operation. With
   * delayFlush=true, stages the change without writing (call flush() later). */
  void markSpanDead(size_t startPage, size_t nPages, bool delayFlush = false) {
    Base::setSpan(startPage, nPages, delayFlush);
  }
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

  /** Create a page usage object whose redundant copies start at the given two
   * hardware pages. */
  PageUsageBitmap(GenericSpiEeprom &eeprom, addr_t copyAStartPage,
                  addr_t copyBStartPage)
      : Base(eeprom, copyAStartPage, copyBStartPage) {}

  /** Return true if the page identified by pageNum is marked in-use and should
   * not be used for storage of a new object. */
  bool isPageInUse(size_t pageNum) const { return Base::isPageSet(pageNum); }

  bool isPageFree(size_t pageNum) const { return !Base::isPageSet(pageNum); }

  /** Mark the specified page as allocated for storing a record. */
  void markPageInUse(size_t pageNum) { Base::setPageBit(pageNum); }

  /** Mark the specified page as free; no longer storing a record. */
  void markPageFree(size_t pageNum) { Base::clearPageBit(pageNum); }

  /** Mark a contiguous run of `nPages` pages in-use in one operation. */
  void markSpanInUse(size_t startPage, size_t nPages, bool delayFlush = false) {
    Base::setSpan(startPage, nPages, delayFlush);
  }

  /** Mark a contiguous run of `nPages` pages free in one operation. */
  void markSpanFree(size_t startPage, size_t nPages, bool delayFlush = false) {
    Base::clearSpan(startPage, nPages, delayFlush);
  }

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
 * Interface for a record whose location can be tracked by a `DataMap`. The
 * DataMap persists the hardware page at which each record's data begins; it
 * does not persist the record's size.
 */
class DataMappable {
public:
  using addr_t = uint32_t;

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

  /** Confirm the record's currently stored copy passes its integrity check,
   * without disturbing the in-RAM `data`. Used by diagnostics
   * (EepromPageManager::verifyAll()) and the slow-write advisory path. */
  virtual bool verify() const = 0;
};

/**
 * Tracks, for each of n `DataMappable` records, the hardware page at which
 * that record's data currently begins.
 *
 * The start-page array (n entries of `sizeof(addr_t)` bytes, indexed by record
 * id) is persisted crash-safely and in two redundant copies via a
 * `RedundantPagedStore`: copy A begins at hardware page `mapAStartPageNum`,
 * copy B at `mapBStartPageNum`. Each serialized page carries its own seqId
 * preamble and CRC32 trailer (the engine versions every page independently, so
 * a torn update or a power loss leaves the previous good copy of every entry
 * intact). The size of each record (`sizeInPages()`) is not itself persisted;
 * it is purely an in-program convenience available to the records' allocator.
 */
template <size_t hwPageSizeBytes> class DataMap {
public:
  using addr_t = uint32_t;

  /**
   * Create a DataMap tracking `numRecords` records, with its two redundant
   * copies serialized starting at hardware pages `mapAStartPageNum` (A) and
   * `mapBStartPageNum` (B).
   */
  DataMap(GenericSpiEeprom &eeprom, addr_t mapAStartPageNum,
          addr_t mapBStartPageNum, size_t numRecords)
      : _numRecords(numRecords),
        _startPages(new addr_t[numRecords ? numRecords : 1]),
        _store(eeprom, reinterpret_cast<uint8_t *>(_startPages),
               numRecords * sizeof(addr_t), mapAStartPageNum,
               mapBStartPageNum) {
    memset(_startPages, 0, _numRecords * sizeof(addr_t));
  }

  ~DataMap() { delete[] _startPages; }

  /**
   * Set up the DataMap by loading the serialized start-page array from the
   * EEPROM, taking each page from its newest valid redundant copy.
   *
   * Returns false (still loading as much as it can) if some page had no valid
   * copy at all, so the caller can react to detected corruption instead of
   * silently trusting it.
   */
  bool init() { return _store.load(); }

  /** Number of hardware pages needed to serialize the backing array (per
   * copy). */
  size_t numMapPages() const { return _store.numPages(); }

  /** Return the page number at which the record identified by `recordId`
   * currently starts, as stored in the backing array. */
  addr_t getStartPage(addr_t recordId) const { return _startPages[recordId]; }

  /** Set, in memory only, the start page for the record identified by
   * `recordId`, marking the affected serialized page(s) dirty. Call flush() to
   * persist. */
  void setStartPage(addr_t recordId, addr_t startPage) {
    _startPages[recordId] = startPage;
    size_t firstByteIdx = recordId * sizeof(addr_t);
    _store.markDirtyByteRange(firstByteIdx, firstByteIdx + sizeof(addr_t) - 1);
  }

  /** Write only the dirty serialized page(s) back to the EEPROM (each to its
   * inactive redundant slot). This is the durability barrier for a record
   * relocation. */
  bool flush() { return _store.flushDirty(); }

  /** Write both redundant copies in full from the current array (used during
   * formatting so both sites are preloaded for later single-page updates). */
  void formatBoth() { _store.formatBoth(); }

  /** Confirm both copies validate without disturbing the in-RAM array. */
  bool verify() const { return _store.verify(); }

private:
  /**
   * Number of records whose page offsets are tracked (length of _startPages
   * backing array).
   */
  const size_t _numRecords;

  /** In-memory copy of the start-page array, indexed by record id. Declared
   * before `_store`, which references it. */
  addr_t *_startPages;

  /** Crash-safe dual-copy persistence for `_startPages`. */
  RedundantPagedStore<hwPageSizeBytes> _store;
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
  using addr_t = uint32_t;

  /** The page this record is stored at starts out zeroed; a `DataMap` (or an
   * `EepromPageManager`) is responsible for assigning the real page via
   * setPageNum() before this is used for storage. */
  PageBackedData(addr_t recordId, GenericSpiEeprom &eeprom)
      : data(), recordId(recordId), _pageNum(0), _eeprom(eeprom) {}
  PageBackedData(const T &initial, addr_t recordId, GenericSpiEeprom &eeprom)
      : data(initial), recordId(recordId), _pageNum(0), _eeprom(eeprom) {}

  /** PageBackedData always occupies exactly one hardware page. */
  size_t sizeInPages() const override { return 1; }

  /** This record's unique id within its DataMap. */
  addr_t getRecordId() const override { return recordId; }

  /** The hardware page this record is currently stored at. */
  addr_t getPageNum() const override { return _pageNum; }

  /** Reassign the hardware page this record is stored at (e.g. when relocated
   * by a wear-leveling allocator). Does not move the stored data; call
   * store() afterward to write `data` to the new page. */
  void setPageNum(addr_t newPageNum) override { _pageNum = newPageNum; }

  /**
   * Load the stored copy of `data` from the current page. Verifies the
   * CRC32 trailer; if it does not match, returns false and leaves `data`
   * unmodified.
   */
  bool load() override {
    uint8_t buf[sizeof(T) + sizeof(uint32_t)];
    size_t n = _eeprom.read(buf, _byteAddr(), sizeof(buf));
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
  }

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

    size_t written = _eeprom.write(buf, _byteAddr(), sizeof(buf));
    if (written != sizeof(buf)) {
      return false;
    }

    uint8_t readBack[sizeof(T) + sizeof(uint32_t)];
    size_t readCount = _eeprom.read(readBack, _byteAddr(), sizeof(readBack));
    if (readCount != sizeof(readBack)) {
      return false;
    }
    if (memcmp(buf, readBack, sizeof(readBack)) != 0) {
      return false;
    }

    uint32_t readBackCrc;
    memcpy(&readBackCrc, readBack + sizeof(T), sizeof(uint32_t));
    return computeCrc32(readBack, sizeof(T)) == readBackCrc;
  }

  /** Confirm the stored copy's CRC32 without modifying `data`. */
  bool verify() const override {
    uint8_t buf[sizeof(T) + sizeof(uint32_t)];
    if (_eeprom.read(buf, _byteAddr(), sizeof(buf)) != sizeof(buf)) {
      return false;
    }
    uint32_t storedCrc;
    memcpy(&storedCrc, buf + sizeof(T), sizeof(uint32_t));
    return computeCrc32(buf, sizeof(T)) == storedCrc;
  }

  /** Local copy of the EEPROM-backed data. */
  T data;

  /** This record's unique id within its DataMap. */
  const addr_t recordId;

private:
  eeprom_addr_t _byteAddr() const {
    return static_cast<eeprom_addr_t>(_pageNum * hwPageSizeBytes);
  }

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
 * post-wrap values). That case is detected by seeing any writeSequenceId
 * within `k` of UINT32_MAX (the only pre-wrap values that can still be live)
 * alongside a small post-wrap value, and resolved by considering only
 * candidates whose writeSequenceId is `<= k` (exactly the slots written since
 * the wrap). Detecting the whole top-`k` band, rather than only the literal
 * UINT32_MAX, keeps the disambiguation correct even when dead-slot skipping
 * means the slots weren't overwritten in strict ring order.
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
  using addr_t = uint32_t;

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
      : data(), recordId(recordId), _basePage(0), _currentSlot(0),
        _currentSeq(0), _eeprom(eeprom), _deadList(nullptr) {}
  WearLevelingPageData(const T &initial, addr_t recordId,
                       GenericSpiEeprom &eeprom)
      : data(initial), recordId(recordId), _basePage(0), _currentSlot(0),
        _currentSeq(0), _eeprom(eeprom), _deadList(nullptr) {}

  /** A WearLevelingPageData record occupies its entire ring of `k` pages. */
  size_t sizeInPages() const override { return k; }

  /** This record's unique id within its DataMap. */
  addr_t getRecordId() const override { return recordId; }

  /** Bind (or rebind) the deadlist this ring consults before writing to a
   * slot, and updates when a slot's write fails verification. Typically
   * called once, right after constructing the `EepromPageManager` that owns
   * the deadlist (see the constructor comment for why this can't happen at
   * construction time instead). */
  void setDeadPageOracle(DeadPageOracle &deadList) { _deadList = &deadList; }

  /** The base hardware page of the ring (i.e. its first of `k` pages). */
  addr_t getPageNum() const override { return _basePage; }

  /** Reassign the ring's base page (e.g. when relocated wholesale by
   * `EepromPageManager::storeRecord()` after too many of this ring's slots
   * went dead). Forgets any previously known slot/sequence, since a new
   * ring's history starts fresh; call store() afterward to write `data`
   * there for the first time. */
  void setPageNum(addr_t newBasePage) override {
    _basePage = newBasePage;
    _currentSlot = 0;
    _currentSeq = 0;
  }

  /** Which of the `k` slots currently holds the most-recently-written data,
   * per the last successful load() or store(). Meaningless until one of
   * those has succeeded at least once. */
  size_t currentSlot() const { return _currentSlot; }

  /** The hardware page backing `currentSlot()`. */
  addr_t currentPageNum() const {
    return static_cast<addr_t>(_basePage + _currentSlot);
  }

  /** The writeSequenceId last read (via load()) or written (via store())
   * into the current slot, or 0 if no valid record has been found yet. */
  uint32_t currentWriteSeqId() const { return _currentSeq; }

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
    bool sawHigh = false;
    bool sawLow = false;

    for (size_t i = 0; i < k; i++) {
      uint8_t buf[_slotSize];
      validSlot[i] = _readSlot(i, buf) && _crcValid(buf);
      if (!validSlot[i]) {
        continue;
      }

      memcpy(&rawSeq[i], buf, sizeof(uint32_t));
      // A live pre-wrap value can only be within the top-k band: by
      // construction the last k writes before the wrap were UINT32_MAX
      // down to UINT32_MAX-k+1.
      if (rawSeq[i] > UINT32_MAX - k) {
        sawHigh = true;
      }
      if (rawSeq[i] <= k) {
        sawLow = true;
      }
    }

    bool wrapped = sawHigh && sawLow;
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
  }

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

    if (_deadList == nullptr) {
      // Write fails because the page deadlist was not initialized first.
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
      if (_deadList->isPageDead(_basePage + slot)) {
        continue;
      }

      if (_writeSlot(slot, nextSeq)) {
        _currentSlot = slot;
        _currentSeq = nextSeq;
        return true;
      }

      // If we get here, the write failed. Mark the page as dead and try again,
      // assuming we have sufficient space still alive in which to re-attempt.
      _deadList->markPageDead(_basePage + slot);
      if (_deadSlotCount() * 2 >= k) {
        return false;
      }
    }

    return false;
  }

  /** Confirm at least one slot in the ring holds a CRC-valid record, without
   * disturbing the in-RAM `data`. */
  bool verify() const override {
    for (size_t i = 0; i < k; i++) {
      uint8_t buf[_slotSize];
      if (_readSlot(i, buf) && _crcValid(buf)) {
        return true;
      }
    }
    return false;
  }

  /** Local copy of the EEPROM-backed data. */
  T data;

  /** This record's unique id within its DataMap. */
  const addr_t recordId;

private:
  /** Bytes occupied by one ring slot: writeSequenceId + T + CRC32. */
  static constexpr size_t _slotSize = 2 * sizeof(uint32_t) + sizeof(T);

  /** Byte address of the `slot`'th hardware page in the ring. */
  eeprom_addr_t _pageAddr(size_t slot) const {
    return static_cast<eeprom_addr_t>((_basePage + slot) * hwPageSizeBytes);
  }

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
  }

  /** Read slot `i`'s raw bytes (writeSequenceId + T + CRC32) into `buf`.
   * Returns false if the read was short. */
  bool _readSlot(size_t i, uint8_t buf[_slotSize]) const {
    return _eeprom.read(buf, _pageAddr(i), _slotSize) == _slotSize;
  }

  /** Returns true iff `buf` (a full slot read via `_readSlot()`) has a
   * CRC32 trailer matching its writeSequenceId+T payload. */
  static bool _crcValid(const uint8_t buf[_slotSize]) {
    uint32_t storedCrc;
    memcpy(&storedCrc, buf + sizeof(uint32_t) + sizeof(T), sizeof(uint32_t));
    return computeCrc32(buf, sizeof(uint32_t) + sizeof(T)) == storedCrc;
  }

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

    eeprom_addr_t pageAddr = _pageAddr(slot);
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
  }

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
 * Result of `EepromPageManager::verifyAll()`: a full CRC sweep of every
 * managed structure plus every record's data page(s).
 */
struct EepromVerifyResult {
  bool rootOk;
  bool deadListOk;
  bool usageOk;
  bool dataMapOk;
  size_t numRecords;
  size_t numBadRecords;
  /** True iff all system structures and every record verified. */
  bool ok;
};

/**
 * Top-level coordinator for wear-leveled EEPROM storage.
 *
 * Lays out a fixed set of "system" pages at the start of the EEPROM. Every
 * system structure is stored in two redundant, CRC-protected copies (an "A"
 * generation and a "B" generation) so that a torn write or a power loss can
 * never destroy the last good copy:
 *   - page 0: the root page (`EepromRootPageData`).
 *   - copy A: deadlist, usage list, data map (in that order).
 *   - copy B: deadlist, usage list, data map (in that order).
 * Application data for the supplied `DataMappable` records begins immediately
 * after copy B (see `firstDataPageNum()`).
 *
 * @param bitSize the size of the EEPROM, in bits.
 * @param hwPageSizeBytes the size of one write page for the EEPROM, in bytes.
 */
template <size_t bitSize, size_t hwPageSizeBytes> class EepromPageManager {
  /** Total number of hardware pages in the managed EEPROM. */
  static constexpr size_t _numPages = (bitSize / 8) / hwPageSizeBytes;

  /** Payload bytes per serialized page, after the seqId + CRC32 overhead. */
  static constexpr size_t _payloadPerPage =
      RedundantPagedStore<hwPageSizeBytes>::PAYLOAD_PER_PAGE;

  /**
   * Number of hardware pages needed to serialize a single `EepromPageBitmap`
   * (the deadlist or the usage list) covering all `_numPages` pages of this
   * EEPROM, per redundant copy. For a small EEPROM this is 1, but for a large
   * one it can be several pages. The deadlist and usage list are always the
   * same size as each other, since both cover the same `_numPages`.
   */
  static constexpr size_t _bitmapTableSizeBytes = (_numPages + 7) / 8;
  static constexpr size_t _bitmapNumPages =
      (_bitmapTableSizeBytes + _payloadPerPage - 1) / _payloadPerPage;

  /** Number of serialized pages a data map needs for `n` records, per copy. */
  static constexpr size_t _mapPagesFor(size_t n) {
    return (n * sizeof(addr_t) + _payloadPerPage - 1) / _payloadPerPage;
  }

public:
  using addr_t = uint32_t;

  /**
   * The hardware page numbers at which copy A of each system structure is
   * rooted. Copy B (and the first data page) depend on the data map's size,
   * which depends on the record count, so those are computed at construction.
   */
  static constexpr size_t ROOT_PAGE_NUM = 0;
  static constexpr size_t DEAD_LIST_PAGE_NUM = ROOT_PAGE_NUM + 1;
  static constexpr size_t USAGE_LIST_PAGE_NUM =
      DEAD_LIST_PAGE_NUM + _bitmapNumPages;
  static constexpr size_t DATA_MAP_START_PAGE_NUM =
      USAGE_LIST_PAGE_NUM + _bitmapNumPages;

  /** Magic number proving the root page has been formatted by this system. */
  static constexpr uint32_t ROOT_PAGE_MAGIC = 0x0571AC94;

  /** The page-manager system's own on-disk format version. */
  static constexpr uint32_t SYSTEM_FORMAT_VERSION = 1;

  /** How long (in ms) a single `storeRecord()` write+readback may take before
   * it's treated as a suspiciously slow write, prompting an advisory readback
   * verification (not, by itself, a relocation). The M95256 datasheet
   * specifies a 5ms max write time; this allows generous margin. */
  static constexpr unsigned long WRITE_STALL_THRESHOLD_MILLIS = 20;

  /** Root page is valid: magic, system version, and app version all check out.
   */
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
  /** The data map failed to load (no valid copy of one or more of its pages).
   */
  static constexpr int ERR_DATA_MAP_CORRUPT = -5;
  /** A page bitmap (deadlist or usage list) failed to load. */
  static constexpr int ERR_BITMAP_CORRUPT = -6;

  /**
   * Create a page manager for `numRecords` `DataMappable` records, backed by
   * `eeprom`. `appFormatVersion` is the application's own data format
   * version, persisted to (and checked against) the root page.
   */
  EepromPageManager(GenericSpiEeprom &eeprom, DataMappable *const *records,
                    size_t numRecords, uint32_t appFormatVersion)
      : _eeprom(eeprom),
        _deadList(eeprom, DEAD_LIST_PAGE_NUM, _deadListBPage(numRecords)),
        _usage(eeprom, USAGE_LIST_PAGE_NUM, _usageBPage(numRecords)),
        _dataMap(eeprom, DATA_MAP_START_PAGE_NUM, _dataMapBPage(numRecords),
                 numRecords),
        _rootPage(/*recordId=*/0, eeprom), _records(records),
        _numRecords(numRecords), _appFormatVersion(appFormatVersion),
        _firstDataPageNum(_firstDataPage(numRecords)) {}

  /**
   * Format a brand-new EEPROM. Stages the deadlist, usage list, and data map
   * in RAM, reserving the system pages (root through copy B of the data map)
   * plus an initial run of free pages for each record, then writes both
   * redundant copies of every system structure and finally the root page.
   *
   * The root page (the validity gate read first by init()) is written LAST, so
   * a power loss anywhere during formatting leaves the EEPROM looking
   * unformatted rather than half-formatted-but-trusted.
   *
   * Returns false if there isn't enough free space to place every record; in
   * that case nothing has been committed to disk (the root page was never
   * written), so the device is left in its prior state.
   */
  bool formatNew() {
    // Stage the deadlist (all-live) and usage list in RAM first; nothing is
    // written to the EEPROM until every record has been placed, so a failure
    // to fit leaves the device untouched.
    _deadList.clearAll();
    _usage.clearAll();

    // Reserve all system pages (everything below the first data page) in RAM.
    _usage.markSpanInUse(0, _firstDataPageNum, /*delayFlush=*/true);

    size_t nextSearch = _firstDataPageNum;
    for (size_t i = 0; i < _numRecords; i++) {
      DataMappable *record = _records[i];
      size_t numPages = record->sizeInPages();
      size_t startPage = _findFreePageRun(nextSearch, numPages);
      if (startPage == ERR_NO_FREE_PAGES) {
        return false; // Nothing durable written yet (root still unwritten).
      }

      _usage.markSpanInUse(startPage, numPages, /*delayFlush=*/true);
      _dataMap.setStartPage(record->getRecordId(),
                            static_cast<addr_t>(startPage));
      record->setPageNum(static_cast<addr_t>(startPage));
      nextSearch = startPage + numPages;
    }

    // Write both redundant copies of every system structure from the staged
    // in-RAM state, so each structure has two complete, current copies.
    _deadList.formatBoth();
    _usage.formatBoth();
    _dataMap.formatBoth();

    // Root page last: its validity now implies a complete format.
    _rootPage.data.magic = ROOT_PAGE_MAGIC;
    _rootPage.data.systemVersion = SYSTEM_FORMAT_VERSION;
    _rootPage.data.appVersion = _appFormatVersion;
    memset(_rootPage.data.reserved, 0, sizeof(_rootPage.data.reserved));
    return _rootPage.store();
  }

  /**
   * Load the root page, deadlist, usage list, and data map from the EEPROM.
   * Does *not* load any individual record's data -- callers are responsible
   * for calling load() on whichever records they actually need.
   *
   * Updates each known `DataMappable`'s cached page number from the freshly
   * loaded data map.
   *
   * Returns ROOT_PAGE_OK on success, or the first applicable error code.
   */
  int init() {
    int rootStatus = checkRootPageValid();
    if (rootStatus != ROOT_PAGE_OK) {
      return rootStatus;
    }

    if (!_deadList.init() || !_usage.init()) {
      return ERR_BITMAP_CORRUPT;
    }
    if (!_dataMap.init()) {
      return ERR_DATA_MAP_CORRUPT;
    }

    for (size_t i = 0; i < _numRecords; i++) {
      _records[i]->setPageNum(
          _dataMap.getStartPage(_records[i]->getRecordId()));
    }

    return ROOT_PAGE_OK;
  }

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
  }

  /**
   * Persist `record` to its currently assigned page(s).
   *
   * If the write cannot be verified, the record is relocated (see
   * `_relocateAndStore`). If the write *succeeds* but took implausibly long
   * (> WRITE_STALL_THRESHOLD_MILLIS), that's treated only as an advisory
   * signal to re-read and re-verify the stored copy: a slow-but-correct write
   * (e.g. one delayed by interrupt latency) is kept in place, and only an
   * actually-failed readback triggers relocation.
   *
   * NOTE: this advisory readback is meaningful only because write() today is
   * synchronous (the cell commit has completed by the time it returns). If/when
   * write() becomes asynchronous -- letting the MCU proceed before the
   * low-level flush completes -- a slow write would need its commit confirmed
   * before this verify() could be trusted; revisit this path then.
   *
   * Returns true iff `record` ends up durably (and verifiably) stored
   * somewhere.
   */
  bool storeRecord(DataMappable &record) {
    unsigned long startMillis = millis();
    bool ok = record.store();
    unsigned long elapsedMillis = millis() - startMillis;

    if (!ok) {
      return _relocateAndStore(record);
    }

    if (elapsedMillis > WRITE_STALL_THRESHOLD_MILLIS && !record.verify()) {
      return _relocateAndStore(record);
    }

    return true;
  }

  /**
   * Full integrity sweep: verify the root page, both bitmaps, the data map,
   * and every record's data page(s) via their CRC32s. Call after a successful
   * init() so each record's page number is current. Does not modify any
   * record's in-RAM data.
   */
  EepromVerifyResult verifyAll() {
    EepromVerifyResult r{};
    r.rootOk = (checkRootPageValid() == ROOT_PAGE_OK);
    r.deadListOk = _deadList.verify();
    r.usageOk = _usage.verify();
    r.dataMapOk = _dataMap.verify();
    r.numRecords = _numRecords;
    r.numBadRecords = 0;
    for (size_t i = 0; i < _numRecords; i++) {
      if (!_records[i]->verify()) {
        r.numBadRecords++;
      }
    }
    r.ok = r.rootOk && r.deadListOk && r.usageOk && r.dataMapOk &&
           r.numBadRecords == 0;
    return r;
  }

  /** Direct access to the deadlist, e.g. for tests or diagnostics. */
  WearLevelDeadList<bitSize, hwPageSizeBytes> &deadList() { return _deadList; }

  /** Direct access to the usage (free/used) list, e.g. for tests or
   * diagnostics. */
  PageUsageBitmap<bitSize, hwPageSizeBytes> &usage() { return _usage; }

  /** Direct access to the data map, e.g. for tests or diagnostics. */
  DataMap<hwPageSizeBytes> &dataMap() { return _dataMap; }

  /** The first hardware page available for application data; everything
   * before this is reserved for the root page and the two redundant copies of
   * the deadlist, usage list, and data map. */
  size_t firstDataPageNum() const { return _firstDataPageNum; }

private:
  /** Copy-B start pages, computed from the record count (copy A is the public
   * constexpr layout above; copy B follows copy A's data map). */
  static constexpr size_t _deadListBPage(size_t n) {
    return DATA_MAP_START_PAGE_NUM + _mapPagesFor(n);
  }
  static constexpr size_t _usageBPage(size_t n) {
    return _deadListBPage(n) + _bitmapNumPages;
  }
  static constexpr size_t _dataMapBPage(size_t n) {
    return _usageBPage(n) + _bitmapNumPages;
  }
  static constexpr size_t _firstDataPage(size_t n) {
    return _dataMapBPage(n) + _mapPagesFor(n);
  }

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
        if (p >= _numPages || _usage.isPageInUse(p) ||
            _deadList.isPageDead(p)) {
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
  }

  /**
   * Relocate `record` to a fresh run of pages after a failed store(), in a
   * crash-safe order:
   *   1. find and reserve a fresh, live run (committed to the usage list
   *      *before* any data is written there);
   *   2. write and verify the record at the new location;
   *   3. only then repoint the data map (the durability barrier -- a power
   *      loss before this leaves the old, still-referenced copy authoritative);
   *   4. finally retire the old run by marking it dead.
   *
   * The old run is deliberately *not* freed: a page that failed a verified
   * write must never be reallocated, so leaving it in-use *and* dead removes
   * it from the pool without spending a write to free it.
   */
  bool _relocateAndStore(DataMappable &record) {
    size_t numPages = record.sizeInPages();
    addr_t oldPageNum = record.getPageNum();

    size_t newPageNum = _findFreePageRun(_firstDataPageNum, numPages);
    if (newPageNum == ERR_NO_FREE_PAGES) {
      // Nowhere to go. Retire the old run so we stop trusting/hammering it;
      // the data map still points there, so any still-readable old copy
      // remains reachable.
      _deadList.markSpanDead(oldPageNum, numPages);
      return false;
    }

    // Reserve the new run first (durable before we write data into it).
    _usage.markSpanInUse(newPageNum, numPages);

    record.setPageNum(static_cast<addr_t>(newPageNum));
    if (!record.store()) {
      // The new location is bad too. Retire it (leave it in-use so it's never
      // reused) and fall back to the old page, which the map still references.
      _deadList.markSpanDead(newPageNum, numPages);
      record.setPageNum(oldPageNum);
      return false;
    }

    // Commit: repoint the map at the verified new copy. This is the barrier.
    _dataMap.setStartPage(record.getRecordId(),
                          static_cast<addr_t>(newPageNum));
    if (!_dataMap.flush()) {
      // The commit itself couldn't be verified; the redundant map preserved
      // the old pointer on disk, so fall back to the old location. Roll the
      // map's in-RAM entry back to match (and mark it dirty again), so a later
      // flush can't push this abandoned new pointer to disk -- flushDirty()
      // drops the dirty flag on failure, so we can't rely on it to suppress
      // the abandoned write for us.
      _dataMap.setStartPage(record.getRecordId(), oldPageNum);
      record.setPageNum(oldPageNum);
      return false;
    }

    // Retire the old run only after the new copy is durably referenced.
    _deadList.markSpanDead(oldPageNum, numPages);
    return true;
  }

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
