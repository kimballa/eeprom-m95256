// (c) Copyright 2024 Aaron Kimball
//
// M95256 SPI EEPROM driver
// See STMicroelectronics M95256-W datasheet for usage.
//
// Instantiate an instance of `EepromM95256` for this specific chip,
// or use `SpiEeprom<sizeInBits, pageSizeInBytes>` for another compatible SPI
// EEPROM IC driver.
//
// You can use the `EepromBacked<T>` class as a convenient way to
// wrap EEPROM-backed data structures with a mechanism to retrieve
// and persist the data to the IC at a fixed address.

#ifndef _EEPROM_M95256_LIB_H
#define _EEPROM_M95256_LIB_H

#include <Arduino.h>
#include <SPI.h>

// bitmasks for status register decoding

constexpr uint8_t EEPROM_STATUS_REG_WRITE_PROTECT = 0xF0;
constexpr uint8_t EEPROM_STATUS_REG_WRITE_IN_PROGRESS = 0x01;
constexpr uint8_t EEPROM_STATUS_REG_WRITE_ENABLE_LATCH = 0x02;
constexpr uint8_t EEPROM_STATUS_REG_BP0 = 0x04; // block protect bit 0
constexpr uint8_t EEPROM_STATUS_REG_BP1 = 0x08; // block protect bit 1

/** Return true if v is a power of 2. */
inline constexpr bool isPowerOf2(int v) { return v && ((v & (v - 1)) == 0); }

// Flag available for IRQs to set to enforce EEPROM stops writing data.
// (e.g., on brownout detection.)
extern volatile bool forceSuppressEepromWrite;

/** Set the EEPROM write lock to true. Immediately aborts in-progress writes and
 * prevents future writes. */
inline void setEepromWriteLock() { forceSuppressEepromWrite = true; }

/**
 * Base class for SpiEeprom, not specialized to a particular storage size.
 *
 * @see SpiEeprom for the implementation.
 */
class GenericSpiEeprom {
public:
  using addr_t = uint16_t;
  virtual size_t read(void *buf, addr_t addr, size_t count) = 0;
  virtual size_t write(void *buf, addr_t addr, size_t count) = 0;
};

/**
 * A container for fixed-size data stored at a particular address in an EEPROM.
 *
 * This can be initialized with an optional starter copy of the data, along with
 * a reference to the EEPROM and the address where the backing copy is stored.
 *
 * The load() function will update the local copy from the EEPROM, and store()
 * will persist local changes back to the EEPROM storage.
 */
template <typename T> class EepromBacked {
public:
  using addr_t = uint16_t;

  EepromBacked(addr_t eepromAddr, GenericSpiEeprom &eeprom)
      : data(), addr(eepromAddr), _eeprom(eeprom){};
  EepromBacked(const T &initial, addr_t eepromAddr, GenericSpiEeprom &eeprom)
      : data(initial), addr(eepromAddr), _eeprom(eeprom){};
  EepromBacked(T &&initial, addr_t eepromAddr, GenericSpiEeprom &eeprom)
      : data(initial), addr(eepromAddr), _eeprom(eeprom){};
  EepromBacked(const EepromBacked &other)
      : data(other.data), addr(other.addr), _eeprom(other._eeprom){};
  EepromBacked(const EepromBacked &&other)
      : data(other.data), addr(other.addr), _eeprom(other._eeprom){};

  /**
   * Load the stored copy of the data from the EEPROM into this local copy.
   * This will overwrite any unsaved local changes.
   *
   * Returns the number of bytes read.
   */
  size_t load() { return _eeprom.read(&data, addr, sizeof(T)); };

  /**
   * Store this local copy of the data back to the EEPROM.
   *
   * Returns the number of bytes written.
   */
  size_t store() { return _eeprom.write(&data, addr, sizeof(T)); };

  EepromBacked<T> operator=(const T &other) {
    data = other;
    return *this;
  };
  EepromBacked<T> operator=(const T &&other) {
    data = other;
    return *this;
  };
  T &operator()() { return data; };

  /** Local copy of the EEPROM-backed data. */
  T data;

  /** Address within the EEPROM where the data is stored. */
  const addr_t addr;

private:
  /** The EEPROM backing this data. */
  GenericSpiEeprom &_eeprom;
};

/**
 * Generic paged byte-addressible SPI EEPROM driver.
 */
template <size_t bitSize, size_t hwPageSizeBytes>
class SpiEeprom : public GenericSpiEeprom {

  static_assert(isPowerOf2(bitSize), "bitSize must be a power of two.");
  static_assert(isPowerOf2(hwPageSizeBytes),
                "hwPageSizeBytes must be a power of two.");

public:
  using addr_t = uint16_t;

  SpiEeprom(uint8_t csPin)
      : _csPin(csPin),
        _EEPROM_SPI_SETTINGS(_EEPROM_M95256_MAX_CLK, MSBFIRST, SPI_MODE0){};

  /** Initialize SPI bus on the chosen pin set. */
  void setup() {
    // Set up the chip-select pin as an output and assert it HIGH; we are not
    // currently writing to the DAC chip. This pin should always return HIGH
    // when we are not actively transfering data.
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);
  };

  /**
   * Reads 'count' bytes starting at address 'addr' into 'buf'.
   *
   * If the target address plus the data count would exceed the maximum valid
   * address of the EEPROM IC, this will truncate the read at the top of the
   * address space. The returned value will reflect the truncated read count.
   *
   * Returns the number of bytes read.
   */
  virtual size_t read(void *buf, addr_t addr, size_t count) override {
    if (addr > MAX_ADDR) {
      Serial.println("ERROR: Read outside of address space.");
      return 0; // Reject reads outside EEPROM address space.
    }
    // Clamp read size to top of addr space.
    count = min(count, (size_t)(BYTE_SIZE - addr));

    waitForWriteComplete();
    size_t nOut = 0, nIn = 0;
    uint8_t addrBuf[ADDR_LEN];
    addrBuf[0] = (addr >> 8) & 0xFF;
    addrBuf[1] = (addr >> 0) & 0xFF;
    _executeTx(INSTR_READ, addrBuf, NULL, 0, 0, (uint8_t *)buf, 0, count, nOut,
               nIn);
    return nIn;
  };

  /**
   * Writes 'count' bytes from 'buf' to the EEPROM, starting at address 'addr'.
   *
   * This method performs the write operation in a number of distinct
   * transactions, each targeted at a single page. If the starting address and
   * total byte count to write fit within a single page, this is performed in
   * one transaction. Otherwise we will use as many separate transactions as
   * necessary. This method waits for one transaction to commit before beginning
   * the next page-level write transaction.
   *
   * If the target address plus the data count would exceed the maximum valid
   * address of the EEPROM IC, this will truncate the write at the top of the
   * address space. The returned value will reflect the truncated write count.
   *
   * This method does not wait for the final page write transaction to commit.
   * You can use the waitForWriteComplete() method after this method returns
   * to block until that commit happens, or use isWriteInProgress() to
   * interrogate the status of the device in a non-blocking fashion.
   *
   * Returns the number of bytes written.
   */
  virtual size_t write(void *buf, addr_t addr, size_t count) override {
    if (addr > MAX_ADDR) {
      Serial.println("ERROR: Write outside of address space.");
      return 0; // Reject writes outside EEPROM address space.
    }
    // Clamp write size to top of addr space.
    count = min(count, (size_t)(BYTE_SIZE - addr));

    size_t totalWritten = 0, pageWritten = 0, opWritten = 0, readSize = 0;

    size_t countRemaining = count; // Number of bytes remaining to write.
    // Mask to calculate the offset of the starting address inside its page.
    const addr_t OFFSET_MASK = hwPageSizeBytes - 1;
    addr_t offset = addr & OFFSET_MASK; // offset within the first page.
    // number of bytes to write from the current page.
    addr_t thisPageCount = min(countRemaining, hwPageSizeBytes - offset);
    // Address to start the next page-level write tx.
    addr_t thisPageAddr = addr;
    uint8_t addrBuf[ADDR_LEN];

    // We need to write the SPI memory one page at a time, wait for it to
    // commit the page, and then write the next page.
    while (countRemaining > 0) {
      if (forceSuppressEepromWrite) {
        Serial.println(
            "WARNING: EEPROM write-suppression bit set. Canceling write...");
        break;
      }

      // Start by ensuring any previous write tx is complete.
      waitForWriteComplete();

      if (forceSuppressEepromWrite) {
        Serial.println(
            "WARNING: EEPROM write-suppression bit set. Canceling write...");
        break;
      }

      // Unlock the device for writing.
      _executeTx(INSTR_WREN, NULL, NULL, 0, 0, NULL, 0, 0, opWritten, readSize);

      // Write the next page.
      addrBuf[0] = (thisPageAddr >> 8) & 0xFF;
      addrBuf[1] = (thisPageAddr >> 0) & 0xFF;
      _executeTx(INSTR_WRITE, addrBuf, (uint8_t *)buf, totalWritten,
                 thisPageCount, NULL, 0, 0, pageWritten, readSize);

      totalWritten += pageWritten;
      if (pageWritten != thisPageCount) {
        // We did not completely write the page. Abort!
        Serial.printf("EEPROM Write Error: Expected to write %d bytes at addr "
                      "0x%04X, actual write count = %d\n",
                      thisPageCount, thisPageAddr, pageWritten);
        break;
      }

      // Set up for the next page.
      countRemaining -= thisPageCount;
      thisPageAddr += thisPageCount;
      thisPageCount = min(countRemaining, hwPageSizeBytes);
    }

    // We don't actually need to wait until the last write tx is complete before
    // returning.
    return totalWritten;
  };

  /** Reads and returns the contents of the EEPROM status register. */
  uint8_t readStatusRegister() const {
    size_t nOut = 0, nIn = 0;
    uint8_t statusReg;
    _executeTx(INSTR_RDSR, NULL, NULL, 0, 0, &statusReg, 0, 1, nOut, nIn);
    return statusReg;
  };

  /** Returns true if there is a pending write operation. */
  bool isWriteInProgress() const {
    return (readStatusRegister() & EEPROM_STATUS_REG_WRITE_IN_PROGRESS) != 0;
  };

  /** Blocks until there is no pending WRITE operation. */
  void waitForWriteComplete() const {
    while (isWriteInProgress()) {
      // Write operation is 5ms max, check for completion every 1ms.
      delayMicroseconds(1000);
    }
  };

  size_t getSizeBytes() const { return BYTE_SIZE; };
  size_t getPageSize() const { return hwPageSizeBytes; };

private:
  const uint8_t _csPin;

  /**
   * Execute an SPI transaction.
   *
   * @param opcode the operation to execute.
   * @param addrBuf a buffer of size ADDR_LEN containing the address to read or
   *   write. May be NULL if this operation does not require an address.
   * @param sendBuffer a buffer of octets to send as data after the opcode, may
   *   be NULL if no write is required. The send buffer must be large enough
   *   such that sendBuffer[sendOffset + sendMaxCt - 1] is a valid address.
   * @param sendOffset start offset in the send buffer.
   * @param sendMaxCt number of bytes (starting at 'sendOffset') to write.
   * @param recvBuffer a buffer to fill with octets that have been read back
   *   from the SPI bus, may be NULL if no read is required. The receive buffer
   *   must be large enough such that recvBuffer[sendOffset + sendMaxCt - 1]
   *   is a valid address.
   * @param recvOffset start offset in the read buffer
   * @param recvMaxCt number of bytes (starting at 'readOffset') to fill
   * @param numWritten returns the number of bytes written from sendBuffer.
   * @param numReceived returns the number of bytes read into recvBuffer.
   */
  void _executeTx(uint8_t opcode, const uint8_t *addrBuf,
                  const uint8_t *sendBuffer, size_t sendOffset,
                  size_t sendMaxCt, uint8_t *recvBuffer, size_t recvOffset,
                  size_t recvMaxCt, size_t &numWritten,
                  size_t &numReceived) const {

    numWritten = 0;
    numReceived = 0;
    SPI.beginTransaction(_EEPROM_SPI_SETTINGS);
    delayNanoseconds(50);
    digitalWrite(this->_csPin, LOW);
    delayNanoseconds(50);

    SPI.transfer(opcode);

    if (NULL != addrBuf) {
      for (size_t i = 0; i < ADDR_LEN; i++) {
        SPI.transfer(addrBuf[i]);
      }
    }

    size_t transferLen = max(sendMaxCt, recvMaxCt);
    for (size_t i = 0; i < transferLen; i++) {
      bool sendByte = sendBuffer != NULL && sendMaxCt > i;
      uint8_t byteOut = sendByte ? sendBuffer[sendOffset + i] : 0;
      uint8_t byteIn = SPI.transfer(byteOut);
      bool recvByte = recvBuffer != NULL && recvMaxCt > i;

      if (sendByte) {
        numWritten++;
      }

      if (recvByte) {
        recvBuffer[recvOffset + i] = byteIn;
        numReceived++;
      }
    }

    delayNanoseconds(50);
    digitalWrite(this->_csPin, HIGH);
    delayNanoseconds(50);
    SPI.endTransaction();
  };

  /* Constants internal to EEPROM protocol implementation details: */

  const SPISettings _EEPROM_SPI_SETTINGS;

  static constexpr uint8_t INSTR_WREN = 0x06;  // Write enable
  static constexpr uint8_t INSTR_WRDI = 0x04;  // Write disable
  static constexpr uint8_t INSTR_RDSR = 0x05;  // Read status reg
  static constexpr uint8_t INSTR_WRSR = 0x01;  // Write status reg
  static constexpr uint8_t INSTR_READ = 0x03;  // Read memory
  static constexpr uint8_t INSTR_WRITE = 0x02; // Write memory
  static constexpr uint8_t INSTR_RDID = 0x83;  // Read identification page
  static constexpr uint8_t INSTR_WRID = 0x82;  // Write identification page
  static constexpr uint8_t INSTR_RDLS =
      0x83; // Read identification page lock status
  static constexpr uint8_t INSTR_WRLS =
      0x82; // Write identification page lock status

  // Note: RDLS/WRLS use a specific 'address' within the identification page.
  // RDID/WRID/RDLS/WRLS only valid for M95256-D device.

  // TODO(aaron): Support WRSR, RDID, WRIS, RDLS, WRLS operations.
  // TODO(aaron): Do we need to support / use WRDI directly? (Probably not...)

  static constexpr size_t BYTE_SIZE = bitSize / 8;
  /** Lowest addressable byte of memory in the EEPROM. */
  static constexpr uint16_t MIN_ADDR = 0x0000;
  /** Highest addressable byte of memory in the EEPROM. */
  static constexpr uint16_t MAX_ADDR = BYTE_SIZE - 1;
  /** Number of bytes in an address word. */
  static constexpr size_t ADDR_LEN = sizeof(addr_t);

  // Use up to 10 MHz clock on 3.3V VDDIO. (20 MHz ok at 5V.)
  static constexpr uint32_t _EEPROM_M95256_MAX_CLK = 10000000;
};

/** STM M95256 256kbit EEPROM. */
class EepromM95256 : public SpiEeprom<256 * 1024, 64> {
public:
  EepromM95256(uint8_t csPin) : SpiEeprom(csPin){};
};

#endif /* _EEPROM_M95256_LIB_H */
