// (c) Copyright 2024 Aaron Kimball

// Set to true to forcibly abort EEPROM writes. This will
// be checked before every per-page write operation, so it
// can be used e.g. by a brownout detection IRQ to notify
// any in-progress EEPROM operation to cease.
volatile bool forceSuppressEepromWrite = false;

// Raised by the async store/verify path when a write appears stalled (WIP stuck
// on past the stall threshold). See eeprom-m95256.h for the contract.
volatile bool isEepromWriteStalled = false;
