# (c) Copyright 2025 Aaron Kimball

lib_name := eeprom-m95256
libs := Wire
src_dirs := src 

include ../arduino-makefile/arduino.mk

# Desktop (x86_64) unit test suite -- runs against a doctest harness with
# stubbed-out Arduino.h/SPI.h, not against the Arduino toolchain above.
TEST_DIR := test
TEST_BUILD_DIR := $(TEST_DIR)/build
TEST_BIN := $(TEST_BUILD_DIR)/run_tests
TEST_SRCS := $(wildcard $(TEST_DIR)/*.cpp) src/eeprom-m95256.cpp
TEST_CXX := g++
TEST_CXXFLAGS := -std=gnu++20 -Wall -Wextra -g -fsanitize=address,undefined \
	-I$(TEST_DIR)/harness -Isrc

.PHONY: test
test:
	@mkdir -p $(TEST_BUILD_DIR)
	$(TEST_CXX) $(TEST_CXXFLAGS) $(TEST_SRCS) -o $(TEST_BIN)
	$(TEST_BIN)
