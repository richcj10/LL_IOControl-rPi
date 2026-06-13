#pragma once
#include <stdint.h>

// Firmware-update state machine. The host streams a (raw or GZIP) image to
// REG_FW_DATA_PORT in blocks, sets the expected CRC32, then commits; the staged
// image is applied on the next boot by the core's OTA bootloader.
// All flash / LittleFS work happens in FWUpdateUpdate() (the main loop), never
// inside an SPI transaction. See docs/REGISTER_MAP.md §Firmware update.

void FWUpdateInit();          // mount LittleFS, status = IDLE
void FWUpdateUpdate();        // main loop: process a pending command + flush blocks
void FWUpdateFeed(uint8_t b); // SPI handler: push one streamed image byte to the buffer
