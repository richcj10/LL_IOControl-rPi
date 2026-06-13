#pragma once
#include <stdint.h>

constexpr uint8_t FW_VERSION_MAJ = 0;
constexpr uint8_t FW_VERSION_MIN = 1;
constexpr uint8_t DEVICE_ID_VAL  = 0xA1;

// SPI command byte: bit7 = R/W, bits 6:0 = register address.
// NOTE: only addresses 0x00-0x7F are reachable over the wire.
constexpr uint8_t SPI_CMD_READ  = 0x80;
constexpr uint8_t SPI_CMD_WRITE = 0x00;
constexpr uint8_t SPI_ADDR_MASK = 0x7F;

// System 0x00–0x0F
constexpr uint8_t REG_DEVICE_ID  = 0x00;
constexpr uint8_t REG_FW_VER_MAJ = 0x01;
constexpr uint8_t REG_FW_VER_MIN = 0x02;
constexpr uint8_t REG_STATUS     = 0x03;
constexpr uint8_t REG_INT_FLAGS  = 0x04; ///< Read-only, clear-on-read. Bits 3:0 = DI change flags.
constexpr uint8_t REG_INT_MASK   = 0x05; ///< Gates the INT pin only; flags still accumulate when masked.
constexpr uint8_t REG_ERROR      = 0x06; ///< Read-only, clear-on-read. See ERR_* bits below.

/// @name REG_ERROR bits (clear-on-read)
/// @{
constexpr uint8_t ERR_RO_WRITE       = 0x01; ///< Write attempted to a read-only register
constexpr uint8_t ERR_PWM_FREQ_RANGE = 0x02; ///< PWM frequency 0 or below ~7.46 Hz floor; clamped/ignored
constexpr uint8_t ERR_SPI_ABORT      = 0x04; ///< CS deasserted mid-transaction
constexpr uint8_t ERR_SPI_TIMEOUT    = 0x08; ///< SPI clock stalled mid-transaction
constexpr uint8_t ERR_SPI_DESYNC     = 0x10; ///< Stale FIFO bytes flushed at idle (aborted/extra frame)
constexpr uint8_t ERR_DI_MODE        = 0x20; ///< Invalid input-mode config (e.g. only one pin of an encoder pair)
constexpr uint8_t ERR_FW             = 0x40; ///< Firmware-update failure (FS mount, buffer overflow, bad sequence)
/// @}

// Digital IO 0x10–0x1F
constexpr uint8_t REG_DI_STATE    = 0x10;
constexpr uint8_t REG_DI_DEBOUNCE = 0x11;
constexpr uint8_t REG_OUT_MODE    = 0x12;
constexpr uint8_t REG_OUT_VAL_0   = 0x13;
constexpr uint8_t REG_OUT_VAL_1   = 0x14;
constexpr uint8_t REG_OUT_VAL_2   = 0x15;
constexpr uint8_t REG_OUT_VAL_3   = 0x16;
constexpr uint8_t REG_OUT_VAL_4   = 0x17;
constexpr uint8_t REG_OUT_VAL_5   = 0x18;
constexpr uint8_t REG_OUT_VAL_6   = 0x19;

/// Per-input mode select, 2 bits per input (inputs 0..3 = GP10/GP11/GP12/GP16).
/// Encoder mode consumes a pair: enc0 = inputs {0=A,1=B}, enc1 = inputs {2=A,3=B}.
/// Both pins of a pair must be set to ENCODER or ERR_DI_MODE is flagged.
constexpr uint8_t REG_DI_MODE     = 0x1A;
constexpr uint8_t DI_MODE_LEVEL   = 0; ///< Debounced on/off → REG_DI_STATE (default)
constexpr uint8_t DI_MODE_ENCODER = 1; ///< Quadrature A/B → signed 32-bit position
constexpr uint8_t DI_MODE_FREQ    = 2; ///< Edge frequency → 32-bit Hz value

/// Edge select for FREQUENCY mode, 2 bits per input: 0=rising, 1=falling, 2=both.
/// Ignored for LEVEL and ENCODER (encoder always decodes every A/B edge, x4).
constexpr uint8_t REG_DI_EDGE_CFG = 0x1B;
constexpr uint8_t DI_EDGE_RISING  = 0;
constexpr uint8_t DI_EDGE_FALLING = 1;
constexpr uint8_t DI_EDGE_BOTH    = 2;

/// Write a bitmask (bit n = input n) to zero the matching counter/position.
/// Self-clearing: reads back 0 once the main loop has applied it.
constexpr uint8_t REG_CNT_CLEAR   = 0x1C;

/// Frequency measurement window in 10 ms units (default 100 = 1 s). 0 → 1 s.
constexpr uint8_t REG_FREQ_WINDOW = 0x1D;

// ADC 0x20–0x3F
// Read the L byte first: it latches the matching H byte so the 16-bit value
// cannot tear if an ADC update lands between the two transactions.
constexpr uint8_t REG_ADC0_L    = 0x20;
constexpr uint8_t REG_ADC0_H    = 0x21;
constexpr uint8_t REG_ADC1_L    = 0x22;
constexpr uint8_t REG_ADC1_H    = 0x23;
constexpr uint8_t REG_ADC2_L    = 0x24;
constexpr uint8_t REG_ADC2_H    = 0x25;
constexpr uint8_t REG_ADC3_L    = 0x26;
constexpr uint8_t REG_ADC3_H    = 0x27;

/// IIR filter coefficient per channel, 0-255 mapped to alpha 0.0-1.0.
/// @warning Alpha = 0 freezes the filter: the channel's output registers hold
///          their last value indefinitely and never track the input.
///          Alpha = 255 is raw pass-through (no filtering).
constexpr uint8_t REG_ADC0_ALPHA = 0x30;
constexpr uint8_t REG_ADC1_ALPHA = 0x31;
constexpr uint8_t REG_ADC2_ALPHA = 0x32;
constexpr uint8_t REG_ADC3_ALPHA = 0x33;

// PWM frequency per slice 0x40–0x47
// Slice 0 = GP0/GP1, Slice 1 = GP2/GP3, Slice 2 = GP4/GP5, Slice 3 = GP6
// Write the L byte first, then the H byte: the frequency is latched and
// applied on the H-byte write, so a half-written value is never used.
constexpr uint8_t REG_PWM_FREQ0_L = 0x40;
constexpr uint8_t REG_PWM_FREQ0_H = 0x41;
constexpr uint8_t REG_PWM_FREQ1_L = 0x42;
constexpr uint8_t REG_PWM_FREQ1_H = 0x43;
constexpr uint8_t REG_PWM_FREQ2_L = 0x44;
constexpr uint8_t REG_PWM_FREQ2_H = 0x45;
constexpr uint8_t REG_PWM_FREQ3_L = 0x46;
constexpr uint8_t REG_PWM_FREQ3_H = 0x47;

// Smart-input value block 0x50–0x5F: one signed 32-bit value per input,
// little-endian (byte 0 = LSB). Holds encoder position (ENCODER mode) or the
// last measured frequency in Hz (FREQUENCY mode); reads 0 in LEVEL mode.
// Read byte 0 first: it latches bytes 1–3 so a 32-bit value can't tear across
// the four single-byte transactions (same scheme as the ADC registers).
constexpr uint8_t REG_CNT0_0 = 0x50; ///< Input 0, bytes 0x50–0x53
constexpr uint8_t REG_CNT1_0 = 0x54; ///< Input 1, bytes 0x54–0x57
constexpr uint8_t REG_CNT2_0 = 0x58; ///< Input 2, bytes 0x58–0x5B
constexpr uint8_t REG_CNT3_0 = 0x5C; ///< Input 3, bytes 0x5C–0x5F
constexpr uint8_t REG_CNT_END = 0x5F;

// Firmware update 0x60–0x73
// The host streams a (raw or GZIP) image to REG_FW_DATA_PORT in blocks, sets the
// expected CRC32, then commits. The staged image is applied on the next boot by
// the core's always-present OTA bootloader. See docs/REGISTER_MAP.md §Firmware update.
constexpr uint8_t REG_FW_CMD         = 0x60; ///< Write a FW_CMD_* value to drive the update
constexpr uint8_t REG_FW_STATUS      = 0x61; ///< Read-only FW_ST_* state
constexpr uint8_t REG_FW_BLOCK_ADDR0 = 0x62; ///< Host bookkeeping (informational); image offset, LE
constexpr uint8_t REG_FW_BLOCK_ADDR1 = 0x63;
constexpr uint8_t REG_FW_BLOCK_ADDR2 = 0x64;
constexpr uint8_t REG_FW_BLOCK_ADDR3 = 0x65;
constexpr uint8_t REG_FW_BLOCK_SIZE0 = 0x66; ///< Host bookkeeping (informational); block length, LE
constexpr uint8_t REG_FW_BLOCK_SIZE1 = 0x67;
constexpr uint8_t REG_FW_DATA_PORT   = 0x68; ///< Stream image bytes here (burst write, non-incrementing)
constexpr uint8_t REG_FW_CRC32_0     = 0x70; ///< Expected CRC32 of the streamed image, LE
constexpr uint8_t REG_FW_CRC32_1     = 0x71;
constexpr uint8_t REG_FW_CRC32_2     = 0x72;
constexpr uint8_t REG_FW_CRC32_3     = 0x73;

/// @name REG_FW_CMD command values
/// @{
constexpr uint8_t FW_CMD_BEGIN  = 0x01; ///< Open/truncate staging, reset CRC, suspend I/O → READY
constexpr uint8_t FW_CMD_END    = 0x02; ///< Flush, close, compare CRC → VERIFY_OK / VERIFY_FAIL
constexpr uint8_t FW_CMD_COMMIT = 0x03; ///< If VERIFY_OK: schedule OTA and reboot
constexpr uint8_t FW_CMD_ABORT  = 0x04; ///< Discard staging, restore I/O → IDLE
/// @}

/// @name REG_FW_STATUS state values
/// @{
constexpr uint8_t FW_ST_IDLE        = 0x00; ///< No update session
constexpr uint8_t FW_ST_READY       = 0x01; ///< Ready to receive a block / between blocks
constexpr uint8_t FW_ST_BUSY        = 0x02; ///< Flushing a block to flash
constexpr uint8_t FW_ST_VERIFY_OK   = 0x03; ///< Streamed CRC matched; ready to COMMIT
constexpr uint8_t FW_ST_VERIFY_FAIL = 0x04; ///< CRC mismatch
constexpr uint8_t FW_ST_ERROR       = 0x05; ///< FS / overflow / sequence error (see REG_ERROR)
/// @}
