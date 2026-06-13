#include "SPISlave.h"
#include "Registers.h"
#include "FWUpdate.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <string.h>

#define PIN_MOSI  8
#define PIN_CS    9
#define PIN_SCK  14
#define PIN_MISO 15
#define PIN_INT  13

// Max time to wait for SCK activity mid-transaction before declaring the bus dead.
// Bounds every wait loop so a stuck CS or silent master can never hang the device.
#define SPI_BYTE_TIMEOUT_US 1000u

volatile uint8_t regMap[256];
volatile uint8_t pwmFreqPending = 0;
volatile uint8_t diReconfigPending = 0;
volatile uint8_t cntClearPending = 0;
volatile uint8_t fwCmdPending = 0;

// H bytes latched on the matching L-byte read so 16-bit ADC values can't tear
static uint8_t adcLatchH[4];
// Upper 3 bytes latched on the byte-0 read so 32-bit counters can't tear
static uint8_t cntLatch[4][3];
// Base address of the most recent multi-byte latch. A follow-on byte returns
// the latch only if it continues this base, so a standalone / out-of-order
// high-byte read returns live data instead of a stale latch.
static uint8_t lastLatchBase = 0xFF;

static bool isReadOnly(uint8_t addr) {
    if (addr == REG_DEVICE_ID)  return true;
    if (addr == REG_FW_VER_MAJ) return true;
    if (addr == REG_FW_VER_MIN) return true;
    if (addr == REG_STATUS)     return true;
    if (addr == REG_INT_FLAGS)  return true;
    if (addr == REG_ERROR)      return true;
    if (addr == REG_DI_STATE)   return true;
    if (addr >= REG_ADC0_L && addr <= REG_ADC3_H) return true;
    if (addr >= REG_CNT0_0 && addr <= REG_CNT_END) return true;
    if (addr == REG_FW_STATUS)  return true;
    return false;
}

void SPISlaveInit() {
    memset((void *)regMap, 0, sizeof(regMap));

    regMap[REG_DEVICE_ID]  = DEVICE_ID_VAL;
    regMap[REG_FW_VER_MAJ] = FW_VERSION_MAJ;
    regMap[REG_FW_VER_MIN] = FW_VERSION_MIN;
    regMap[REG_STATUS]     = 0x01;

    regMap[REG_INT_MASK]    = 0x0F;
    regMap[REG_DI_DEBOUNCE] = 10;
    regMap[REG_FREQ_WINDOW] = 100; // 100 × 10 ms = 1 s default frequency window
    regMap[REG_ADC0_ALPHA]  = 26;
    regMap[REG_ADC1_ALPHA]  = 26;
    regMap[REG_ADC2_ALPHA]  = 26;
    regMap[REG_ADC3_ALPHA]  = 26;
    // 1000 Hz = 0x03E8 default for all slices
    regMap[REG_PWM_FREQ0_L] = 0xE8; regMap[REG_PWM_FREQ0_H] = 0x03;
    regMap[REG_PWM_FREQ1_L] = 0xE8; regMap[REG_PWM_FREQ1_H] = 0x03;
    regMap[REG_PWM_FREQ2_L] = 0xE8; regMap[REG_PWM_FREQ2_H] = 0x03;
    regMap[REG_PWM_FREQ3_L] = 0xE8; regMap[REG_PWM_FREQ3_H] = 0x03;
    regMap[REG_OUT_MODE]    = 0x00;

    spi_init(spi1, 10000000);
    spi_set_slave(spi1, true);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_INT);
    gpio_set_dir(PIN_INT, GPIO_OUT);
    gpio_put(PIN_INT, 1);
}

static void spiFlushFifos() {
    spi_hw_t *hw = spi_get_hw(spi1);
    // Toggling SSE is the only way to clear the PL022 TX FIFO
    hw->cr1 &= ~SPI_SSPCR1_SSE_BITS;
    while (hw->sr & SPI_SSPSR_RNE_BITS) {
        (void)hw->dr;
    }
    hw->cr1 |= SPI_SSPCR1_SSE_BITS;
}

// Wait for a data byte mid-transaction. Fails (and flags the error) if the
// master deasserts CS or stops clocking.
static bool spiReadByteBlocking(uint8_t *out) {
    spi_hw_t *hw = spi_get_hw(spi1);
    uint32_t start = time_us_32();
    while (!(hw->sr & SPI_SSPSR_RNE_BITS)) {
        if (gpio_get(PIN_CS)) {
            regMap[REG_ERROR] |= ERR_SPI_ABORT;
            return false;
        }
        if ((time_us_32() - start) > SPI_BYTE_TIMEOUT_US) {
            regMap[REG_ERROR] |= ERR_SPI_TIMEOUT;
            return false;
        }
    }
    *out = (uint8_t)hw->dr;
    return true;
}

// Read a register value for a read transaction, applying the multi-byte latch
// rules: the first byte of an ADC/counter value snapshots the rest so a wide
// value can't tear — across either a burst or separate single transactions.
static uint8_t spiReadRegister(uint8_t addr) {
    uint8_t val = regMap[addr];

    // L-byte read latches the H byte; an H-byte read returns that latch only if
    // it directly follows its L byte (same base), else it returns live data.
    if (addr >= REG_ADC0_L && addr <= REG_ADC3_H) {
        uint8_t idx = (uint8_t)((addr - REG_ADC0_L) >> 1);
        if (((addr - REG_ADC0_L) & 1) == 0) {
            adcLatchH[idx] = regMap[addr + 1];
            lastLatchBase  = addr;
        } else if (lastLatchBase == (uint8_t)(addr - 1)) {
            val = adcLatchH[idx];
        }
    }

    // Byte-0 read latches the upper 3 bytes; a follow-on byte returns the latch
    // only if it continues that same byte-0 read, else it returns live data.
    if (addr >= REG_CNT0_0 && addr <= REG_CNT_END) {
        uint8_t idx = (uint8_t)((addr - REG_CNT0_0) >> 2);
        uint8_t off = (uint8_t)((addr - REG_CNT0_0) & 3);
        if (off == 0) {
            cntLatch[idx][0] = regMap[addr + 1];
            cntLatch[idx][1] = regMap[addr + 2];
            cntLatch[idx][2] = regMap[addr + 3];
            lastLatchBase    = addr;
        } else if (lastLatchBase == (uint8_t)(addr - off)) {
            val = cntLatch[idx][off - 1];
        }
    }

    return val;
}

// Clear-on-read side effects, applied only after a byte is actually clocked
// out — never for a speculatively-prepared byte the master chooses to skip.
static void spiReadClear(uint8_t addr) {
    if (addr == REG_INT_FLAGS) regMap[REG_INT_FLAGS] = 0;
    if (addr == REG_ERROR)     regMap[REG_ERROR]     = 0;
}

// Result of trying to clock out one response byte.
enum SpiServe { SPI_SERVE_SENT, SPI_SERVE_CSEND, SPI_SERVE_TIMEOUT };

// Queue one response byte and wait for the master to clock it out. Unlike a
// mid-byte abort, a CS deassert here is the NORMAL end of a read burst (the
// master stops once it has the bytes it wants), so it returns CSEND rather
// than flagging an error.
static SpiServe spiServeReadByte(uint8_t val) {
    spi_hw_t *hw = spi_get_hw(spi1);
    hw->dr = (uint32_t)val;
    uint32_t start = time_us_32();
    while (hw->sr & SPI_SSPSR_BSY_BITS) {
        if (gpio_get(PIN_CS)) return SPI_SERVE_CSEND;
        if ((time_us_32() - start) > SPI_BYTE_TIMEOUT_US) return SPI_SERVE_TIMEOUT;
    }
    // Discard the dummy byte clocked in alongside our response
    while (hw->sr & SPI_SSPSR_RNE_BITS) {
        (void)hw->dr;
    }
    return SPI_SERVE_SENT;
}

// Receive one streamed byte for a firmware-data burst write. Like a normal
// mid-transaction read, but a CS deassert between bytes is the NORMAL end of a
// streamed block (CSEND) rather than an abort.
static SpiServe spiRecvStreamByte(uint8_t *out) {
    spi_hw_t *hw = spi_get_hw(spi1);
    uint32_t start = time_us_32();
    while (!(hw->sr & SPI_SSPSR_RNE_BITS)) {
        if (gpio_get(PIN_CS)) return SPI_SERVE_CSEND;
        if ((time_us_32() - start) > SPI_BYTE_TIMEOUT_US) return SPI_SERVE_TIMEOUT;
    }
    *out = (uint8_t)hw->dr;
    return SPI_SERVE_SENT;
}

void SPISlaveUpdate() {
    spi_hw_t *hw = spi_get_hw(spi1);

    if (gpio_get(PIN_CS)) {
        // Idle. Unconsumed RX bytes mean an aborted or oversized transaction;
        // a lone leftover TX byte is the expected tail of a read burst (the
        // speculatively-queued next byte the master chose not to clock) and is
        // flushed without flagging an error.
        bool rxLeftover = (hw->sr & SPI_SSPSR_RNE_BITS) != 0;
        bool txLeftover = (hw->sr & SPI_SSPSR_TFE_BITS) == 0;
        if (rxLeftover || txLeftover) {
            spiFlushFifos();
            if (rxLeftover) regMap[REG_ERROR] |= ERR_SPI_DESYNC;
        }
        return;
    }

    // Selected but no command byte yet: return instead of spinning so the
    // main loop (ADC, debounce, PWM) stays alive even if CS is held low.
    if (!(hw->sr & SPI_SSPSR_RNE_BITS)) {
        return;
    }

    uint8_t cmd = (uint8_t)hw->dr;
    bool isRead = (cmd & SPI_CMD_READ) != 0;
    uint8_t addr = cmd & SPI_ADDR_MASK;

    if (isRead) {
        // Auto-incrementing burst read: serve regMap[addr], addr+1, addr+2 …
        // one byte per clocked byte until the master deasserts CS. The address
        // wraps within the 7-bit space; a single read is just a burst of one.
        uint8_t cur = addr;
        unsigned served = 0;
        for (;;) {
            SpiServe r = spiServeReadByte(spiReadRegister(cur));
            if (r == SPI_SERVE_SENT) {
                spiReadClear(cur);
                cur = (uint8_t)((cur + 1) & SPI_ADDR_MASK);
                // Bound the burst so a master that clocks forever can't starve
                // the main loop. The whole map is 128 bytes; 256 is ample.
                if (++served >= 256) {
                    regMap[REG_ERROR] |= ERR_SPI_DESYNC;
                    break;
                }
                continue;
            }
            if (r == SPI_SERVE_TIMEOUT) {
                regMap[REG_ERROR] |= ERR_SPI_TIMEOUT;
            }
            break; // SPI_SERVE_CSEND = clean end of burst
        }
        // Drop the speculatively-queued byte the master never clocked so the
        // next frame starts aligned regardless of when the idle poll runs.
        spiFlushFifos();
    } else if (addr == REG_FW_DATA_PORT) {
        // Streaming firmware-image write: every clocked byte goes to the data
        // port (no auto-increment) and is buffered for the main loop to flush
        // to LittleFS. Ends when the master deasserts CS.
        for (;;) {
            uint8_t b;
            SpiServe r = spiRecvStreamByte(&b);
            if (r == SPI_SERVE_SENT) {
                // Stop consuming once the block buffer is full (or no session is
                // active): bounds the loop so a runaway master can't starve the
                // main loop, and discards over-sized blocks.
                if (!FWUpdateFeed(b)) break;
                continue;
            }
            if (r == SPI_SERVE_TIMEOUT) {
                regMap[REG_ERROR] |= ERR_SPI_TIMEOUT;
            }
            break; // SPI_SERVE_CSEND = clean end of the block
        }
        spiFlushFifos();
    } else {
        uint8_t val = 0;
        if (!spiReadByteBlocking(&val)) {
            return;
        }
        regWrite(addr, val);
    }
}

uint8_t regRead(uint8_t addr) {
    return regMap[addr];
}

void regWrite(uint8_t addr, uint8_t val) {
    if (isReadOnly(addr)) {
        regMap[REG_ERROR] |= ERR_RO_WRITE;
        return;
    }
    regMap[addr] = val;

    // Frequency is applied on the H-byte write (master writes L first),
    // so the main loop never sees a half-written 16-bit value
    if (addr >= REG_PWM_FREQ0_L && addr <= REG_PWM_FREQ3_H &&
        ((addr - REG_PWM_FREQ0_L) & 1)) {
        pwmFreqPending |= (uint8_t)(1u << ((addr - REG_PWM_FREQ0_L) >> 1));
    }

    // Mode/edge changes need the GPIO interrupts reconfigured; defer to the
    // main loop so we never touch IRQ config from inside an SPI transaction
    if (addr == REG_DI_MODE || addr == REG_DI_EDGE_CFG) {
        diReconfigPending = 1;
    }

    // CNT_CLEAR is a command register: latch the requested bits for the main
    // loop and self-clear so a read shows the command has been consumed
    if (addr == REG_CNT_CLEAR) {
        cntClearPending |= val;
        regMap[REG_CNT_CLEAR] = 0;
    }

    // Firmware-update commands touch LittleFS/flash; defer to the main loop
    if (addr == REG_FW_CMD) {
        fwCmdPending = val;
    }
}

void setINT(bool assert) {
    gpio_put(PIN_INT, assert ? 0 : 1);
}
