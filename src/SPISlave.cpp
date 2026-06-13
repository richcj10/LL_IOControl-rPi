#include "SPISlave.h"
#include "Registers.h"
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

// H bytes latched on the matching L-byte read so 16-bit ADC values can't tear
static uint8_t adcLatchH[4];
// Upper 3 bytes latched on the byte-0 read so 32-bit counters can't tear
static uint8_t cntLatch[4][3];

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

// Queue a response byte and wait for the master to clock it out, with the
// same CS/timeout bailouts. The aborted byte is left in the TX FIFO and gets
// flushed on the next idle pass.
static bool spiWriteByteBlocking(uint8_t val) {
    spi_hw_t *hw = spi_get_hw(spi1);
    hw->dr = (uint32_t)val;
    uint32_t start = time_us_32();
    while (hw->sr & SPI_SSPSR_BSY_BITS) {
        if (gpio_get(PIN_CS)) {
            regMap[REG_ERROR] |= ERR_SPI_ABORT;
            return false;
        }
        if ((time_us_32() - start) > SPI_BYTE_TIMEOUT_US) {
            regMap[REG_ERROR] |= ERR_SPI_TIMEOUT;
            return false;
        }
    }
    // Discard the dummy byte the master clocked in while reading the response
    while (hw->sr & SPI_SSPSR_RNE_BITS) {
        (void)hw->dr;
    }
    return true;
}

void SPISlaveUpdate() {
    spi_hw_t *hw = spi_get_hw(spi1);

    if (gpio_get(PIN_CS)) {
        // Idle. Anything left in the FIFOs means an aborted or oversized
        // transaction — flush so the next frame starts aligned.
        if ((hw->sr & SPI_SSPSR_RNE_BITS) || !(hw->sr & SPI_SSPSR_TFE_BITS)) {
            spiFlushFifos();
            regMap[REG_ERROR] |= ERR_SPI_DESYNC;
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
        uint8_t val = regMap[addr];

        // L-byte read latches the H byte; H-byte read returns the latch,
        // so a 16-bit ADC value can't tear across the two transactions
        if (addr >= REG_ADC0_L && addr <= REG_ADC3_H) {
            uint8_t idx = (uint8_t)((addr - REG_ADC0_L) >> 1);
            if (((addr - REG_ADC0_L) & 1) == 0) {
                adcLatchH[idx] = regMap[addr + 1];
            } else {
                val = adcLatchH[idx];
            }
        }

        // Byte-0 read latches the upper 3 bytes; later byte reads return the
        // latch, so a 32-bit counter/position can't tear across transactions
        if (addr >= REG_CNT0_0 && addr <= REG_CNT_END) {
            uint8_t idx = (uint8_t)((addr - REG_CNT0_0) >> 2);
            uint8_t off = (uint8_t)((addr - REG_CNT0_0) & 3);
            if (off == 0) {
                cntLatch[idx][0] = regMap[addr + 1];
                cntLatch[idx][1] = regMap[addr + 2];
                cntLatch[idx][2] = regMap[addr + 3];
            } else {
                val = cntLatch[idx][off - 1];
            }
        }

        if (!spiWriteByteBlocking(val)) {
            return;
        }

        if (addr == REG_INT_FLAGS) {
            regMap[REG_INT_FLAGS] = 0;
        }
        if (addr == REG_ERROR) {
            regMap[REG_ERROR] = 0;
        }
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
}

void setINT(bool assert) {
    gpio_put(PIN_INT, assert ? 0 : 1);
}
