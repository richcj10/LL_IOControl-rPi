#pragma once
#include <stdint.h>

extern volatile uint8_t regMap[256];

// Bit n set = slice n has a freshly latched 16-bit frequency to apply
// (set by the REG_PWM_FREQn_H write, consumed by the main loop)
extern volatile uint8_t pwmFreqPending;

// Set when REG_DI_MODE/REG_DI_EDGE_CFG is written; the main loop reconfigures
// the input GPIO interrupts and clears it.
extern volatile uint8_t diReconfigPending;

// Bitmask of counters to zero (bit n = input n), set by a REG_CNT_CLEAR write
// and consumed by the main loop.
extern volatile uint8_t cntClearPending;

void SPISlaveInit();
void SPISlaveUpdate();
void setINT(bool assert);
uint8_t regRead(uint8_t addr);
void regWrite(uint8_t addr, uint8_t val);
