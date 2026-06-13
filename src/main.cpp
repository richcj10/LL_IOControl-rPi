#include <Arduino.h>
#include "SPISlave.h"
#include "DigitalIO.h"
#include "ADC.h"
#include "Registers.h"

static const uint8_t PWM_FREQ_REGS[4][2] = {
    {REG_PWM_FREQ0_L, REG_PWM_FREQ0_H},
    {REG_PWM_FREQ1_L, REG_PWM_FREQ1_H},
    {REG_PWM_FREQ2_L, REG_PWM_FREQ2_H},
    {REG_PWM_FREQ3_L, REG_PWM_FREQ3_H},
};

void setup() {
    SPISlaveInit();
    DigitalIOInit();
    ADCInit();
}

void loop() {
    ADCUpdate();
    DigitalIOUpdate();

    // Frequencies are latched on the H-byte write, so both bytes are
    // guaranteed consistent by the time the pending bit is set
    for (uint8_t s = 0; s < 4; s++) {
        if (pwmFreqPending & (1u << s)) {
            pwmFreqPending &= (uint8_t)~(1u << s);
            uint16_t freq = regMap[PWM_FREQ_REGS[s][0]] |
                            ((uint16_t)regMap[PWM_FREQ_REGS[s][1]] << 8);
            applyPWMFreq(s, freq);
        }
    }

    SPISlaveUpdate();

    // Level-driven INT: assert while any unmasked flag is pending, deassert
    // once the ESP32 reads (and thereby clears) INT_FLAGS. Masked flags still
    // accumulate in the register but never drive the pin.
    setINT((regMap[REG_INT_FLAGS] & regMap[REG_INT_MASK]) != 0);
}
