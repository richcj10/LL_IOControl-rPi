#include "hardware/adc.h"
#include "Registers.h"
#include "SPISlave.h"

static float filteredADC[4] = {0};

void ADCInit() {
    adc_init();
    for (int ch = 0; ch < 4; ch++) {
        adc_gpio_init(26 + ch);
    }
}

void ADCUpdate() {
    for (int ch = 0; ch < 4; ch++) {
        adc_select_input(ch);
        uint16_t raw = adc_read();
        float alpha = regMap[REG_ADC0_ALPHA + ch] / 255.0f;
        filteredADC[ch] = alpha * (raw << 4) + (1.0f - alpha) * filteredADC[ch];
        uint16_t out = (uint16_t)filteredADC[ch];
        regMap[REG_ADC0_L + ch * 2] = out & 0xFF;
        regMap[REG_ADC0_H + ch * 2] = out >> 8;
    }
}
