#include "DigitalIO.h"
#include "Registers.h"
#include "SPISlave.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <math.h>

static const uint8_t OUTPUT_PINS[7] = {0, 1, 2, 3, 4, 5, 6};
static const uint8_t INPUT_PINS[4]  = {10, 11, 12, 16};

// Current wrap per slice — needed to scale 0-255 duty to actual counter range
static uint16_t sliceWrap[4] = {255, 255, 255, 255};

static uint8_t lastRaw[4];
static uint8_t stableState[4];
static uint32_t lastChangeTime[4];

static uint8_t prevOutVal[7];
static uint8_t prevOutMode;

// --- Smart-input (encoder / frequency) state ---------------------------------
// Quadrature decode table, indexed by (prevAB << 2) | curAB, giving the step
// for one A/B transition. Invalid (double-bit) transitions yield 0. x4 decode.
static const int8_t QUAD_TABLE[16] = {
    0,  1, -1,  0,
   -1,  0,  0,  1,
    1,  0,  0, -1,
    0, -1,  1,  0,
};

// Encoder pairs: enc p uses INPUT_PINS[2p] = A, INPUT_PINS[2p+1] = B.
static volatile int32_t  encPos[2];      // signed position, written in ISR
static volatile uint8_t  encPrevAB[2];   // last (A<<1)|B per pair
static volatile uint32_t freqAccum[4];   // edges counted in current window
static uint32_t freqHz[4];               // last computed Hz per input
static uint32_t freqWindowStart;         // millis() at window start

static int pinToInput(uint gpio) {
    for (int i = 0; i < 4; i++) {
        if (INPUT_PINS[i] == gpio) return i;
    }
    return -1;
}

static uint8_t inputMode(int i) {
    return (regMap[REG_DI_MODE] >> (i * 2)) & 0x03;
}

// GPIO edge ISR: counts frequency edges and decodes encoder transitions.
// Runs on core0, preempting the main loop — keeps only volatile state.
static void gpioCallback(uint gpio, uint32_t events) {
    int i = pinToInput(gpio);
    if (i < 0) return;

    uint8_t mode = inputMode(i);
    if (mode == DI_MODE_FREQ) {
        freqAccum[i]++;
    } else if (mode == DI_MODE_ENCODER) {
        int p = i >> 1;
        uint8_t a = gpio_get(INPUT_PINS[2 * p]) ? 1 : 0;
        uint8_t b = gpio_get(INPUT_PINS[2 * p + 1]) ? 1 : 0;
        uint8_t cur = (uint8_t)((a << 1) | b);
        encPos[p] += QUAD_TABLE[(encPrevAB[p] << 2) | cur];
        encPrevAB[p] = cur;
    }
}

// (Re)configure the per-input GPIO interrupts from REG_DI_MODE / REG_DI_EDGE_CFG.
// Called at init and whenever the master changes a mode/edge register.
static void configureInterrupts() {
    const uint32_t bothEdges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    // Start from a clean slate so a mode change can also disable interrupts
    for (int i = 0; i < 4; i++) {
        gpio_set_irq_enabled(INPUT_PINS[i], bothEdges, false);
    }

    // Encoders: both pins of a pair must be in ENCODER mode
    for (int p = 0; p < 2; p++) {
        bool aEnc = inputMode(2 * p)     == DI_MODE_ENCODER;
        bool bEnc = inputMode(2 * p + 1) == DI_MODE_ENCODER;
        if (aEnc && bEnc) {
            uint8_t a = gpio_get(INPUT_PINS[2 * p]) ? 1 : 0;
            uint8_t b = gpio_get(INPUT_PINS[2 * p + 1]) ? 1 : 0;
            encPrevAB[p] = (uint8_t)((a << 1) | b);
            gpio_set_irq_enabled(INPUT_PINS[2 * p],     bothEdges, true);
            gpio_set_irq_enabled(INPUT_PINS[2 * p + 1], bothEdges, true);
        } else if (aEnc || bEnc) {
            regMap[REG_ERROR] |= ERR_DI_MODE; // incomplete encoder pair
        }
    }

    // Frequency inputs (skip any pin already claimed by an active encoder)
    for (int i = 0; i < 4; i++) {
        if (inputMode(i) != DI_MODE_FREQ) continue;
        uint8_t edge = (regMap[REG_DI_EDGE_CFG] >> (i * 2)) & 0x03;
        uint32_t mask = (edge == DI_EDGE_RISING)  ? GPIO_IRQ_EDGE_RISE
                      : (edge == DI_EDGE_FALLING) ? GPIO_IRQ_EDGE_FALL
                                                  : bothEdges;
        gpio_set_irq_enabled(INPUT_PINS[i], mask, true);
    }

    gpio_set_irq_callback(gpioCallback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void DigitalIOInit() {
    for (int i = 0; i < 7; i++) {
        pinMode(OUTPUT_PINS[i], OUTPUT);
        digitalWrite(OUTPUT_PINS[i], LOW);
    }

    for (int i = 0; i < 4; i++) {
        pinMode(INPUT_PINS[i], INPUT_PULLUP);
        lastRaw[i]        = digitalRead(INPUT_PINS[i]);
        stableState[i]    = lastRaw[i];
        lastChangeTime[i] = millis();
    }

    // Configure each slice from the register defaults via the same clamped
    // path used at runtime (a hand-computed divider here previously exceeded
    // the 8.4-bit hardware maximum of 255.9375)
    for (uint8_t s = 0; s < 4; s++) {
        uint16_t hz = regMap[REG_PWM_FREQ0_L + s * 2] |
                      ((uint16_t)regMap[REG_PWM_FREQ0_L + s * 2 + 1] << 8);
        applyPWMFreq(s, hz);
    }

    for (int i = 0; i < 7; i++) {
        uint8_t pin = OUTPUT_PINS[i];
        uint slice  = pwm_gpio_to_slice_num(pin);
        pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
        pwm_set_enabled(slice, true);
    }

    prevOutMode = regMap[REG_OUT_MODE];
    for (int i = 0; i < 7; i++) {
        prevOutVal[i] = regMap[REG_OUT_VAL_0 + i];
    }

    freqWindowStart = millis();
    configureInterrupts();
}

void DigitalIOUpdate() {
    uint8_t debounce_ms = regMap[REG_DI_DEBOUNCE];
    if (debounce_ms == 0) debounce_ms = 10;

    // Apply a pending mode/edge change before sampling this pass
    if (diReconfigPending) {
        diReconfigPending = 0;
        configureInterrupts();
    }

    // Apply pending counter clears (bit n = input n; either pin clears its pair)
    if (cntClearPending) {
        uint8_t m = cntClearPending;
        cntClearPending = 0;
        noInterrupts();
        for (int i = 0; i < 4; i++) {
            if (m & (1 << i)) freqAccum[i] = 0;
        }
        for (int p = 0; p < 2; p++) {
            if (m & (3 << (2 * p))) encPos[p] = 0;
        }
        interrupts();
    }

    uint8_t diState = 0;
    uint8_t changedMask = 0;
    uint32_t now = millis();

    // LEVEL inputs: debounce as before. ENCODER/FREQUENCY inputs are serviced
    // by the GPIO ISR, so they're skipped here and leave their DI_STATE bit 0.
    for (int i = 0; i < 4; i++) {
        if (inputMode(i) != DI_MODE_LEVEL) continue;

        uint8_t raw = digitalRead(INPUT_PINS[i]);

        if (raw != lastRaw[i]) {
            lastRaw[i]        = raw;
            lastChangeTime[i] = now;
        }

        if ((now - lastChangeTime[i]) >= debounce_ms && raw != stableState[i]) {
            stableState[i] = raw;
            changedMask   |= (1 << i);
        }

        // INPUT_PULLUP: active-low — invert so bit=1 means input asserted
        if (!stableState[i]) {
            diState |= (1 << i);
        }
    }

    regMap[REG_DI_STATE] = diState;

    // Flags always accumulate, even for masked inputs — INT_MASK only gates
    // the INT pin, so the master can still see masked changes on a later read
    if (changedMask) {
        regMap[REG_INT_FLAGS] |= changedMask;
    }

    // Close out a frequency window: Hz = edges × 1000 / elapsed_ms
    uint16_t windowMs = (uint16_t)regMap[REG_FREQ_WINDOW] * 10;
    if (windowMs == 0) windowMs = 1000;
    uint32_t elapsed = now - freqWindowStart;
    if (elapsed >= windowMs) {
        freqWindowStart = now;
        for (int i = 0; i < 4; i++) {
            if (inputMode(i) != DI_MODE_FREQ) continue;
            noInterrupts();
            uint32_t edges = freqAccum[i];
            freqAccum[i] = 0;
            interrupts();
            freqHz[i] = (uint32_t)(((uint64_t)edges * 1000u) / elapsed);
        }
    }

    // Mirror each input's 32-bit value into its counter register block
    for (int i = 0; i < 4; i++) {
        int32_t v = 0;
        uint8_t mode = inputMode(i);
        if (mode == DI_MODE_ENCODER) {
            noInterrupts();
            v = encPos[i >> 1];   // both pins of a pair report the position
            interrupts();
        } else if (mode == DI_MODE_FREQ) {
            v = (int32_t)freqHz[i];
        }
        uint8_t base = REG_CNT0_0 + i * 4;
        regMap[base + 0] = (uint8_t)(v & 0xFF);
        regMap[base + 1] = (uint8_t)((v >> 8) & 0xFF);
        regMap[base + 2] = (uint8_t)((v >> 16) & 0xFF);
        regMap[base + 3] = (uint8_t)((v >> 24) & 0xFF);
    }

    uint8_t outMode = regMap[REG_OUT_MODE];
    for (int ch = 0; ch < 7; ch++) {
        uint8_t val  = regMap[REG_OUT_VAL_0 + ch];
        uint8_t mode = (outMode >> ch) & 0x01;
        if (val != prevOutVal[ch] || outMode != prevOutMode) {
            applyOutput(ch, mode, val);
            prevOutVal[ch] = val;
        }
    }
    prevOutMode = outMode;
}

void applyOutput(uint8_t channel, uint8_t mode, uint8_t value) {
    uint8_t pin   = OUTPUT_PINS[channel];
    uint slice    = pwm_gpio_to_slice_num(pin);
    uint chan     = pwm_gpio_to_channel(pin);

    if (mode == 0) {
        gpio_set_function(pin, GPIO_FUNC_SIO);
        gpio_set_dir(pin, GPIO_OUT);
        gpio_put(pin, value ? 1 : 0);
    } else {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        // Scale 0-255 duty byte to actual wrap range for this slice
        uint16_t level = (uint16_t)(((uint32_t)value * ((uint32_t)sliceWrap[slice] + 1)) / 255);
        pwm_set_chan_level(slice, chan, level);
    }
}

void applyPWMFreq(uint8_t slice, uint16_t hz) {
    if (slice >= 4) return;

    if (hz == 0) {
        regMap[REG_ERROR] |= ERR_PWM_FREQ_RANGE;
        return;
    }

    uint32_t total = 125000000UL / hz;
    float div;
    uint32_t wrap;

    if (total <= 65536UL) {
        div  = 1.0f;
        wrap = total - 1;
    } else {
        // Round div up to nearest 1/16 so wrap stays within 16-bit range
        div = ceilf(((float)total / 65536.0f) * 16.0f) / 16.0f;
        if (div > 255.9375f) div = 255.9375f;
        wrap = (uint32_t)((float)total / div) - 1;
        if (wrap > 65535UL) {
            // Requested frequency is below the ~7.46 Hz hardware floor
            // (125 MHz / (255.9375 * 65536)); clamp instead of truncating
            wrap = 65535UL;
            regMap[REG_ERROR] |= ERR_PWM_FREQ_RANGE;
        }
    }

    sliceWrap[slice] = (uint16_t)wrap;
    pwm_set_clkdiv(slice, div);
    pwm_set_wrap(slice, (uint16_t)wrap);
    // Counter may sit above the new wrap; reset it to avoid one full
    // 65536-count glitch period
    pwm_set_counter(slice, 0);

    // Duty levels were scaled against the old wrap — rescale every PWM-mode
    // channel on this slice
    uint8_t outMode = regMap[REG_OUT_MODE];
    for (uint8_t ch = 0; ch < 7; ch++) {
        if (pwm_gpio_to_slice_num(OUTPUT_PINS[ch]) != slice) continue;
        if ((outMode >> ch) & 0x01) {
            applyOutput(ch, 1, regMap[REG_OUT_VAL_0 + ch]);
        }
    }
}
