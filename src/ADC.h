#pragma once
#include <Arduino.h>

void ADCInit();

/**
 * @brief Sample all four ADC channels and update REG_ADCn_L/H.
 *
 * Each channel runs a single-pole IIR filter:
 *   out = alpha * in + (1 - alpha) * out
 * with alpha = REG_ADCn_ALPHA / 255. Raw 12-bit samples are left-shifted to
 * 16-bit full scale before filtering.
 *
 * @warning REG_ADCn_ALPHA = 0 freezes the filter: the channel's output
 *          registers hold their last value indefinitely and never track the
 *          input. Use 1 for the slowest tracking filter, or 255 for raw
 *          pass-through (no filtering).
 */
void ADCUpdate();
