#pragma once
#include <Arduino.h>

void DigitalIOInit();
void DigitalIOUpdate();

void applyOutput(uint8_t channel, uint8_t mode, uint8_t value);
void applyPWMFreq(uint8_t slice, uint16_t hz);
