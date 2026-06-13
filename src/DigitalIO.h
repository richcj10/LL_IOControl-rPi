#pragma once
#include <Arduino.h>

void DigitalIOInit();
void DigitalIOUpdate();

// Disable / re-enable the input edge interrupts. Used to quiesce the inputs
// during firmware-update flash writes (no ISR should run from XIP flash while
// it is being programmed). Resume re-applies the current REG_DI_MODE config.
void DigitalIOSuspendInputs();
void DigitalIOResumeInputs();

void applyOutput(uint8_t channel, uint8_t mode, uint8_t value);
void applyPWMFreq(uint8_t slice, uint16_t hz);
