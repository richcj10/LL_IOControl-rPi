# LL-IOController

RP2040 (Raspberry Pi Pico) firmware that turns the chip into an **SPI-slave I/O
co-processor** for a host MCU (an ESP32). The host drives outputs and reads
inputs through an 8-bit register file over SPI.

## Features

- **7 outputs** — per-channel GPIO or PWM (0–255 duty), per-slice PWM frequency.
- **4 ADC inputs** — 12-bit, IIR-filtered, per-channel filter coefficient.
- **4 digital inputs** — each independently configurable over SPI as:
  - **LEVEL** — debounced on/off with change interrupts (default),
  - **ENCODER** — quadrature ×4 decode → signed 32-bit position (2 pairs),
  - **FREQUENCY** — edge frequency → Hz, configurable edge & window.
- **Interrupt line** to the host for digital-input changes.

## Interface

The full SPI protocol, register/memory map, and input config map are documented
in **[docs/REGISTER_MAP.md](docs/REGISTER_MAP.md)**.

## Build

PlatformIO (RP2040, earlephilhower Arduino core):

```
pio run            # build
pio run -t upload  # flash
```

## Layout

| File | Purpose |
|------|---------|
| [src/main.cpp](src/main.cpp)       | Main loop: ADC → digital I/O → PWM → SPI service |
| [src/SPISlave.cpp](src/SPISlave.cpp) | SPI-slave transaction handling, register file |
| [src/DigitalIO.cpp](src/DigitalIO.cpp) | Inputs (level/encoder/frequency), outputs, PWM |
| [src/ADC.cpp](src/ADC.cpp)         | ADC sampling and filtering |
| [src/Registers.h](src/Registers.h) | Register addresses and bit definitions |
