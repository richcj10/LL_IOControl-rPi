# LL-IOController — SPI Protocol & Register Map

RP2040-based I/O co-processor. Acts as an **SPI slave**; the host (an ESP32)
reads/writes an 8-bit register file to drive outputs and read inputs. This
document is the authoritative interface contract between firmware and host.

Firmware version: see `REG_DEVICE_ID` / `REG_FW_VER_*` (currently `0xA1`, v0.1).

---

## 1. SPI physical layer

| Property        | Value                                   |
|-----------------|-----------------------------------------|
| Peripheral      | `spi1` (slave)                          |
| Mode            | 0 (CPOL = 0, CPHA = 0)                  |
| Bit order       | MSB first                               |
| Word size       | 8 bits                                  |
| Max clock       | 10 MHz                                  |
| Byte timeout    | 1000 µs of SCK inactivity → abort frame |

### Pinout (RP2040 GPIO)

| Signal | GPIO | Direction (device) |
|--------|------|--------------------|
| MOSI   | 8    | in                 |
| CS     | 9    | in                 |
| INT    | 13   | out (to host)      |
| SCK    | 14   | in                 |
| MISO   | 15   | out                |

`INT` is an active-low, level-driven interrupt to the host: asserted (low)
while any **unmasked** flag is pending in `REG_INT_FLAGS`, deasserted once the
host reads `REG_INT_FLAGS` (which clears it). See [§5](#5-interrupts).

---

## 2. Transaction format

Every transaction is **2 bytes**: a command byte followed by one data byte.

### Command byte

```
 bit 7   6   5   4   3   2   1   0
      R/W  └────────  address  ────────┘
```

- **bit 7** — `1` = read, `0` = write.
- **bits 6:0** — register address (`0x00`–`0x7F`). Only these 128 addresses
  are reachable; the high bit is consumed by R/W.

### Write `[cmd=0x00|addr] [value]`
Master sends the command byte, then the value byte. The register updates on
receipt of the value.

### Read `[cmd=0x80|addr] [dummy] [dummy] …`
Master sends the command byte, then clocks dummy bytes to read data. The device
returns the register value on the second byte, and **auto-increments** the
address for every further byte clocked — so you don't re-send the address for
sequential registers. The master ends the burst simply by deasserting CS.

```
TX: 0x80|0x50   RX: —     (command byte; RX discarded)
TX: 0x00        RX: b0    (regMap[0x50])
TX: 0x00        RX: b1    (regMap[0x51])
TX: 0x00        RX: b2    (regMap[0x52])
TX: 0x00        RX: b3    (regMap[0x53])   ← deassert CS to stop
```

The address wraps within the 7-bit space (`0x7F → 0x00`). A single-register
read is just a burst of length one. Multi-byte latching ([§3](#3-multi-byte-values-tear-free-reads))
composes naturally: a burst starting at byte 0 of an ADC/counter value latches
the rest on the first byte and reads the latch on the following bytes, so the
whole value is coherent.

> **Burst writes are not supported** — writes are always a single
> `[cmd][value]` pair. Only reads auto-increment.

> CS must stay asserted for the whole transaction. Deasserting CS *mid-byte*,
> or stalling SCK past the byte timeout, aborts the frame and sets an error bit
> (`ERR_SPI_ABORT` on a write / `ERR_SPI_TIMEOUT`). Ending a read burst by
> deasserting CS *between* bytes is normal and is **not** an error. Unconsumed
> RX bytes left at idle set `ERR_SPI_DESYNC` and are flushed.

> **Timing:** after the command byte, allow the slave a moment to enter the
> transaction before clocking data (the slave services SPI from its polled main
> loop, which can be busy for up to ~1 ms). Once the burst is flowing, data
> bytes are served from a tight loop; keep the clock moderate (or insert small
> inter-byte gaps) so the slave can stage each next byte, especially while
> encoder interrupts are active.

---

## 3. Multi-byte values (tear-free reads)

Some values are wider than 8 bits and update asynchronously to SPI reads.
To prevent reading a half-updated value, the device **latches** the remaining
bytes when you read the first byte. **Always read the low/first byte first.**

| Value type        | Width  | Read order            | Registers          |
|-------------------|--------|-----------------------|--------------------|
| ADC sample        | 16-bit | L byte latches H      | `0x20`–`0x27`      |
| Smart-input value | 32-bit | byte 0 latches 1–3    | `0x50`–`0x5F`      |

16-bit **PWM frequency writes** are the mirror case: write the **L byte first,
then the H byte** — the frequency is applied on the H-byte write, so a
half-written value is never used.

---

## 4. Register map

Legend: **R** = read-only (writes set `ERR_RO_WRITE`), **R/W** = read-write,
**COR** = clear-on-read, **WC** = self-clearing command register.

### System `0x00`–`0x0F`

| Addr | Name            | Acc | Description                                  |
|------|-----------------|-----|----------------------------------------------|
| 0x00 | `REG_DEVICE_ID` | R   | Device ID, fixed `0xA1`                       |
| 0x01 | `REG_FW_VER_MAJ`| R   | Firmware major version                        |
| 0x02 | `REG_FW_VER_MIN`| R   | Firmware minor version                        |
| 0x03 | `REG_STATUS`    | R   | Status flags (bit0 = ready)                   |
| 0x04 | `REG_INT_FLAGS` | R/COR | DI change flags, bits 3:0. Cleared on read   |
| 0x05 | `REG_INT_MASK`  | R/W | Gates the INT pin only; flags still latch     |
| 0x06 | `REG_ERROR`     | R/COR | Error bits (see [§6](#6-error-register)). COR |

### Digital I/O & input config `0x10`–`0x1F`

| Addr | Name             | Acc | Description                                   |
|------|------------------|-----|-----------------------------------------------|
| 0x10 | `REG_DI_STATE`   | R   | Debounced input state, bit n = input n (active-high). Only LEVEL-mode inputs report here |
| 0x11 | `REG_DI_DEBOUNCE`| R/W | Debounce window in ms (0 → 10). LEVEL mode only |
| 0x12 | `REG_OUT_MODE`   | R/W | Output mode, bit n: 0 = GPIO, 1 = PWM          |
| 0x13 | `REG_OUT_VAL_0`  | R/W | Output 0 value (GPIO: 0/non-0; PWM: 0–255 duty)|
| 0x14 | `REG_OUT_VAL_1`  | R/W | Output 1 value                                 |
| 0x15 | `REG_OUT_VAL_2`  | R/W | Output 2 value                                 |
| 0x16 | `REG_OUT_VAL_3`  | R/W | Output 3 value                                 |
| 0x17 | `REG_OUT_VAL_4`  | R/W | Output 4 value                                 |
| 0x18 | `REG_OUT_VAL_5`  | R/W | Output 5 value                                 |
| 0x19 | `REG_OUT_VAL_6`  | R/W | Output 6 value                                 |
| 0x1A | `REG_DI_MODE`    | R/W | Per-input mode, 2 bits/input (see [§7](#7-input-config-map)) |
| 0x1B | `REG_DI_EDGE_CFG`| R/W | Per-input edge select for FREQUENCY mode       |
| 0x1C | `REG_CNT_CLEAR`  | WC  | Write bitmask (bit n = input n) to zero counters |
| 0x1D | `REG_FREQ_WINDOW`| R/W | Frequency window, 10 ms units (default 100 = 1 s; 0 → 1 s) |

### ADC samples `0x20`–`0x27` (read L first)

| Addr | Name        | Acc | Description                  |
|------|-------------|-----|------------------------------|
| 0x20 | `REG_ADC0_L`| R   | ADC ch0, low byte (latches H)|
| 0x21 | `REG_ADC0_H`| R   | ADC ch0, high byte           |
| 0x22 | `REG_ADC1_L`| R   | ADC ch1, low byte            |
| 0x23 | `REG_ADC1_H`| R   | ADC ch1, high byte           |
| 0x24 | `REG_ADC2_L`| R   | ADC ch2, low byte            |
| 0x25 | `REG_ADC2_H`| R   | ADC ch2, high byte           |
| 0x26 | `REG_ADC3_L`| R   | ADC ch3, low byte            |
| 0x27 | `REG_ADC3_H`| R   | ADC ch3, high byte           |

Samples are 12-bit ADC readings left-shifted to 16-bit, then IIR-filtered.

### ADC filter `0x30`–`0x33`

| Addr | Name           | Acc | Description                                |
|------|----------------|-----|--------------------------------------------|
| 0x30 | `REG_ADC0_ALPHA`| R/W | IIR coefficient ch0, 0–255 ↦ α 0.0–1.0     |
| 0x31 | `REG_ADC1_ALPHA`| R/W | ch1. α=0 freezes output; α=255 = raw        |
| 0x32 | `REG_ADC2_ALPHA`| R/W | ch2                                         |
| 0x33 | `REG_ADC3_ALPHA`| R/W | ch3                                         |

### PWM frequency `0x40`–`0x47` (write L then H)

These set PWM **frequency**, not duty cycle. Duty is set separately, per output
channel, via `REG_OUT_VAL_n` (`0x13`–`0x19`, 0–255 ↦ 0–100%) with the channel
in PWM mode (`REG_OUT_MODE` bit set).

Frequency is **per slice**, and a slice drives a *pair* of pins:
slice 0 = GP0/GP1, slice 1 = GP2/GP3, slice 2 = GP4/GP5, slice 3 = GP6. So the
two pins on a slice **share one frequency** but each keeps its **own duty
cycle** (e.g. GP0 at 25 % and GP1 at 75 % at the same Hz). Changing a slice's
frequency rescales every PWM channel on it so duty % stays constant.

Frequency in Hz, 16-bit; applied on the H-byte write. Floor ≈ 7.46 Hz
(below it `ERR_PWM_FREQ_RANGE` is set and the value is clamped).

| Addr | Name             | Acc | Description            |
|------|------------------|-----|------------------------|
| 0x40 | `REG_PWM_FREQ0_L`| R/W | Slice 0 freq, low byte |
| 0x41 | `REG_PWM_FREQ0_H`| R/W | Slice 0 freq, high byte|
| 0x42 | `REG_PWM_FREQ1_L`| R/W | Slice 1 freq, low byte |
| 0x43 | `REG_PWM_FREQ1_H`| R/W | Slice 1 freq, high byte|
| 0x44 | `REG_PWM_FREQ2_L`| R/W | Slice 2 freq, low byte |
| 0x45 | `REG_PWM_FREQ2_H`| R/W | Slice 2 freq, high byte|
| 0x46 | `REG_PWM_FREQ3_L`| R/W | Slice 3 freq, low byte |
| 0x47 | `REG_PWM_FREQ3_H`| R/W | Slice 3 freq, high byte|

### Smart-input values `0x50`–`0x5F` (read byte 0 first)

Signed 32-bit, little-endian. Encoder position (ENCODER mode) or last measured
frequency in Hz (FREQUENCY mode); reads 0 in LEVEL mode. Reading byte 0
latches bytes 1–3.

| Addr range  | Name        | Acc | Input |
|-------------|-------------|-----|-------|
| 0x50–0x53   | `REG_CNT0_0`| R   | 0     |
| 0x54–0x57   | `REG_CNT1_0`| R   | 1     |
| 0x58–0x5B   | `REG_CNT2_0`| R   | 2     |
| 0x5C–0x5F   | `REG_CNT3_0`| R   | 3     |

### Firmware update `0x60`–`0x73`

Stub addresses reserved for the field-update protocol (`REG_FW_CMD`,
`REG_FW_STATUS`, `REG_FW_BLOCK_ADDR*`, `REG_FW_BLOCK_SIZE*`, `REG_FW_DATA_PORT`,
`REG_FW_CRC32_*`). Not yet implemented.

---

## 5. Interrupts

- A debounced change on any **LEVEL**-mode input sets the matching bit in
  `REG_INT_FLAGS` (bits 3:0 = inputs 3:0).
- Flags **always** accumulate, even for masked inputs.
- `REG_INT_MASK` gates only whether the `INT` **pin** is driven, not whether
  flags latch. The pin is asserted while `(INT_FLAGS & INT_MASK) != 0`.
- Reading `REG_INT_FLAGS` clears it (and so deasserts the pin).

ENCODER/FREQUENCY inputs do **not** raise INT or appear in `REG_DI_STATE`;
the host polls their value registers (`0x50`–`0x5F`).

---

## 6. Error register

`REG_ERROR` (`0x06`) is clear-on-read. Bits:

| Bit  | Name                  | Meaning                                       |
|------|-----------------------|-----------------------------------------------|
| 0x01 | `ERR_RO_WRITE`        | Write attempted to a read-only register       |
| 0x02 | `ERR_PWM_FREQ_RANGE`  | PWM freq 0 or below ~7.46 Hz; clamped/ignored |
| 0x04 | `ERR_SPI_ABORT`       | CS deasserted mid-transaction                 |
| 0x08 | `ERR_SPI_TIMEOUT`     | SCK stalled mid-transaction                    |
| 0x10 | `ERR_SPI_DESYNC`      | Stale FIFO bytes flushed at idle               |
| 0x20 | `ERR_DI_MODE`         | Invalid input mode (e.g. half an encoder pair) |

---

## 7. Input config map

Four digital inputs map to RP2040 GPIO as:

| Input | GPIO | Encoder role  |
|-------|------|---------------|
| 0     | 10   | enc0 channel A|
| 1     | 11   | enc0 channel B|
| 2     | 12   | enc1 channel A|
| 3     | 16   | enc1 channel B|

### `REG_DI_MODE` (0x1A) — 2 bits per input

```
 bit 7   6 | 5   4 | 3   2 | 1   0
   input 3 | input 2 | input 1 | input 0
```

| Value | Mode      | Behavior                                              |
|-------|-----------|-------------------------------------------------------|
| 0     | LEVEL     | Debounced on/off → `REG_DI_STATE`, raises INT (default)|
| 1     | ENCODER   | Quadrature ×4 decode → signed position in `0x50+`     |
| 2     | FREQUENCY | Edge frequency → Hz in `0x50+`                         |
| 3     | reserved  | —                                                     |

**Encoder pairing:** ENCODER consumes a fixed pair — enc0 = inputs {0, 1},
enc1 = inputs {2, 3}. **Both** pins of a pair must be set to ENCODER; setting
only one sets `ERR_DI_MODE` and that pair stays inactive. Both pins of a pair
report the same position value in their respective `0x50+` slots.

### `REG_DI_EDGE_CFG` (0x1B) — 2 bits per input (FREQUENCY only)

| Value | Edge counted |
|-------|--------------|
| 0     | rising       |
| 1     | falling      |
| 2     | both         |

Ignored for LEVEL and ENCODER (encoder always decodes every A/B edge).

### `REG_CNT_CLEAR` (0x1C)

Write a bitmask — bit n zeroes input n's value (for encoders, either pin's bit
clears the pair). Self-clearing: reads back 0 once applied.

### `REG_FREQ_WINDOW` (0x1D)

Measurement window in 10 ms units (e.g. 50 = 500 ms, 200 = 2 s). Default 100
(1 s). 0 is treated as 1 s. Larger windows give finer low-frequency resolution
but slower updates.

---

## 8. Examples

Read the 16-bit ADC channel 1 value:

```
TX: 0x80|0x22  (read REG_ADC1_L)   RX: <low byte>   ← latches high byte
TX: 0x80|0x23  (read REG_ADC1_H)   RX: <high byte>
value = low | (high << 8)
```

Configure input 0 as a frequency counter on both edges, 500 ms window:

```
write REG_DI_MODE     (0x1A) = 0x02   ; input0 = FREQUENCY, others LEVEL
write REG_DI_EDGE_CFG (0x1B) = 0x02   ; input0 = both edges
write REG_FREQ_WINDOW (0x1D) = 50     ; 50 × 10 ms = 500 ms
```

Read input 0 frequency (Hz), 32-bit — single burst, address sent once:

```
TX: 0x80|0x50  RX: —    ; command (read, addr 0x50)
TX: 0x00       RX: b0   ; regMap[0x50], latches b1..b3
TX: 0x00       RX: b1   ; regMap[0x51]
TX: 0x00       RX: b2   ; regMap[0x52]
TX: 0x00       RX: b3   ; regMap[0x53]  → deassert CS
hz = b0 | (b1<<8) | (b2<<16) | (b3<<24)
```

The same burst can keep going to sweep contiguous registers — e.g. start at
`0x50` and clock 16 bytes to read all four 32-bit input values in one frame.

Enable encoder 0 (inputs 0 & 1) and zero its position:

```
write REG_DI_MODE  (0x1A) = 0x05   ; input0=ENCODER(1), input1=ENCODER(1)
write REG_CNT_CLEAR(0x1C) = 0x01   ; zero the pair
; read signed position from 0x50..0x53 (byte 0 first)
```
