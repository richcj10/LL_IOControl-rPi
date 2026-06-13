#include "FWUpdate.h"
#include "Registers.h"
#include "SPISlave.h"
#include "DigitalIO.h"
#include <Arduino.h>
#include <LittleFS.h>
#include <PicoOTA.h>

// One streamed block is buffered in RAM, then flushed to LittleFS in the main
// loop. 4 KB bounds the SPI streaming burst and is LittleFS-friendly.
#define FW_BUF_SIZE     4096
#define FW_STAGING_PATH "/fw_update.img"

static uint8_t  fwBuf[FW_BUF_SIZE];
static volatile uint16_t fwBufLen = 0; // only touched from the main-loop thread
static bool     fwSession = false;
static bool     fwMounted = false;
static File     fwFile;
static uint32_t fwCrc;                  // running CRC-32 state (pre-final XOR)

// Reflected CRC-32 (poly 0xEDB88320) — must match the host and PicoOTA's OTACRC32.
static void crcReset() { fwCrc = 0xFFFFFFFFu; }
static void crcAdd(const uint8_t *d, uint32_t len) {
    uint32_t c = fwCrc;
    for (uint32_t i = 0; i < len; i++) {
        c ^= d[i];
        for (int j = 0; j < 8; j++) {
            c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
        }
    }
    fwCrc = c;
}
static uint32_t crcGet() { return ~fwCrc; }

static void fwFail() {
    if (fwFile) fwFile.close();
    fwSession = false;
    fwBufLen  = 0;
    regMap[REG_ERROR]     |= ERR_FW;
    regMap[REG_FW_STATUS]  = FW_ST_ERROR;
    DigitalIOResumeInputs();
}

void FWUpdateInit() {
    fwMounted = LittleFS.begin();
    regMap[REG_FW_STATUS] = FW_ST_IDLE;
}

// Called per clocked byte from the SPI streaming path. Kept tiny and flash-free.
void FWUpdateFeed(uint8_t b) {
    if (!fwSession) return; // ignore stray data outside a session
    if (fwBufLen < FW_BUF_SIZE) {
        fwBuf[fwBufLen++] = b;
    } else {
        // Block larger than the buffer — host must keep blocks <= FW_BUF_SIZE
        regMap[REG_ERROR]     |= ERR_FW;
        regMap[REG_FW_STATUS]  = FW_ST_ERROR;
    }
}

static void fwFlushBlock() {
    if (!fwSession || !fwFile) return;
    regMap[REG_FW_STATUS] = FW_ST_BUSY;
    uint16_t n = fwBufLen;
    size_t w = fwFile.write(fwBuf, n);
    fwFile.flush();
    if (w != n) { fwFail(); return; }
    crcAdd(fwBuf, n);
    fwBufLen = 0;
    regMap[REG_FW_STATUS] = FW_ST_READY;
}

static void fwBegin() {
    if (!fwMounted) fwMounted = LittleFS.begin();
    if (!fwMounted) { fwFail(); return; }

    // Quiesce inputs: no flash-resident ISR should run during LittleFS writes.
    DigitalIOSuspendInputs();

    LittleFS.remove(FW_STAGING_PATH);
    fwFile = LittleFS.open(FW_STAGING_PATH, "w");
    if (!fwFile) { fwFail(); return; }

    crcReset();
    fwBufLen  = 0;
    fwSession = true;
    regMap[REG_FW_STATUS] = FW_ST_READY;
}

static void fwEnd() {
    if (!fwSession) { fwFail(); return; }
    if (fwBufLen > 0) fwFlushBlock();
    if (!fwSession) return; // fwFlushBlock may have failed
    fwFile.close();
    fwSession = false;

    uint32_t expected = (uint32_t)regMap[REG_FW_CRC32_0]
                      | ((uint32_t)regMap[REG_FW_CRC32_1] << 8)
                      | ((uint32_t)regMap[REG_FW_CRC32_2] << 16)
                      | ((uint32_t)regMap[REG_FW_CRC32_3] << 24);

    regMap[REG_FW_STATUS] =
        (crcGet() == expected) ? FW_ST_VERIFY_OK : FW_ST_VERIFY_FAIL;
}

static void fwCommit() {
    if (regMap[REG_FW_STATUS] != FW_ST_VERIFY_OK) { fwFail(); return; }
    // Schedule the staged image for the bootloader; addFile auto-detects GZIP.
    picoOTA.begin();
    picoOTA.addFile(FW_STAGING_PATH);
    picoOTA.commit();
    LittleFS.end();
    rp2040.reboot(); // does not return
}

static void fwAbort() {
    if (fwFile) fwFile.close();
    fwSession = false;
    fwBufLen  = 0;
    LittleFS.remove(FW_STAGING_PATH);
    DigitalIOResumeInputs();
    regMap[REG_FW_STATUS] = FW_ST_IDLE;
}

void FWUpdateUpdate() {
    uint8_t cmd = fwCmdPending;
    if (cmd) {
        fwCmdPending = 0;
        switch (cmd) {
            case FW_CMD_BEGIN:  fwBegin();  break;
            case FW_CMD_END:    fwEnd();    break;
            case FW_CMD_COMMIT: fwCommit(); break;
            case FW_CMD_ABORT:  fwAbort();  break;
            default: /* ignore unknown command */ break;
        }
    }

    // Flush a streamed block once the burst has ended (CS deasserted).
    if (fwSession && fwBufLen > 0) {
        fwFlushBlock();
    }
}
