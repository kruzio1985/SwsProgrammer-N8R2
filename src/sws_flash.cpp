/* =============================================================================
 * sws_flash.cpp - flash po SWS przez rejestry MSPI ukladu docelowego.
 *
 * Sekwencje 1:1 z pvvx sources/UART2SWire/Source/main.c (funkcje read_flash,
 * _swire_fcmd_faddr, swire_fcmd) oraz Source/swire.c (swire_read_bytes).
 * =============================================================================
 */

#include "sws_flash.h"

#include <Arduino.h>

#include "sws_master.h"

#define R_MSPI_DATA   0x000Cu
#define R_MSPI_CTRL   0x000Du
#define R_SWIRE_MODE  0x00B3u

static uint32_t s_lastJedec = 0;
static bool     s_busyPoll  = false;
static uint32_t s_busyTimeouts = 0;

/* --- warstwa MSPI ---------------------------------------------------------- */
static inline bool fcsLow()  { return swsWriteReg8(R_MSPI_CTRL, 0x00); }
static inline bool fcsHigh() { return swsWriteReg8(R_MSPI_CTRL, SWS_MSPI_CS); }

/* Wysyla komende flash + 24-bitowy adres do rejestru danych MSPI.
 * Kazdy bajt osobną ramką - rejestr 0x0d jest pod 0x0d, wiec NIE wolno
 * pisac ich jednym blokiem (auto-inkrementacja trafilaby w rejestr kontrolny). */
static bool fcmdFaddr(uint8_t cmd, uint32_t addr) {
    if (!fcsLow()) return false;
    if (!swsWriteReg8(R_MSPI_DATA, cmd)) return false;
    if (!swsWriteReg8(R_MSPI_DATA, (uint8_t)(addr >> 16))) return false;
    if (!swsWriteReg8(R_MSPI_DATA, (uint8_t)(addr >> 8))) return false;
    return swsWriteReg8(R_MSPI_DATA, (uint8_t)addr);
}

static bool fcmd(uint8_t cmd) {
    if (!fcsLow()) return false;
    if (!swsWriteReg8(R_MSPI_DATA, cmd)) return false;
    return fcsHigh();
}

/* [0x0c] = 0 (start odczytu) + [0x0d] = RD|SDO (tryb auto-read). */
static bool startAutoRead() {
    const uint8_t p[2] = {0x00, (uint8_t)SWS_MSPI_AUTO_READ};
    return swsWriteReg(R_MSPI_DATA, p, 2);
}

static bool fifoMode(bool on) { return swsWriteReg8(R_SWIRE_MODE, on ? 0x80 : 0x00); }

/* --- JEDEC / status -------------------------------------------------------- */
bool swsFlashJedecId(uint32_t *id) {
    uint8_t b[3] = {0, 0, 0};

    if (!fcsLow()) return false;
    if (!swsWriteReg8(R_MSPI_DATA, SWS_FLASH_JEDEC_ID_CMD)) return false;
    if (!startAutoRead()) return false;
    if (!fifoMode(true)) return false;
    const bool ok = swsReadReg(R_MSPI_DATA, b, 3);
    fifoMode(false);
    fcsHigh();
    if (!ok) return false;

    s_lastJedec = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    if (id) *id = s_lastJedec;
    return true;
}

bool swsFlashStatus(uint8_t *status) {
    uint8_t s = 0;
    if (!fcsLow()) return false;
    if (!swsWriteReg8(R_MSPI_DATA, SWS_FLASH_STATUS_CMD)) return false;
    if (!startAutoRead()) return false;
    const bool ok = swsReadReg(R_MSPI_DATA, &s, 1);
    fcsHigh();
    if (!ok) return false;
    if (status) *status = s;
    return true;
}

bool swsFlashIsBusy(bool *busy) {
    uint8_t s = 0;
    if (!swsFlashStatus(&s)) return false;
    if (busy) *busy = (s & 0x01) != 0;
    return true;
}

/* --- odczyt ---------------------------------------------------------------- */
bool swsFlashRead(uint32_t addr, uint8_t *buf, uint32_t n) {
    if (!n) return true;
    if (!fcmdFaddr(SWS_FLASH_READ_CMD, addr)) return false;
    if (!startAutoRead()) return false;

    bool ok;
    if (n == 1) {
        ok = swsReadReg(R_MSPI_DATA, buf, 1);
    } else {
        if (!fifoMode(true)) return false;
        ok = swsReadReg(R_MSPI_DATA, buf, n);
        fifoMode(false);
    }

    fcsHigh();
    return ok;
}

/* --- kasowanie ------------------------------------------------------------- */
static bool waitReady(uint32_t timeoutMs) {
    const uint32_t t0 = millis();
    for (;;) {
        delay(2);
        uint8_t s = 0;
        if (!swsFlashStatus(&s)) return false;
        if (!(s & 0x01)) return true;
        if ((uint32_t)(millis() - t0) > timeoutMs) return false;
    }
}

bool swsFlashEraseSector(uint32_t addr) {
    if (!fcmd(SWS_FLASH_WREN_CMD)) return false;
    if (!fcmdFaddr(SWS_FLASH_ERASE4K_CMD, addr & ~(SWS_FLASH_SECTOR - 1))) return false;
    fcsHigh();
    return waitReady(5000);
}

bool swsFlashEraseAll() {
    if (!fcmd(SWS_FLASH_WREN_CMD)) return false;
    if (!fcmd(SWS_FLASH_ERASE_ALL_CMD)) return false;
    fcsHigh();
    return waitReady(120000);
}

/* --- zapis ----------------------------------------------------------------- */
bool swsFlashWritePage(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (!n || n > SWS_FLASH_PAGE) return false;
    if (addr & (SWS_FLASH_PAGE - 1)) return false;

    if (!fcmd(SWS_FLASH_WREN_CMD)) return false;
    if (!fcmdFaddr(SWS_FLASH_WRITE_CMD, addr)) return false;

    bool ok;
    if (n == 1) {
        ok = swsWriteReg8(R_MSPI_DATA, buf[0]);
    } else {
        if (!fifoMode(true)) return false;
        ok = swsWriteReg(R_MSPI_DATA, buf, n);
        fifoMode(false);
    }

    fcsHigh();
    if (!ok) return false;
    return waitReady(2000);
}

/* Zapis w calych stronach (256 B). Adres musi byc wyrownany do strony. */
bool swsFlashWriteRange(uint32_t addr, const uint8_t *buf, uint32_t n, bool eraseFirst) {
    if (addr & (SWS_FLASH_PAGE - 1)) return false;

    uint32_t done = 0;
    while (done < n) {
        const uint32_t cur = addr + done;

        if (eraseFirst && (cur & (SWS_FLASH_SECTOR - 1)) == 0) {
            if (!swsFlashEraseSector(cur)) return false;
        }

        uint32_t chunk = SWS_FLASH_PAGE;
        if (chunk > n - done) chunk = n - done;
        if (!swsFlashWritePage(cur, buf + done, chunk)) return false;

        done += chunk;
    }
    return true;
}

/* =============================================================================
 * Diagnostyka niskopoziomowa (komendy konsoli STAT/WEL/WRSR/FCMD/ERASE1/WRPG).
 *
 * Wzorcem jest SDK Telink:
 *     flash_send_cmd(): mspi_high(); sleep_us(1); mspi_low(); mspi_write(cmd);
 *                       mspi_wait();
 *     flash_wait_done(): po zboczu CS w gore -> odczyt [0x05] i petla po WIP
 *     mspi_wait():       czeka na skasowanie FLD_MSPI_BUSY = bit4 [0x0d]
 *
 * Uwaga na czas: jedna ramka odczytu SWS to ~1 ms, wiec petla po BUSY ma
 * rozdzielczosc ~1 ms. Jesli BUSY wisi (sprzet nie konczy bajtu), zobaczymy
 * timeout - i to jest informacja diagnostyczna, a nie blad pomijalny.
 * =============================================================================
 */
bool swsMspiCtrlRead(uint8_t *v) { return swsReadReg(R_MSPI_CTRL, v, 1); }

bool swsMspiCtrlWrite(uint8_t v, uint8_t *readback) {
    if (!swsWriteReg8(R_MSPI_CTRL, v)) return false;
    if (!readback) return true;
    return swsReadReg(R_MSPI_CTRL, readback, 1);
}

bool swsMspiWaitBusy(uint32_t timeoutMs, uint32_t *waitedMs) {
    const uint32_t t0 = millis();
    for (;;) {
        uint8_t v = 0;
        if (!swsReadReg(R_MSPI_CTRL, &v, 1)) return false;
        if (!(v & SWS_MSPI_BUSY)) {
            if (waitedMs) *waitedMs = millis() - t0;
            return true;
        }
        if ((uint32_t)(millis() - t0) > timeoutMs) {
            if (waitedMs) *waitedMs = millis() - t0;
            return false;
        }
    }
}

void swsSetBusyPoll(bool on) { s_busyPoll = on; }
bool swsGetBusyPoll() { return s_busyPoll; }
uint32_t swsBusyTimeoutCount() { return s_busyTimeouts; }

/* Zapis bajtu do [0x0c]; przy wlaczonym BPOLL najpierw czekamy na BUSY=0
 * (odpowiednik mspi_wait() z SDK). Timeout nie przerywa operacji, tylko
 * jest liczony - inaczej diagnostyka zawiesilaby sie na wiszacym bicie. */
static bool mspiWriteByte(uint8_t b) {
    if (s_busyPoll && !swsMspiWaitBusy(50, nullptr)) s_busyTimeouts++;
    return swsWriteReg8(R_MSPI_DATA, b);
}

bool swsFlashRawTx(const uint8_t *bytes, uint32_t n, uint8_t *ctrlLo, uint8_t *ctrlHi) {
    if (!swsMspiCtrlWrite(SWS_MSPI_CS, nullptr)) return false;  /* CS w gore */
    delayMicroseconds(2);                                       /* sleep_us(1) z SDK */
    if (!swsMspiCtrlWrite(0x00, ctrlLo)) return false;          /* CS w dol */
    for (uint32_t i = 0; i < n; i++) {
        if (!mspiWriteByte(bytes[i])) return false;
    }
    return swsMspiCtrlWrite(SWS_MSPI_CS, ctrlHi);               /* CS w gore = wykonanie */
}

bool swsFlashRawTxRead(const uint8_t *bytes, uint32_t n, uint8_t *out, uint32_t no) {
    if (!swsMspiCtrlWrite(SWS_MSPI_CS, nullptr)) return false;
    delayMicroseconds(2);
    if (!swsMspiCtrlWrite(0x00, nullptr)) return false;
    for (uint32_t i = 0; i < n; i++) {
        if (!mspiWriteByte(bytes[i])) return false;
    }

    bool ok = true;
    if (no) {
        if (!swsWriteReg8(R_MSPI_DATA, 0x00)) return false;      /* dummy: zegar */
        if (!swsWriteReg8(R_MSPI_CTRL, SWS_MSPI_AUTO_READ)) return false;
        if (no == 1) {
            ok = swsReadReg(R_MSPI_DATA, out, 1);
        } else {
            if (!fifoMode(true)) return false;
            ok = swsReadReg(R_MSPI_DATA, out, no);
            fifoMode(false);
        }
    }
    fcsHigh();
    return ok;
}

bool swsFlashStatusCmd(uint8_t cmd, uint8_t *out, uint32_t n, bool sdkStyle) {
    if (!n) return true;

    if (!sdkStyle) {
        if (!fcsLow()) return false;
        if (!swsWriteReg8(R_MSPI_DATA, cmd)) return false;
        if (!startAutoRead()) return false;
        bool ok;
        if (n == 1) {
            ok = swsReadReg(R_MSPI_DATA, out, 1);
        } else {
            if (!fifoMode(true)) return false;
            ok = swsReadReg(R_MSPI_DATA, out, n);
            fifoMode(false);
        }
        fcsHigh();
        return ok;
    }

    /* Metoda z SDK: kazdy bajt to "mspi_write(0) -> zegar" i odczyt [0x0c].
     * Dzieli z nami tylko [0x00b2]; sluzy do porownania obu sciezek odczytu. */
    if (!swsMspiCtrlWrite(SWS_MSPI_CS, nullptr)) return false;
    delayMicroseconds(2);
    if (!swsMspiCtrlWrite(0x00, nullptr)) return false;
    if (!swsWriteReg8(R_MSPI_DATA, cmd)) return false;
    for (uint32_t i = 0; i < n; i++) {
        if (!swsWriteReg8(R_MSPI_DATA, 0x00)) return false;
        if (!swsReadReg(R_MSPI_DATA, &out[i], 1)) return false;
    }
    fcsHigh();
    return true;
}

bool swsFlashWren() {
    const uint8_t b[1] = {SWS_FLASH_WREN_CMD};
    return swsFlashRawTx(b, 1, nullptr, nullptr);
}

bool swsFlashWriteStatusReg(uint8_t val) {
    const uint8_t b[2] = {SWS_FLASH_WRSR_CMD, val};
    if (!swsFlashWren()) return false;
    return swsFlashRawTx(b, 2, nullptr, nullptr);
}
