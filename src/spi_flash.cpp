/* =============================================================================
 * spi_flash.cpp - SPI NOR flash (25xx) na ESP32-S3.
 *
 * Standardowe komendy 25-series: 9F (JEDEC), 03 (read), 06 (WREN), 05 (RDSR),
 * 02 (page program), 20 (sector erase 4K), C7 (chip erase).
 * =============================================================================
 */

#include <Arduino.h>
#include <SPI.h>

#include "config.h"
#include "spi_flash.h"

static bool begun = false;

void spiFlashBegin() {
    if (begun) return;
    pinMode(SPI_CS_PIN, OUTPUT);
    digitalWrite(SPI_CS_PIN, HIGH);
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
    SPI.beginTransaction(SPISettings(SPI_SPEED_HZ, MSBFIRST, SPI_MODE0));
    begun = true;
}

static void csLow()  { digitalWrite(SPI_CS_PIN, LOW); }
static void csHigh() { digitalWrite(SPI_CS_PIN, HIGH); }

static void sendAddr24(uint32_t addr) {
    SPI.transfer((addr >> 16) & 0xFF);
    SPI.transfer((addr >> 8) & 0xFF);
    SPI.transfer(addr & 0xFF);
}

bool spiFlashJedecId(uint32_t *id) {
    spiFlashBegin();
    csLow();
    SPI.transfer(0x9F);
    uint8_t m = SPI.transfer(0x00);
    uint8_t t = SPI.transfer(0x00);
    uint8_t c = SPI.transfer(0x00);
    csHigh();
    *id = ((uint32_t)m << 16) | ((uint32_t)t << 8) | c;
    return !(*id == 0x000000 || *id == 0xFFFFFF);
}

uint32_t spiFlashChipSize(uint32_t jedec) {
    switch (jedec & 0xFF) {
        case 0x10: return 64u * 1024u;        // 512 kbit
        case 0x11: return 128u * 1024u;       // 1 Mbit
        case 0x12: return 256u * 1024u;       // 2 Mbit
        case 0x13: return 512u * 1024u;       // 4 Mbit
        case 0x14: return 1024u * 1024u;      // 8 Mbit
        case 0x15: return 2u * 1024u * 1024u; // 16 Mbit
        case 0x16: return 4u * 1024u * 1024u; // 32 Mbit
        case 0x17: return 8u * 1024u * 1024u; // 64 Mbit
        case 0x18: return 16u * 1024u * 1024u;// 128 Mbit
        case 0x19: return 32u * 1024u * 1024u;// 256 Mbit (adres. 4B!)
        case 0x20: return 64u * 1024u * 1024u;// 512 Mbit (adres. 4B!)
        case 0x21: return 128u * 1024u * 1024u;// 1 Gbit (adres. 4B!)
        default:   return 0;
    }
}

static const char *spiFlashVendorName(uint8_t id) {
    switch (id) {
        case 0x01: return "Spansion/Cypress";
        case 0x0B: return "XMC";
        case 0x1C: return "EON";
        case 0x20: return "Micron/Numonyx/ST";
        case 0x37: return "AMIC";
        case 0x40: return "GigaDevice";
        case 0x51: return "GigaDevice";
        case 0x5E: return "Zbit";
        case 0x68: return "Boya";
        case 0x70: return "Puya";
        case 0x7F: return "ISSI";
        case 0x81: return "ISSI";
        case 0x8C: return "ESMT";
        case 0x9D: return "ISSI/PMC";
        case 0xA1: return "Fudan";
        case 0xAD: return "NexFlash";
        case 0xBF: return "Microchip/SST";
        case 0xC2: return "Macronix";
        case 0xC8: return "GigaDevice";
        case 0xD5: return "Fidelix";
        case 0xEF: return "Winbond";
        default:   return "nieznany producent";
    }
}

const char *spiFlashJedecName(uint32_t jedec) {
    static char buf[64];
    uint32_t sz = spiFlashChipSize(jedec);
    const char *vend = spiFlashVendorName((jedec >> 16) & 0xFF);
    if (sz >= 1024u * 1024u) {
        snprintf(buf, sizeof(buf), "%s (%u MB)", vend, (unsigned)(sz / (1024u * 1024u)));
    } else if (sz) {
        snprintf(buf, sizeof(buf), "%s (%u KB)", vend, (unsigned)(sz / 1024u));
    } else {
        snprintf(buf, sizeof(buf), "%s (rozmiar nieznany)", vend);
    }
    return buf;
}

bool spiFlashBlankCheck(uint32_t addr, uint32_t len,
                        uint32_t *firstNonBlank, uint32_t *nonBlankCount) {
    *firstNonBlank = 0xFFFFFFFF;
    *nonBlankCount = 0;
    if (len == 0 || (uint64_t)addr + len > SPI_FLASH_MAX_ADDR) return false;

    uint8_t tmp[SPI_FLASH_PAGE];
    uint32_t off = 0;
    while (off < len) {
        uint32_t c = len - off < sizeof(tmp) ? len - off : (uint32_t)sizeof(tmp);
        if (!spiFlashRead(addr + off, tmp, c)) return false;
        for (uint32_t i = 0; i < c; i++) {
            if (tmp[i] != 0xFF) {
                if (*nonBlankCount == 0) *firstNonBlank = addr + off + i;
                (*nonBlankCount)++;
            }
        }
        off += c;
    }
    return true;
}

bool spiFlashRead(uint32_t addr, uint8_t *buf, uint32_t n) {
    if (addr >= SPI_FLASH_MAX_ADDR || n == 0) return false;
    spiFlashBegin();
    csLow();
    SPI.transfer(0x03);
    sendAddr24(addr);
    for (uint32_t i = 0; i < n; i++) buf[i] = SPI.transfer(0x00);
    csHigh();
    return true;
}

static bool waitBusy(uint32_t timeoutMs) {
    uint32_t t0 = millis();
    while (true) {
        csLow();
        SPI.transfer(0x05);
        uint8_t sr = SPI.transfer(0x00);
        csHigh();
        if (!(sr & 0x01)) return true;
        if ((int32_t)(millis() - t0) > (int32_t)timeoutMs) return false;
        delay(1);
    }
}

static bool writeEnable() {
    csLow();
    SPI.transfer(0x06);
    csHigh();
    return true;
}

bool spiFlashEraseSector(uint32_t addr) {
    if (addr >= SPI_FLASH_MAX_ADDR) return false;
    spiFlashBegin();
    writeEnable();
    csLow();
    SPI.transfer(0x20);
    sendAddr24(addr);
    csHigh();
    return waitBusy(2000);
}

bool spiFlashEraseChip() {
    spiFlashBegin();
    writeEnable();
    csLow();
    SPI.transfer(0xC7);
    csHigh();
    return waitBusy(120000);
}

bool spiFlashWritePage(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (addr >= SPI_FLASH_MAX_ADDR || n == 0 || n > SPI_FLASH_PAGE) return false;
    spiFlashBegin();
    writeEnable();
    csLow();
    SPI.transfer(0x02);
    sendAddr24(addr);
    for (uint32_t i = 0; i < n; i++) SPI.transfer(buf[i]);
    csHigh();
    return waitBusy(2000);
}

bool spiFlashWriteRange(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (addr >= SPI_FLASH_MAX_ADDR || (uint64_t)addr + n > SPI_FLASH_MAX_ADDR) return false;

    uint32_t s = addr & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    uint32_t e = (addr + n + SPI_FLASH_SECTOR - 1) & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    for (uint32_t a = s; a < e; a += SPI_FLASH_SECTOR) {
        if (!spiFlashEraseSector(a)) return false;
    }

    uint32_t off = 0;
    while (off < n) {
        uint32_t c = n - off;
        uint32_t within = SPI_FLASH_PAGE - ((addr + off) % SPI_FLASH_PAGE);
        if (c > within) c = within;
        if (!spiFlashWritePage(addr + off, buf + off, c)) return false;
        off += c;
    }
    return true;
}

bool spiFlashVerify(uint32_t addr, const uint8_t *data, uint32_t len,
                    uint32_t *firstBad, uint32_t *badCount) {
    uint8_t tmp[SPI_FLASH_PAGE];
    *firstBad = 0xFFFFFFFF;
    *badCount = 0;
    uint32_t off = 0;
    while (off < len) {
        uint32_t c = len - off < sizeof(tmp) ? len - off : (uint32_t)sizeof(tmp);
        if (!spiFlashRead(addr + off, tmp, c)) return false;
        for (uint32_t i = 0; i < c; i++) {
            if (tmp[i] != data[off + i]) {
                if (*badCount == 0) *firstBad = addr + off + i;
                (*badCount)++;
            }
        }
        off += c;
    }
    return true;
}
