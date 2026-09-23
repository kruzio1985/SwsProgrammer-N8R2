/* =============================================================================
 * spi_flash.h - dostep do zewnetrznej kosci SPI NOR flash (25xx, np. W25Q/MX25L).
 *
 * Tryb "programatora uniwersalnego" bez adapterow: kosc (wylutowana albo
 * klips SOIC-8) podlaczamy bezposrednio do pinow ESP32-S3:
 *
 *      ESP32-S3                 SPI flash (SOIC-8)
 *      SPI_SCK_PIN   (CLK)  --- pin 6  CLK
 *      SPI_MISO_PIN  (MISO) --- pin 2  DO  (kosc -> ESP32)
 *      SPI_MOSI_PIN  (MOSI) --- pin 5  DI  (ESP32 -> kosc)
 *      SPI_CS_PIN    (CS)   --- pin 1  CS
 *      3V3                   --- pin 8  VCC
 *      GND                   --- pin 4  GND
 *
 * Adresacja 3-bajtowa (do 16 MB). Kosc wieksza niz 16 MB wymaga trybu 4-bajtowego
 * i nie jest tu obslugiwana (wystarczajaco dla BIOS/SPI Flash 1..16 MB).
 * =============================================================================
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define SPI_FLASH_PAGE      256u
#define SPI_FLASH_SECTOR    4096u
/* Adresacja 3-bajtowa = max 16 MB. */
#define SPI_FLASH_MAX_ADDR  (16u * 1024u * 1024u)

/* Inicjalizacja pinow + SPI (bezpieczna do wielokrotnego wywolania). */
void spiFlashBegin();

/* JEDEC ID (9F): producent + typ + pojemnosc. false = kosc nie odpowiada. */
bool spiFlashJedecId(uint32_t *id);

/* Rozmiar kosci w bajtach na podstawie bajtu pojemnosci z JEDEC ID. 0 = nieznany. */
uint32_t spiFlashChipSize(uint32_t jedec);

/* Nazwa kosci (producent + pojemnosc) na podstawie JEDEC ID. */
const char *spiFlashJedecName(uint32_t jedec);

/* Sprawdza, czy obszar jest czysty (same 0xFF). Zwraca true, gdy caly obszar
 * da sie odczytac; *nonBlank = liczba bajtow != 0xFF, *firstNonBlank = adres
 * pierwszego takiego bajtu (0xFFFFFFFF, gdy brak). */
bool spiFlashBlankCheck(uint32_t addr, uint32_t len,
                        uint32_t *firstNonBlank, uint32_t *nonBlankCount);

/* Odczyt n bajtow (komenda 03). */
bool spiFlashRead(uint32_t addr, uint8_t *buf, uint32_t n);

/* Porownanie danych z zawartoscia kosci. Zwraca false tylko przy bledzie
 * odczytu; niezgodne bajty liczy do *badCount (0 = zgodne). */
bool spiFlashVerify(uint32_t addr, const uint8_t *data, uint32_t len,
                    uint32_t *firstBad, uint32_t *badCount);

/* Kasowanie sektora 4 kB (20). */
bool spiFlashEraseSector(uint32_t addr);

/* Kasowanie calej kosci (C7). Moze trwac kilkadziesiat sekund. */
bool spiFlashEraseChip();

/* Zapis strony (02), max 256 B, bez kasowania. */
bool spiFlashWritePage(uint32_t addr, const uint8_t *buf, uint32_t n);

/* Kasowanie sektorow + zapis stronami calego obszaru (jednorazowy bufor). */
bool spiFlashWriteRange(uint32_t addr, const uint8_t *buf, uint32_t n);
