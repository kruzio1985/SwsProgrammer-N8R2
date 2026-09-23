/* =============================================================================
 * sd_card.h - obsluga karty microSD na osobnym, drugim SPI (HSPI).
 *
 * Modul Catalex microSD (AMS1117 + 74ABT125) - patrz config.h po piny.
 * System plikow: FAT16/FAT32 (biblioteka SD/FFat z rdzenia ESP32).
 * =============================================================================
 */
#pragma once

#include <Arduino.h>
#include <SD.h>

/* Leniwy, bezpieczny do wielokrotnego wywolania init. Zwraca true = karta
 * zamontowana (FAT16/FAT32). */
bool sdCardBegin();

/* Czy karta jest zamontowana i widoczna. */
bool sdCardMounted();

/* Diagnostyka: wymusza swiezy init, wypisuje stan pinow i wynik na out.
 * Zwraca true = karta widoczna. */
bool sdCardProbe(Print &out);

/* Typ karty jako nazwa ("MMC" / "SDSC" / "SDHC/SDXC" / "brak"). */
const char *sdCardTypeName();

uint64_t sdCardTotalBytes();
uint64_t sdCardUsedBytes();

/* Lista plikow/katalogow (path = "" lub "/" = katalog glowny). */
bool sdCardLs(Print &out, const char *path);

/* Wypisz zawartosc pliku jako tekst na out. */
bool sdCardCat(Print &out, const char *path);

/* Zapis lub dopisanie danych do pliku (FILE_WRITE / FILE_APPEND). */
bool sdCardWrite(const char *path, const uint8_t *data, size_t n, bool append);

/* Usun plik. */
bool sdCardDelete(const char *path);

/* Sformatuj karte (FAT) i ponownie zamontuj. Zwraca true = gotowa do uzycia.
 * Wynik / kod bledu FatFS jest wypisywany na out. */
bool sdCardFormat(Print &out);
