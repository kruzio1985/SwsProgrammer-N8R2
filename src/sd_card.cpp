/* =============================================================================
 * sd_card.cpp - karta microSD przez drugi kontroler SPI (HSPI), osobne piny.
 * =============================================================================
 */
#include "sd_card.h"
#include "config.h"
#include "driver/gpio.h"

/* FatFS z biblioteki SD - uzywane tylko do jawnego formatowania karty. */
extern "C" {
#include "ff.h"
}

/* Drugi SPI (HSPI = 1) - pierwszy (FSPI = 0) zajmuje SPI flash 25xx. */
static SPIClass sdcardSpi(HSPI);
static bool sdcardTried = false;

bool sdCardBegin() {
    if (sdcardTried) {
        return SD.cardType() != CARD_NONE;
    }
    sdcardTried = true;

    /* 4 MHz - klips/przewody od modulu Catalex bywaja dlugie. */
    sdcardSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    bool ok = SD.begin(SD_CS_PIN, sdcardSpi, 4000000, "/sd", 5, false);
    if (!ok || SD.cardType() == CARD_NONE) {
        /* Jesli 4 MHz nie starcza (dlugie przewody), sprobuj wolniej. */
        SD.end();
        sdcardSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
        ok = SD.begin(SD_CS_PIN, sdcardSpi, 400000, "/sd", 5, false);
    }
    if (!ok || SD.cardType() == CARD_NONE) {
        SD.end();
        return false;
    }
    return true;
}

/* Twardy, jednoznaczny test zwarcia do GND: pin jako OUTPUT ustawiamy na 1
 * i natychmiast czytamy RZECZYWISTY poziom padu (GPIO.in - nie latch wyjscia).
 * Wysokoimpedancyjna linia (luzna/wolna) utrzyma 1. Twarde zwarcie sciagnie do 0
 * nawet w trakcie aktywnego sterowania HIGH. */
static int sdPinDriveRead(uint8_t pin) {
    gpio_num_t g = (gpio_num_t)pin;
    gpio_reset_pin(g);
    /* Wejscie i wyjscie naraz - inaczej bufor wejsciowy jest wylaczony
     * i gpio_get_level() czyta 0 mimo sterowania HIGH. */
    gpio_set_direction(g, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(g, 1);
    delayMicroseconds(20);
    int v = gpio_get_level(g);      /* faktyczne napiecie padu */
    gpio_reset_pin(g);
    return v;
}

/* Stan linii bez posrednictwa Arduino pinMode (czysty sterownik GPIO). */
static int sdPinLevel(uint8_t pin, gpio_pull_mode_t pull) {
    gpio_num_t g = (gpio_num_t)pin;
    gpio_reset_pin(g);
    gpio_set_direction(g, GPIO_MODE_INPUT);
    gpio_set_pull_mode(g, pull);
    delayMicroseconds(100);         /* czas na ustabilizowanie pull-up/down */
    int v = gpio_get_level(g);
    gpio_reset_pin(g);
    return v;
}

/* Diagnostyka: wymusza swiezy init od zera (karta mogla byc wlozona pozniej)
 * i wypisuje stan pinow oraz wynik. Zwraca true = karta widoczna. */
bool sdCardProbe(Print &out) {
    SD.end();
    sdcardSpi.end();
    sdcardTried = false;

    out.print("Piny SD (pull-up, 1=HIGH): CS=");
    out.print(sdPinLevel(SD_CS_PIN, GPIO_PULLUP_ONLY));
    out.print(" MISO=");
    out.print(sdPinLevel(SD_MISO_PIN, GPIO_PULLUP_ONLY));
    out.print(" MOSI=");
    out.print(sdPinLevel(SD_MOSI_PIN, GPIO_PULLUP_ONLY));
    out.print(" SCK=");
    out.print(sdPinLevel(SD_SCK_PIN, GPIO_PULLUP_ONLY));
    out.print("\r\n");

    out.print("Piny SD (pull-down, 1=HIGH): CS=");
    out.print(sdPinLevel(SD_CS_PIN, GPIO_PULLDOWN_ONLY));
    out.print(" MISO=");
    out.print(sdPinLevel(SD_MISO_PIN, GPIO_PULLDOWN_ONLY));
    out.print(" MOSI=");
    out.print(sdPinLevel(SD_MOSI_PIN, GPIO_PULLDOWN_ONLY));
    out.print(" SCK=");
    out.print(sdPinLevel(SD_SCK_PIN, GPIO_PULLDOWN_ONLY));
    out.print("\r\n");

    out.print("Test zwarcia (wymuszony HIGH, 1=linia wolna): CS=");
    out.print(sdPinDriveRead(SD_CS_PIN));
    out.print(" MISO=");
    out.print(sdPinDriveRead(SD_MISO_PIN));
    out.print(" MOSI=");
    out.print(sdPinDriveRead(SD_MOSI_PIN));
    out.print(" SCK=");
    out.print(sdPinDriveRead(SD_SCK_PIN));
    out.print("\r\n");

    if (sdCardBegin()) {
        out.print("+OK SD karta widoczna: ");
        out.print(sdCardTypeName());
        out.print(", pojemnosc=");
        out.print((unsigned long long)sdCardTotalBytes());
        out.print(" B\r\n");
        return true;
    }
    out.print("-ERR SD: karta nie odpowiada po SPI (brak CMD0).\r\n");
    out.print("       Linie elektrycznie OK (brak zwarcia) - problem z danymi/zasilaniem.\r\n");
    out.print("       Sprawdz po kolei:\r\n");
    out.print("       1) zamien MISO i MOSI (najczestsza przyczyna)\r\n");
    out.print("       2) CS=21 SCK=14 MOSI=16 MISO=15 i GND wspolny z ESP\r\n");
    out.print("       3) zasilanie modulu 5V (nie 3,3V) na VCC\r\n");
    out.print("       4) inna karta (format FAT32)\r\n");
    return false;
}

bool sdCardMounted() {
    return sdcardTried && SD.cardType() != CARD_NONE;
}

const char *sdCardTypeName() {
    switch (SD.cardType()) {
        case CARD_MMC:   return "MMC";
        case CARD_SD:    return "SDSC";
        case CARD_SDHC:  return "SDHC/SDXC";
        default:         return "brak";
    }
}

uint64_t sdCardTotalBytes() {
    return sdCardMounted() ? SD.totalBytes() : 0;
}

uint64_t sdCardUsedBytes() {
    return sdCardMounted() ? SD.usedBytes() : 0;
}

bool sdCardLs(Print &out, const char *path) {
    if (!sdCardMounted()) return false;
    const char *p = (path && path[0]) ? path : "/";
    File dir = SD.open(p);
    if (!dir) return false;
    if (!dir.isDirectory()) {
        dir.close();
        return false;
    }
    File f = dir.openNextFile();
    while (f) {
        char line[64];
        snprintf(line, sizeof(line), "%-32s %10llu B%s\r\n",
                 f.name(), (unsigned long long)f.size(),
                 f.isDirectory() ? " <DIR>" : "");
        out.print(line);
        f = dir.openNextFile();
    }
    return true;
}

bool sdCardCat(Print &out, const char *path) {
    if (!sdCardMounted() || !path || !path[0]) return false;
    File f = SD.open(path, FILE_READ);
    if (!f) return false;
    if (f.isDirectory()) {
        f.close();
        return false;
    }
    uint8_t buf[256];
    while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n > 0) out.write(buf, (size_t)n);
    }
    f.close();
    return true;
}

bool sdCardWrite(const char *path, const uint8_t *data, size_t n, bool append) {
    if (!sdCardMounted() || !path || !path[0]) return false;
    File f = SD.open(path, append ? FILE_APPEND : FILE_WRITE);
    if (!f) return false;
    size_t w = f.write(data, n);
    f.close();
    return w == n;
}

bool sdCardDelete(const char *path) {
    if (!sdCardMounted() || !path || !path[0]) return false;
    return SD.remove(path);
}

bool sdCardFormat(Print &out) {
    /* Bez karty nie probuj formatowac - f_mkfs na nieobecnym dysku moze
     * zawiesic petle glownej. */
    if (SD.cardType() == CARD_NONE) {
        out.print("-ERR brak karty SD\r\n");
        return false;
    }

    /* Odmontuj TYLKO wolumin FatFS, nie deinicjalizuj SPI/sterownika dysku.
     * SD.end() zabilby sterownik (sdcard_uninit) i f_mkfs zwrocilby
     * FR_NOT_READY, bo nie mialby jak rozmawiac z karta. */
    FRESULT um = f_mount(NULL, "0:", 0);
    (void)um;

    BYTE work[FF_MAX_SS];
    FRESULT res = f_mkfs("0:", FM_ANY, 0, work, sizeof(work));
    if (res != FR_OK) {
        out.print("-ERR f_mkfs zwrocil FRESULT=");
        out.print((int)res);
        out.print("\r\n");
    }

    /* Teraz pelny demontaz (odrejestrowanie VFS i sterownika) i swiezy,
     * pelny montaz - bez auto-formatowania przy starcie. */
    SD.end();
    sdcardTried = false;
    sdcardSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    bool ok = SD.begin(SD_CS_PIN, sdcardSpi, 4000000, "/sd", 5, false);
    if (!ok) {
        SD.end();
        out.print("-ERR ponowny montaz karty nieudany\r\n");
    }
    return (res == FR_OK) && ok && SD.cardType() != CARD_NONE;
}
