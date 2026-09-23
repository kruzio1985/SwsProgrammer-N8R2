/* =============================================================================
 * SWS Programmer - programator Telink TLSR825x (Tuya TS0201) na ESP32-S3.
 *
 * Konsola: Serial (UART0, GPIO43/44) -> konwerter CH343 -> COM11 @ 115200.
 * Cel (układ): Serial1 - UART danych docelowego ukladu (patrz config.h).
 *
 * Komendy (każda kończy się \n):
 *   HELP                    - ta lista
 *   PINS                    - aktualne piny
 *   SWSTEST [div]           - test lacza SWS: zapis + odczyt [0x00b2] (bez flasha)
 *   CHIP                    - aktywuj cel, test SWS, odczytaj JEDEC flash
 *   JEDEC                   - ID kosci flash
 *   PROBE                   - sonda sprzetowa: stan linii + nasluch UART
 *   VMEAS <pin>             - napiecie na pinie ADC1 (1..10) w mV
 *   VSCAN                   - napiecia na wszystkich pinach ADC1 1..10
 *   READ <adr> <len>        - zrzuć pamięć flash (binarnie po +DATA)
 *   ERASE <start> <len>     - skasuj sektory 4KB w zakresie
 *   WRITE <rozmiar> [adr]   - odbierz i wgraj obraz .bin (z weryfikacją)
 *   RESET                   - reset celu (uruchom wgrany firmware)
 *   LISTEN [ms] [baud] [NONE|RST|GO]
 *                           - zrzut tego, co uklad nadaje po UART (tekst + hex)
 *   READW <adr> <len>       - odczyt flash bajt po bajcie (pewny, ~0,5 ms/B)
 *   UNIT <us>               - czas jednostki SWS w us (domyslnie 4, zakres 1-4)
 *   PHASE <div>             - wybor fazy probkowania SWS
 *   WIDTHS                  - pomiar szerokosci zboczy SWS
 *   WAVE                    - podglad przebiegu SWS
 *
 * Diagnostyka flasha (nic nie zmieniaja poza ERASE1/WRPG/WRSR/FCMD/FCMDR):
 *   STAT                    - SR1/SR2/SR3 kosci + rejestry MSPI + stan CPU
 *                             (+ test, czy sprzet przyjmuje zapisy [0x0d])
 *   WEL                     - WREN + kontrola bitu WEL (czy WREN dochodzi)
 *   WRSR <hex>              - zapis rejestru statusu (zdjecie blokad BP/SRP)
 *   FCMD <hex...>           - surowa transakcja MSPI do kosci
 *   FCMDR <n> <hex...>      - jak FCMD + odczyt n bajtow
 *   ERASE1 <adr>            - kasowanie 1 sektora + pelna diagnostyka
 *   WRPG <adr> <hex...>     - zapis 1..256 B + odczyt kontrolny
 *   MSPIR <adr> [n]         - odczyt rejestru/SRAM ukladu (np. 40000)
 *   MSPIW <adr> <hex...>    - zapis rejestru/SRAM ukladu + odczyt kontrolny
 *   SRAMTEST [n]            - test bledow lacza SWS na SRAM (bez flasha)
 *   MEMTEST [KB]            - test PSRAM + SRAM ESP32 (wzorce, walking)
 *   CAP <pin> [n] [div]     - logic analyzer: n probek do PSRAM + RLE
 *   OSC <pin> [n] [us]      - mini-oscyloskop ADC: n probek + statystyki
 *   MSPIWATCH [ms]          - czy rejestry MSPI zmieniaja sie same
 *   HALT [ms]               - aktywacja + weryfikacja [0x0602]=0x05
 *   BPOLL <0|1>             - czekanie na BUSY przed zapisem bajtu (jak SDK)
 *   RDHEX <adr> <len>       - podglad flasha w hex (max 256 B)
 *   SPI/SPIREAD/SPIWRITE... - zewnetrzna kosc SPI flash 25xx (klips SOIC-8)
 *   SD/SDLS/SDCAT/SDAPPEND/SDDEL
 *                           - karta microSD (modul Catalex, osobny SPI)
 * =============================================================================
 */

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#include "config.h"
#include "sws_flash.h"
#include "sws_master.h"
#include "spi_flash.h"
#include "sd_card.h"
#include "webui.h"
#include <WiFi.h>

/* --- globalne wyjscie (konsola lub HTTP) ------------------------------------ */
/* TeePrint: Serial (COM11) + bufor logów WWW jednocześnie. */
static TeePrint g_tee;
Print *g_out = &g_tee;

/* printf na biezace wyjscie (konsola lub HTTP). */
static void outPrintf(const char *fmt, ...) {
    char tmp[192];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    g_out->write((const uint8_t *)tmp, strlen(tmp));
}

/* --- bufory i stan ---------------------------------------------------------- */
static char lineBuf[1024];
static size_t lineLen = 0;

static bool rxActive = false;      // trwa odbiór obrazu po WRITE
static uint32_t rxRemaining = 0;   // ile jeszcze bajtów odebrać
static uint32_t rxAddr = 0;        // bieżący adres zapisu w flash

static bool spiRxActive = false;      // trwa odbiór obrazu po SPIWRITE
static uint32_t spiRxRemaining = 0;
static uint32_t spiRxAddr = 0;

/* --- małe pomoce ------------------------------------------------------------- */
static void swsLog(const char *m) {
    g_out->print(m);
    g_out->print("\r\n");
}

/* Podział linii na tokeny oddzielone spacjami (modyfikuje bufor). */
static char *nextTok(char *&p) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == 0) return nullptr;
    char *s = p;
    while (*p && *p != ' ' && *p != '\t') p++;
    if (*p) *p++ = 0;
    return s;
}

static bool parseU32(const char *s, uint32_t &v) {
    if (!s || !*s) return false;
    char *end = nullptr;
    v = (uint32_t)strtoul(s, &end, 0);
    return end && *end == 0;
}

/* --- obsługa zapisu (streaming) --------------------------------------------- */
static void abortWrite(const char *msg) {
    rxActive = false;
    g_out->print("-ERR ");
    g_out->print(msg);
    g_out->print("\r\n");
}

static void finishWrite() {
    rxActive = false;
    g_out->print("+DONE\r\n");
    swsLog("wgranie i weryfikacja zakonczone - reset celu");
    swsResetTargetPulse(50);
}

static void handleRxChunk() {
    if (rxRemaining == 0) {
        finishWrite();
        return;
    }

    uint32_t n = rxRemaining < SWS_SWS_BURST ? rxRemaining : (uint32_t)SWS_SWS_BURST;
    uint8_t buf[SWS_SWS_BURST];

    /* Odbiór jednego bloku. Host czeka na +OK przed wysłaniem kolejnego,
       więc timeout liczony jest osobno dla każdego bloku. */
    uint32_t got = 0;
    uint32_t deadline = millis() + 20000;
    while (got < n) {
        if (Serial.available()) {
            buf[got++] = (uint8_t)Serial.read();
            deadline = millis() + 20000;
        } else if ((int32_t)(millis() - deadline) > 0) {
            abortWrite("timeout przy odbiorze danych");
            return;
        } else {
            delay(1);
        }
    }

    /* Pierwsza strona sektora = kasowanie sektora przed zapisem. */
    if (rxAddr % SWS_FLASH_SECTOR == 0) {
        if (!swsFlashEraseSector(rxAddr)) {
            abortWrite("kasowanie sektora nieudane");
            return;
        }
    }

    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        ok = swsFlashWriteRange(rxAddr, buf, n, false);
        if (ok) {
            static uint8_t chk[SWS_SWS_BURST];
            ok = swsFlashRead(rxAddr, chk, n) && memcmp(chk, buf, n) == 0;
        }
    }

    if (!ok) {
        abortWrite("zapis+weryfikacja nieudane");
        return;
    }

    rxAddr += n;
    rxRemaining -= n;
    g_out->print("+OK ");
    g_out->print(rxAddr, HEX);
    g_out->print("\r\n");

    if (rxRemaining == 0) finishWrite();
}

/* --- obsługa zapisu SPI (streaming, bez limitu RAM) -------------------------- */
static void spiAbortWrite(const char *msg) {
    spiRxActive = false;
    g_out->print("-ERR ");
    g_out->print(msg);
    g_out->print("\r\n");
}

static void spiFinishWrite() {
    spiRxActive = false;
    g_out->print("+DONE\r\n");
    g_out->print("SPI: zapis i weryfikacja zakonczone\r\n");
}

static void spiHandleRxChunk() {
    if (spiRxRemaining == 0) {
        spiFinishWrite();
        return;
    }

    uint32_t n = spiRxRemaining < 256 ? spiRxRemaining : 256;
    uint8_t buf[256];

    uint32_t got = 0;
    uint32_t deadline = millis() + 20000;
    while (got < n) {
        if (Serial.available()) {
            buf[got++] = (uint8_t)Serial.read();
            deadline = millis() + 20000;
        } else if ((int32_t)(millis() - deadline) > 0) {
            spiAbortWrite("SPI timeout przy odbiorze danych");
            return;
        } else {
            delay(1);
        }
    }

    /* Sektor kasujemy raz, na jego początku. */
    if (spiRxAddr % SPI_FLASH_SECTOR == 0) {
        if (!spiFlashEraseSector(spiRxAddr)) {
            spiAbortWrite("SPI kasowanie sektora nieudane");
            return;
        }
    }

    uint32_t off = 0;
    while (off < n) {
        uint32_t c = n - off;
        uint32_t within = SPI_FLASH_PAGE - ((spiRxAddr + off) % SPI_FLASH_PAGE);
        if (c > within) c = within;
        if (!spiFlashWritePage(spiRxAddr + off, buf + off, c)) {
            spiAbortWrite("SPI zapis strony nieudany");
            return;
        }
        off += c;
    }

    uint8_t chk[256];
    if (!spiFlashRead(spiRxAddr, chk, n) || memcmp(chk, buf, n) != 0) {
        spiAbortWrite("SPI weryfikacja nieudana");
        return;
    }

    spiRxAddr += n;
    spiRxRemaining -= n;
    g_out->print("+OK ");
    g_out->print(spiRxAddr, HEX);
    g_out->print("\r\n");

    if (spiRxRemaining == 0) spiFinishWrite();
}

/* --- komendy ----------------------------------------------------------------- */
static void cmdHelp() {
    g_out->print(
        "Komendy:\r\n"
        "  HELP                  - ta lista\r\n"
        "  PINS                  - aktualne piny\r\n"
        "  SWSTEST [div]         - test lacza SWS (zapis+odczyt [0x00b2])\r\n"
        "  CHIP                  - aktywacja + test SWS + JEDEC flash\r\n"
        "  CHIPID / ID           - detekcja ukladu TLSR przez SWS (rejestr 0x7d)\r\n"
        "  AREG <adr>            - odczyt rejestru analogowego (0x00..0xFF)\r\n"
        "  AWR <adr> <wartosc>   - zapis rejestru analogowego + odczyt kontrolny\r\n"
        "  ADUMP                 - zrzut wszystkich rejestrow analogowych\r\n"
        "  JEDEC                 - ID kosci flash\r\n"
        "  PROBE                 - sonda sprzetowa (stan linii + nasluch UART)\r\n"
        "  VMEAS <pin>           - napiecie na pinie ADC (1..10) w mV\r\n"
        "  VSCAN                 - napiecia na wszystkich pinach ADC 1..10\r\n"
        "  PWM <pin> <Hz> <%>    - PWM na wolnym pinie (1,2,7,36..40,47,48)\r\n"
        "  PWMSTOP <pin>         - wylacz PWM i zwolnij pin\r\n"
        "  READ <adr> <len>      - zrzut flash (binarnie po +DATA)\r\n"
        "  ERASE <start> <len>   - kasowanie sektorow 4KB w zakresie\r\n"
        "  WRITE <rozmiar> [adr] - odbierz i wgraj obraz .bin (z weryfikacja)\r\n"
        "  RESET                 - reset celu\r\n"
        "  LISTEN [ms] [baud] [NONE|RST|GO]\r\n"
        "                        - zrzut UART celu: tekst + hex (dom. 2000 ms, 115200)\r\n"
        "                          GO = aktywacja + [0x0602]=0x88 (CPU ReBoot)\r\n"
        "  READW <adr> <len>     - odczyt flash bajt po bajcie (pewny, ~0,5 ms/B)\r\n"
        "  UNIT <us>             - czas jednostki SWS w us (domyslnie 4, zakres 1-4)\r\n"
        "  PHASE <cwiartki>      - punkt probki bitu odczytu (10 = 2,5 jedn., 1-60)\r\n"
        "  WIDTHS [n]            - zmierzone szerokosci LOW bitow [0x00b2] (diagnostyka)\r\n"
        "  WAVE [n]              - surowy przebieg linii po odczycie (diagnostyka)\r\n"
        "--- diagnostyka flasha ---\r\n"
        "  STAT                  - SR1/SR2/SR3 + MSPI + CPU (bez zmian w kosci)\r\n"
        "  WEL                   - WREN + kontrola WEL (czy WREN dochodzi)\r\n"
        "  UNLOCK                - zdjecie blokad BP3..BP0 z kosci flash\r\n"
        "  WRSR <hex>            - zapis rejestru statusu (zdjecie blokad BP/SRP)\r\n"
        "  FCMD <hex...>         - surowa transakcja MSPI do kosci\r\n"
        "  FCMDR <n> <hex...>    - jak FCMD + odczyt n bajtow\r\n"
        "  ERASE1 <adr>          - kasowanie 1 sektora + pelna diagnostyka\r\n"
        "  WRPG <adr> <hex...>   - zapis 1..256 B + odczyt kontrolny\r\n"
        "  MSPIR <adr> [n]       - odczyt rejestru/SRAM ukladu (np. 40000)\r\n"
        "  MSPIW <adr> <hex...>  - zapis rejestru/SRAM ukladu + odczyt kontrolny\r\n"
        "  SRAMTEST [n]          - test bledow lacza SWS na SRAM (bez flasha)\r\n"
        "  MEMTEST [KB]          - test PSRAM + SRAM ESP32 (wzorce, walking)\r\n"
        "  CAP <pin> [n] [div]   - logic analyzer: n probek do PSRAM + RLE\r\n"
        "  OSC <pin> [n] [us]    - mini-oscyloskop ADC: n probek + statystyki\r\n"
        "  MSPIWATCH [ms]        - czy rejestry MSPI zmieniaja sie same\r\n"
        "  HALT [ms]             - aktywacja + weryfikacja [0x0602]=0x05\r\n"
        "  BPOLL <0|1>           - czekanie na BUSY przed zapisem bajtu (jak SDK)\r\n"
        "  RDHEX <adr> <len>     - podglad flasha w hex (max 256 B)\r\n"
        "--- SPI flash 25xx (klips SOIC-8) ---\r\n"
        "  SPI                   - wykryj kosc (JEDEC + rozmiar)\r\n"
        "  SPIREAD <adr> <len>   - zrzut kosci SPI (binarnie po +DATA)\r\n"
        "  SPIHEX <adr> <len>    - podglad kosci SPI w hex (max 256 B)\r\n"
        "  SPIBLANK <adr> <len>  - sprawdz czy obszar czysty (same 0xFF)\r\n"
        "  SPIERASE <start> <len>- kasowanie sektorow 4KB w zakresie\r\n"
        "  SPIERASECHIP          - kasowanie CALEJ kosci SPI (uwaga!)\r\n"
        "  SPIWRITE <adr> <rozmiar>\r\n"
        "                        - odbierz i wgraj obraz do SPI (z weryfikacja)\r\n"
        "--- karta microSD (Catalex, osobny SPI) ---\r\n"
        "  SD                    - wykryj karte: typ, pojemnosc, wolne\r\n"
        "  SDTEST                - diagnostyka SD: stan pinow + swiezy init\r\n"
        "  SDLS [sciezka]        - lista plikow (domyslnie /)\r\n"
        "  SDCAT <plik>          - wypisz zawartosc pliku (tekst)\r\n"
        "  SDAPPEND <plik> <tekst...>\r\n"
        "                        - dopisz linie tekstu do pliku (logowanie)\r\n"
        "  SDDEL <plik>          - usun plik\r\n"
        "--- WiFi ---\r\n"
        "  WIFISCAN              - skan sieci WiFi (wyniki wypisane tutaj)\r\n"
        "  WIFISTAT              - status AP/STA + adresy IP\r\n"
        "  WIFIJOIN <ssid> <haslo>\r\n"
        "                        - zapisz WiFi i polacz jako STA (bez utraty AP)\r\n"
        "  WIFICLEAR             - usun zapisane WiFi (tylko AP)\r\n"
        "+OK HELP\r\n");
}

static void cmdPins() {
    g_out->print("SWS=");
    g_out->print(SWS_PIN);
    g_out->print(" RST=");
    g_out->print(RST_PIN);
    g_out->print(" UART_TX=");
    g_out->print(UART_TX_PIN);
    g_out->print(" UART_RX=");
    g_out->print(UART_RX_PIN);
    g_out->print("\r\n");
    g_out->print("SPI: SCK=");
    g_out->print(SPI_SCK_PIN);
    g_out->print(" MISO=");
    g_out->print(SPI_MISO_PIN);
    g_out->print(" MOSI=");
    g_out->print(SPI_MOSI_PIN);
    g_out->print(" CS=");
    g_out->print(SPI_CS_PIN);
    g_out->print("\r\n");
    g_out->print("RS485: TX=");
    g_out->print(RS485_TX_PIN);
    g_out->print(" RX=");
    g_out->print(RS485_RX_PIN);
    g_out->print(" DE=");
    g_out->print(RS485_DE_PIN);
    g_out->print("\r\n");
    g_out->print("SD: SCK=");
    g_out->print(SD_SCK_PIN);
    g_out->print(" MISO=");
    g_out->print(SD_MISO_PIN);
    g_out->print(" MOSI=");
    g_out->print(SD_MOSI_PIN);
    g_out->print(" CS=");
    g_out->print(SD_CS_PIN);
    g_out->print("\r\n");
    g_out->print("1-Wire: ");
    g_out->print(ONE_WIRE_PIN);
    g_out->print("\r\n");
    g_out->print("I2C: SDA=");
    g_out->print(I2C_SDA_PIN);
    g_out->print(" SCL=");
    g_out->print(I2C_SCL_PIN);
    g_out->print("\r\n");
    g_out->print("ESP bridge: IO0=");
    g_out->print(ESP_IO0_PIN);
    g_out->print(" EN=");
    g_out->print(ESP_EN_PIN);
    g_out->print(" TCP=");
    g_out->print(ESP_BRIDGE_PORT);
    g_out->print(" UART=");
    g_out->print(UART_TX_PIN);
    g_out->print("/");
    g_out->print(UART_RX_PIN);
    g_out->print("\r\n");
    g_out->print("+OK PINS\r\n");
}

static void cmdWifiScan() {
    int n = wifiScanNow();
    g_out->print("+OK WIFISCAN ");
    g_out->print(n);
    g_out->print(" sieci");
    g_out->print(wifiScanPathGet() ? " (fallback STA)\r\n" : " (AP podniesione)\r\n");
    for (int i = 0; i < n; i++) {
        String ssid;
        int rssi;
        bool openNet;
        if (wifiScanGet(i, ssid, rssi, openNet)) {
            g_out->print("  [");
            g_out->print(i);
            g_out->print("] ");
            g_out->print(ssid);
            g_out->print("  RSSI=");
            g_out->print(rssi);
            g_out->print(openNet ? "  OPEN\r\n" : "  (zabezpieczona)\r\n");
        }
    }
}

static void cmdWifiStat() {
    g_out->print("AP: ");
    g_out->print(WEB_AP_SSID);
    g_out->print(" @ ");
    g_out->print(WiFi.softAPIP().toString());
    g_out->print("\r\nSTA: ");
    String ssid = wifiStaGetSsid();
    if (ssid.length()) {
        g_out->print(ssid);
        if (wifiStaConnected()) {
            g_out->print("  CONNECTED  IP=");
            g_out->print(wifiStaGetIp());
        } else {
            g_out->print("  laczenie...");
        }
    } else {
        g_out->print("(brak)");
    }
    g_out->print("\r\n+OK WIFISTAT\r\n");
}

static void cmdWifiJoin(char *p) {
    char *ssid = nextTok(p);
    char *pass = nextTok(p);
    if (!ssid || !*ssid) {
        g_out->print("-ERR uzycie: WIFIJOIN <ssid> <haslo>\r\n");
        return;
    }
    wifiJoin(String(ssid), pass ? String(pass) : String(""));
    g_out->print("+OK lacze z \"");
    g_out->print(ssid);
    g_out->print("\"... (sprawdz: WIFISTAT)\r\n");
}

static void cmdWifiForget() {
    wifiForget();
    g_out->print("+OK usunieto zapisane WiFi\r\n");
}

/* --- pomiar napięcia ---------------------------------------------------------
 * Tylko piny ADC1, czyli GPIO1..GPIO10. Wejście ADC jest wysokoomowe, więc można
 * nim bezpiecznie dotykać dowolnego pinu celu — nic nie jest wymuszane.
 * Służy do ustalenia, który pin złącza P2 to Vdd, a który GND, bez multimetru.
 */
static bool adcReadMv(int pin, uint32_t &mv, uint32_t &raw) {
    if (pin < 1 || pin > 10) return false;
    pinMode(pin, INPUT);
    delay(2);
    uint32_t sumMv = 0, sumRaw = 0;
    const int N = 32;
    for (int i = 0; i < N; i++) {
        sumMv += analogReadMilliVolts(pin);
        sumRaw += (uint32_t)analogRead(pin);
        delay(2);
    }
    mv = sumMv / N;
    raw = sumRaw / N;
    return true;
}

static void cmdVmeas(int pin) {
    uint32_t mv = 0, raw = 0;
    if (!adcReadMv(pin, mv, raw)) {
        g_out->print("-ERR VMEAS: tylko piny ADC1, czyli GPIO1..GPIO10\r\n");
        return;
    }
    outPrintf("+OK VMEAS GPIO%d = %u mV (raw %u)\r\n", pin, (unsigned)mv, (unsigned)raw);
}

static void cmdVscan() {
    g_out->print("Napiecia na pinach ADC1 (GPIO1..GPIO10):\r\n");
    for (int p = 1; p <= 10; p++) {
        uint32_t mv = 0, raw = 0;
        if (!adcReadMv(p, mv, raw)) continue;
        outPrintf("  GPIO%-2d = %4u mV (raw %4u)\r\n", p, (unsigned)mv, (unsigned)raw);
    }
    g_out->print("GND = ~0 mV, Vdd = ~3000 mV, linia sygnalowa = ~3000 mV.\r\n");
    g_out->print("+OK VSCAN\r\n");
}

/* --- PWM: prosty generator na wolnym pinie ---------------------------------- */
static bool pwmPinAllowed(uint32_t pin) {
    static const uint32_t freePins[] = {1, 2, 7, 36, 37, 38, 39, 40, 47, 48};
    for (uint32_t i = 0; i < sizeof(freePins) / sizeof(freePins[0]); i++) {
        if (freePins[i] == pin) return true;
    }
    return false;
}

static void cmdPwm(uint32_t pin, uint32_t freq, uint32_t duty) {
    if (!pwmPinAllowed(pin)) {
        g_out->print("-ERR PWM: dozwolone piny 1,2,7,36,37,38,39,40,47,48\r\n");
        return;
    }
    if (freq < 1) freq = 1000;
    if (freq > 1000000) freq = 1000000;
    if (duty > 100) duty = 100;

    ledcSetup(0, freq, 8);          /* kanal 0, 8 bitow */
    ledcAttachPin((int)pin, 0);
    ledcWrite(0, (uint32_t)(duty * 255u / 100u));

    outPrintf("+OK PWM GPIO%u = %u Hz, wypelnienie %u%%\r\n",
              (unsigned)pin, (unsigned)freq, (unsigned)duty);
}

static void cmdPwmStop(uint32_t pin) {
    if (!pwmPinAllowed(pin)) {
        g_out->print("-ERR PWM: dozwolone piny 1,2,7,36,37,38,39,40,47,48\r\n");
        return;
    }
    ledcDetachPin((int)pin);
    pinMode((int)pin, INPUT);
    outPrintf("+OK PWM STOP GPIO%u\r\n", (unsigned)pin);
}

/* --- diagnostyka sprzętowa ---------------------------------------------------
 * Sprawdza stan elektryczny każdej linii w trzech konfiguracjach wejścia.
 * Nie wymusza na linii żadnego stanu, więc jest bezpieczna nawet przy złym
 * podłączeniu (np. gdy pin sygnałowy trafił na 3V3).
 *
 *   pullup=1 pulldown=0 -> linia "lata": nic nie podłączone
 *   pullup=1 pulldown=1 -> coś trzyma linię wysoko: podłączona (pull-up/3V3)
 *   pullup=0 pulldown=0 -> linia zwarta do masy
 */
static void probeLine(const char *label, int pin) {
    pinMode(pin, INPUT);
    delay(2);
    const int idle = digitalRead(pin);

    pinMode(pin, INPUT_PULLUP);
    delay(5);
    const int up = digitalRead(pin);

    pinMode(pin, INPUT_PULLDOWN);
    delay(5);
    const int dn = digitalRead(pin);

    pinMode(pin, INPUT);

    const char *verdict;
    if (up == 1 && dn == 0)      verdict = "nic nie podlaczone?";
    else if (up == 1 && dn == 1) verdict = "polaczona (stan wysoki)";
    else                         verdict = "zwarta do GND";

    outPrintf("  %-4s GPIO%-2d  idle=%d  pullup=%d  pulldown=%d  -> %s\r\n",
                  label, pin, idle, up, dn, verdict);
}

/* Nasłuch na jednym pinie: czy cokolwiek nadaje. Jeśli dane pojawią się na
 * pinie TX zamiast RX, przewody TX/RX są zamienione. */
static uint32_t probeListen(int pin, uint32_t baud, uint32_t ms) {
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, pin, -1);
    uint32_t n = 0;
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        while (Serial1.available()) {
            Serial1.read();
            n++;
        }
        vTaskDelay(1);
    }
    Serial1.end();
    return n;
}

static void cmdProbe() {
    g_out->print("Sonda sprzetowa (nic nie jest wymuszane):\r\n");
    probeLine("SWS", SWS_PIN);
    probeLine("RST", RST_PIN);
    probeLine("RX", UART_RX_PIN);
    probeLine("TX", UART_TX_PIN);

    g_out->print("Nasluch UART (1 s na pin):\r\n");
    for (uint32_t baud = 115200; baud <= 230400; baud *= 2) {
        const uint32_t onRx = probeListen(UART_RX_PIN, baud, 1000);
        const uint32_t onTx = probeListen(UART_TX_PIN, baud, 1000);
        outPrintf("  @%u: RX(pin %d)=%u bajtow, TX(pin %d)=%u bajtow\r\n",
                      baud, UART_RX_PIN, onRx, UART_TX_PIN, onTx);
    }

    g_out->print("Wskazowki: SWS i RST powinny byc 'polaczona (stan wysoki)'.\r\n");
    g_out->print("Gdy SWS/RST pokazuje 'nic nie podlaczone' - brak kontaktu.\r\n");
    g_out->print("Gdy 'zwarta do GND' - pomylone piny (np. GND zamiast sygnalu).\r\n");
    g_out->print("Dane na pinie TX zamiast RX = zamienione przewody TX/RX.\r\n");
    g_out->print("Brak zasilania celu tez daje 'nic nie podlaczone'.\r\n");
    g_out->print("+OK PROBE\r\n");
}

/* Aktywacja ukladu docelowego i test pelnego obiegu SWS: zapis dzielnika
 * [0x00b2] i odczyt tego samego rejestru. To jednoznaczny dowod, ze SWS dziala
 * w obie strony. Pierwsza proba tuz po zimnym starcie ESP32 bywa nieudana,
 * dlatego powtarzamy ja raz - przypadkowy "-ERR" nie jest wtedy mylacy. */
bool swsLinkTest(uint8_t div, bool verbose) {
    bool gotReply = false;
    uint8_t lastRb = 0;

    for (int attempt = 0; attempt < 2; attempt++) {
        if (!swsActivate(300)) continue;
        if (!swsWriteReg8(0x00B2, div)) continue;
        delay(2);

        uint8_t rb = 0;
        if (!swsReadReg(0x00B2, &rb, 1)) continue;
        if (rb == div) return true;

        gotReply = true;
        lastRb  = rb;
    }

    if (verbose) {
        if (gotReply) {
            g_out->print("-ERR SWS: [0x00b2] zapisane ");
            g_out->print(div);
            g_out->print(", odczytane ");
            g_out->print(lastRb);
            g_out->print("\r\n");
        } else {
            g_out->print("-ERR SWS: brak odpowiedzi przy odczycie [0x00b2]\r\n");
        }
    }
    return false;
}

/* Dzielnik docelowy wynikajacy z aktualnego UNIT - trzyma oba konce w tym
 * samym tempie (UNIT 6/8 bez tego nigdy nie mialy prawa zadzialac). */
static inline uint8_t swsDivFromUnit() { return swsUnitToDiv(swsGetUnitUs()); }

/* Diagnostyka: odczyt [0x00b2] (znana wartosc - dzielnik, typowo 127 =
 * 0b01111111) i wypisanie zmierzonej szerokosci stanu LOW kazdego bitu.
 * Pozwala dobrac punkt probki z danych zamiast zgadywac. */
static void cmdWidths(uint32_t reps) {
    if (!reps) reps = 8;
    if (reps > 100) reps = 100;

    if (!swsLinkTest(swsDivFromUnit())) return;

    const uint8_t expected = swsDivFromUnit();
    g_out->print("PROBKA ");
    g_out->print(swsGetSampleQuarter());
    g_out->print("/4 jednostki, oczekiwana wartosc b2=");
    g_out->print(expected);
    g_out->print("\r\n");

    uint32_t bad = 0;
    for (uint32_t r = 0; r < reps; r++) {
        uint8_t b = 0;
        if (!swsReadReg(0x00B2, &b, 1)) {
            g_out->print("-ERR odczyt [0x00b2] nieudany\r\n");
            return;
        }
        const uint32_t *w    = swsLastBitWidths();
        const uint32_t used  = swsLastSampleQuarter();
        g_out->print("b2=");
        g_out->print(b);
        if (b != expected) {
            bad++;
            g_out->print(" (zle)");
        }
        g_out->print(" LOW[7..0]=");
        for (int i = 0; i < 8; i++) {
            g_out->print(used + w[i]);
            if (i != 7) g_out->print(" ");
        }
        g_out->print(" cwiartek\r\n");
    }
    g_out->print("+OK WIDTHS ");
    g_out->print(reps - bad);
    g_out->print("/");
    g_out->print(reps);
    g_out->print(" poprawnych\r\n");
}

/* Surowy przebieg linii SWS po komendzie odczytu [0x00b2]. Wypisuje przedzialy
 * stanu LOW w cwiartkach jednostki - to pozwala zmierzyc realne szerokosci
 * impulsow i okres bitow zamiast zakladac kodowanie. */
static void cmdWave(uint32_t reps) {
    static uint8_t levels[264];
    if (!reps) reps = 1;
    if (reps > 20) reps = 20;

    if (!swsLinkTest(swsDivFromUnit())) return;

    for (uint32_t r = 0; r < reps; r++) {
        if (!swsReadWave(0x00B2, levels, sizeof(levels))) {
            g_out->print("-ERR WAVE nieudany\r\n");
            return;
        }

        g_out->print("LOW:");
        int runStart = -1;
        for (uint32_t k = 0; k <= sizeof(levels); k++) {
            const bool low = (k < sizeof(levels)) && (levels[k] == 0);
            if (low && runStart < 0) {
                runStart = (int)k;
            } else if (!low && runStart >= 0) {
                g_out->print(" ");
                g_out->print((uint32_t)runStart);
                g_out->print("-");
                g_out->print(k - 1);
                runStart = -1;
            }
        }
        g_out->print("  | koniec=");
        g_out->print(levels[sizeof(levels) - 1] ? "H" : "L");
        g_out->print(" (cwiartki jednostki, ");
        g_out->print(swsGetUnitUs() * sizeof(levels) / 4.0, 1);
        g_out->print(" us)\r\n");
    }
    g_out->print("+OK WAVE\r\n");
}

static void cmdSwstest(uint32_t divArg) {
    const uint8_t div = divArg ? (uint8_t)divArg : swsDivFromUnit();
    g_out->print("UNIT ");
    g_out->print(swsGetUnitUs());
    g_out->print(" us, docelowy [0x00b2]=");
    g_out->print(div);
    g_out->print("\r\n");

    if (!swsLinkTest(div)) {
        g_out->print("Sprawdz Vdd / GND / SWS / RST (pinout w README.md).\r\n");
        return;
    }
    g_out->print("+OK SWS dziala w obie strony\r\n");
}

/* Odczyt JEDEC z aktywacja i ponowieniami. Sam odczyt bez aktywacji zwracal
 * 0x0, bo uklad docelowy nie byl w trybie ISP; do tego pojedynczy odczyt
 * potrafi sie nie udac, wiec odrzucamy identyfikatory niepodobne do prawdziwych. */
static bool jedecRobust(uint32_t *out, int tries = 5) {
    for (int i = 0; i < tries; i++) {
        if (swsLinkTest(swsDivFromUnit(), false)) {
            uint32_t id = 0;
            if (swsFlashJedecId(&id) && id != 0x000000 && id != 0xFFFFFF) {
                *out = id;
                return true;
            }
        }
        delay(20);
    }
    return false;
}

static void cmdChip() {
    if (!swsLinkTest(swsDivFromUnit())) return;

    uint32_t jedec = 0;
    if (!swsFlashJedecId(&jedec)) {
        g_out->print("-ERR SWS dziala, ale odczyt JEDEC flash nieudany\r\n");
        return;
    }
    g_out->print("+OK JEDEC 0x");
    g_out->print(jedec, HEX);
    g_out->print("\r\n");
}

/* Odczyt identyfikatora układu TLSR przez SWS: rejestr cyfrowy 0x7d (3 bajty),
 * wg pvvx/TLSRPGM ReadChipID(): buf[0]=rewizja, buf[1..2]=ID (little-endian). */
static bool readTlsrChipId(uint16_t *cid, uint8_t *rev) {
    uint8_t b[3] = {0, 0, 0};
    if (!swsReadReg(0x7D, b, 3)) return false;
    if (rev) *rev = b[0];
    if (cid) *cid = (uint16_t)(b[1] | (b[2] << 8));
    return true;
}

static const char *tlsrChipName(uint16_t cid) {
    switch (cid) {
        case 0x5562: return "TLSR825x (8251/8253/8258)";
        case 0x5591: return "TLSR8208";
        case 0x5325: return "TLSR8266";
        case 0x5326: return "TLSR8267";
        case 0x5327: return "TLSR8269";
        default:     return "nieznany";
    }
}

static void cmdChipId() {
    if (!swsLinkTest(swsDivFromUnit())) return;

    uint16_t cid = 0;
    uint8_t rev = 0;
    if (!readTlsrChipId(&cid, &rev)) {
        g_out->print("-ERR odczyt ID układu przez SWS nieudany (rejestr 0x7d)\r\n");
        return;
    }
    g_out->print("+OK ChipID 0x");
    if (cid < 0x1000) g_out->print("0");
    if (cid < 0x100) g_out->print("0");
    if (cid < 0x10) g_out->print("0");
    g_out->print(cid, HEX);
    g_out->print(" (");
    g_out->print(tlsrChipName(cid));
    g_out->print("), rewizja 0x");
    if (rev < 0x10) g_out->print("0");
    g_out->print(rev, HEX);
    g_out->print("\r\n");
}

static void cmdAreg(uint32_t addr) {
    if (addr > 0xFF) {
        g_out->print("-ERR adres analogowy 0x00..0xFF\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;
    uint8_t v = 0;
    if (!swsAnalogRead(addr, &v)) {
        g_out->print("-ERR odczyt rejestru analogowego nieudany\r\n");
        return;
    }
    g_out->print("+OK A[0x");
    if (addr < 0x10) g_out->print("0");
    g_out->print(addr, HEX);
    g_out->print("] = 0x");
    if (v < 0x10) g_out->print("0");
    g_out->print(v, HEX);
    g_out->print("\r\n");
}

static void cmdAwrite(uint32_t addr, uint8_t val) {
    if (addr > 0xFF) {
        g_out->print("-ERR adres analogowy 0x00..0xFF\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;
    if (!swsAnalogWrite(addr, val)) {
        g_out->print("-ERR zapis rejestru analogowego nieudany\r\n");
        return;
    }
    uint8_t rb = 0;
    swsAnalogRead(addr, &rb);
    g_out->print("+OK A[0x");
    if (addr < 0x10) g_out->print("0");
    g_out->print(addr, HEX);
    g_out->print("] = 0x");
    if (rb < 0x10) g_out->print("0");
    g_out->print(rb, HEX);
    g_out->print(" (odczyt kontrolny)\r\n");
}

static void cmdAdump() {
    if (!swsLinkTest(swsDivFromUnit())) return;
    g_out->print("+ADUMP\r\n");
    for (uint32_t a = 0; a <= 0xFF; a++) {
        uint8_t v = 0;
        if (!swsAnalogRead(a, &v)) {
            g_out->print("-ERR przerwane przy A[0x");
            g_out->print(a, HEX);
            g_out->print("]\r\n");
            return;
        }
        if ((a & 0x0F) == 0) {
            if (a) g_out->print("\r\n");
            g_out->print("0x");
            if (a < 0x10) g_out->print("0");
            g_out->print(a, HEX);
            g_out->print(": ");
        }
        g_out->print(" ");
        if (v < 0x10) g_out->print("0");
        g_out->print(v, HEX);
    }
    g_out->print("\r\n");
}

static void cmdRead(uint32_t start, uint32_t len) {
    if (len == 0) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;

    g_out->print("+DATA ");
    g_out->print(len);
    g_out->print("\r\n");

    static uint8_t buf[SWS_SWS_BURST];
    uint32_t a = start;
    uint32_t rem = len;
    while (rem) {
        uint32_t c = rem < SWS_SWS_BURST ? rem : (uint32_t)SWS_SWS_BURST;
        if (!swsFlashRead(a, buf, c)) break;  // błąd w trakcie - host wykryje krótki odczyt
        g_out->write(buf, c);
        a += c;
        rem -= c;
    }
}

static void cmdErase(uint32_t start, uint32_t len) {
    if (len == 0) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;

    uint32_t s = start & ~(uint32_t)(SWS_FLASH_SECTOR - 1);
    uint32_t e = (start + len + SWS_FLASH_SECTOR - 1) & ~(uint32_t)(SWS_FLASH_SECTOR - 1);
    for (uint32_t a = s; a < e; a += SWS_FLASH_SECTOR) {
        if (!swsFlashEraseSector(a)) {
            g_out->print("-ERR kasowanie 0x");
            g_out->print(a, HEX);
            g_out->print("\r\n");
            return;
        }
    }
    g_out->print("+OK\r\n");
}

static void cmdWrite(uint32_t size, uint32_t offset) {
    if (size == 0 || size > SWS_FLASH_SIZE) {
        g_out->print("-ERR rozmiar poza zakresem (max 0x");
        g_out->print(SWS_FLASH_SIZE, HEX);
        g_out->print(")\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;

    rxActive = true;
    rxRemaining = size;
    rxAddr = offset;
    g_out->print("+READY\r\n");
}

/* =============================================================================
 * Diagnostyka flasha.
 *
 * Kontekst: `erase` i `write` do flasha celu nic nie robia (po odczycie kontrolnym
 * kosc wyglada identycznie jak kopia). Odczyt, JEDEC i zapisy rejestrow SWS dzialaja,
 * wiec lacze jest sprawne - podejrzane jest albo zabezpieczenie kosci (BP/SRP), albo
 * to, ze zbocze CS w gore nie wykonuje komendy (np. zapisy [0x0d] sa odrzucane,
 * gdy wisi bit BUSY).
 *
 * Komendy ponizej pozwalaja to rozstrzygnac krok po kroku, bez ryzyka:
 *   STAT/WEL   - tylko odczyt + WREN (WREN nic nie zmienia w zawartosci)
 *   FCMD       - surowa komenda; podglad [0x0d] po obu zboczach CS
 *   SRAMTEST   - pomiar bledow lacza SWS bez dotykania flasha
 *   ERASE1     - pelna diagnostyka kasowania (SR1/WEL/WIP + zawartosc sektora)
 *   WRPG       - zapis strony z natychmiastowym odczytem kontrolnym
 * =============================================================================
 */

static uint8_t hexNib(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return (uint8_t)(c - 'A' + 10);
}

static bool isHexDigit(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

/* Zbiera wszystkie cyfry hex z tekstu (spacje i przecinki pomija, obsluguje 0x).
 * Nieparzysta liczba cyfr = doklejone zero z przodu (np. "5" -> 0x05).
 * Zwraca liczbe zebranych bajtow. */
static uint32_t parseHexAll(const char *s, uint8_t *out, uint32_t maxOut) {
    static char digits[2 * SWS_FLASH_PAGE + 2];
    const uint32_t cap = sizeof(digits) - 1;
    uint32_t nd = 0;

    for (const char *q = s; *q; q++) {
        if (*q == 'x' || *q == 'X') {
            if (nd && digits[nd - 1] == '0') nd--;   /* przedrostek 0x */
            continue;
        }
        if (!isHexDigit(*q)) continue;               /* spacje, przecinki, 'h' itp. */
        if (nd >= cap) break;
        digits[nd++] = *q;
    }

    if (nd & 1) {
        for (uint32_t i = nd; i > 0; i--) digits[i] = digits[i - 1];
        digits[0] = '0';
        nd++;
    }

    uint32_t nb = 0;
    for (uint32_t i = 0; i + 1 < nd && nb < maxOut; i += 2) {
        out[nb++] = (uint8_t)((hexNib(digits[i]) << 4) | hexNib(digits[i + 1]));
    }
    return nb;
}

static void printHex8(uint8_t v) {
    g_out->print((v >> 4) & 0x0F, HEX);
    g_out->print(v & 0x0F, HEX);
}

static void printHexBytes(const uint8_t *b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        printHex8(b[i]);
        g_out->print(' ');
    }
}

static void printReg(const char *name, uint32_t addr) {
    uint8_t v = 0;
    g_out->print("  ");
    g_out->print(name);
    g_out->print(" [0x");
    g_out->print(addr, HEX);
    g_out->print("] = ");
    if (swsReadReg(addr, &v, 1)) {
        g_out->print("0x");
        printHex8(v);
    } else {
        g_out->print("BLAD");
    }
    g_out->print("\r\n");
}

/* SR1 (0x05): bit0=WIP, bit1=WEL, bity2..4=BP0..BP2, bit7=SRP0 (rejestr blokad). */
static void printSr1Bits(uint8_t s) {
    g_out->print("     WIP=");
    g_out->print(s & 0x01);
    g_out->print(" WEL=");
    g_out->print((s >> 1) & 1);
    g_out->print(" BP0=");
    g_out->print((s >> 2) & 1);
    g_out->print(" BP1=");
    g_out->print((s >> 3) & 1);
    g_out->print(" BP2=");
    g_out->print((s >> 4) & 1);
    g_out->print(" BP3=");
    g_out->print((s >> 5) & 1);
    g_out->print(" SRP0=");
    g_out->print((s >> 7) & 1);
    if ((s & 0x7C) || (s & 0x80)) g_out->print("   <- BLOKADY USTAWIONE!");
    g_out->print("\r\n");
}

/* Czeka na WIP=0. sawBusy=true znaczy, ze kosc raportowala prace (operacja
 * naprawde sie wykonywala). Kasowanie sektora 4K trwa ~45 ms, jedna ramka
 * odczytu statusu ~5 ms, wiec wynik jest rozstrzygajacy. */
static bool waitWipClear(uint32_t timeoutMs, uint32_t *outMs, bool *sawBusy) {
    const uint32_t t0 = millis();
    bool busy = false;
    for (;;) {
        uint8_t s = 0;
        if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &s, 1, false)) return false;
        if (!(s & 0x01)) break;
        busy = true;
        if ((uint32_t)(millis() - t0) > timeoutMs) break;
    }
    if (outMs) *outMs = millis() - t0;
    if (sawBusy) *sawBusy = busy;
    return true;
}

/* --- STAT: pelny obraz stanu bez zmiany zawartosci kosci --------------------- */
static void cmdStat() {
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    uint8_t sr1 = 0, sr1sdk = 0, sr2 = 0, sr3 = 0;
    const bool ok1 = swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr1, 1, false);
    const bool ok1s = swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr1sdk, 1, true);
    const bool ok2 = swsFlashStatusCmd(SWS_FLASH_STATUS2_CMD, &sr2, 1, false);
    const bool ok3 = swsFlashStatusCmd(SWS_FLASH_STATUS3_CMD, &sr3, 1, false);

    g_out->print("Rejestry statusu kosci:\r\n");
    g_out->print("  SR1 (0x05, auto-read) = ");
    if (ok1) {
        g_out->print("0x");
        printHex8(sr1);
        printSr1Bits(sr1);
    } else {
        g_out->print("BLAD\r\n");
    }
    g_out->print("  SR1 (0x05, jak SDK)   = ");
    if (ok1s) {
        g_out->print("0x");
        printHex8(sr1sdk);
        g_out->print("\r\n");
    } else {
        g_out->print("BLAD\r\n");
    }
    g_out->print("  SR2 (0x35)            = ");
    g_out->print(ok2 ? "0x" : "BLAD\r\n");
    if (ok2) {
        printHex8(sr2);
        g_out->print("\r\n");
    }
    g_out->print("  SR3 (0x15)            = ");
    g_out->print(ok3 ? "0x" : "BLAD\r\n");
    if (ok3) {
        printHex8(sr3);
        g_out->print("\r\n");
    }
    if (ok1 && ok1s && sr1 != sr1sdk)
        g_out->print("  UWAGA: dwie metody odczytu SR1 daja rozne wyniki!\r\n");

    g_out->print("Rejestry ukladu (TLSR):\r\n");
    printReg("MSPI data", 0x000C);
    g_out->print("  MSPI ctrl [0xD] = ");
    uint8_t ctrl = 0;
    if (swsMspiCtrlRead(&ctrl)) {
        g_out->print("0x");
        printHex8(ctrl);
    } else {
        g_out->print("BLAD");
    }
    g_out->print("\r\n");
    printReg("SWS  div ", 0x00B2);
    printReg("SWS  mode", 0x00B3);
    uint8_t cpu[2] = {0, 0};
    if (swsReadReg(0x0602, cpu, 2)) {
        g_out->print("  CPU  stop [0x0602] = 0x");
        printHex8(cpu[1]);
        printHex8(cpu[0]);
        g_out->print(cpu[0] == 0x05 ? "  (CPU zatrzymany)\r\n" : "  (oczekiwane 0x05!)\r\n");
    } else {
        g_out->print("  CPU  stop [0x0602] = BLAD\r\n");
    }

    /* Czy uklad przyjmuje zapisy [0x0d]? Readback != zapis = zapis zignorowany,
     * a wtedy CS nigdy nie faluje i zadna komenda nie dochodzi do kosci. */
    g_out->print("Test przyjecia zapisu [0x0d]:\r\n");
    const uint8_t want[3] = {SWS_MSPI_CS, 0x00, (uint8_t)SWS_MSPI_AUTO_READ};
    const char *desc[3] = {"CS w gore (0x01)", "CS w dol  (0x00)", "auto-read (0x0a)"};
    for (int i = 0; i < 3; i++) {
        uint8_t rb = 0;
        const bool ok = swsMspiCtrlWrite(want[i], &rb);
        g_out->print("  zapis ");
        g_out->print(desc[i]);
        g_out->print(" -> odczyt 0x");
        if (!ok) {
            g_out->print("BLAD\r\n");
            continue;
        }
        printHex8(rb);
        g_out->print((rb & want[i]) == want[i] ? "  OK" : "  ZAPIS ODRZUCONY!");
        if (rb & SWS_MSPI_BUSY) g_out->print(" (BUSY=1!)");
        g_out->print("\r\n");
    }
    swsMspiCtrlWrite(SWS_MSPI_CS, nullptr);   /* zostaw linie CS w spoczynku */
    g_out->print("BPOLL = ");
    g_out->print(swsGetBusyPoll() ? "1" : "0");
    g_out->print(", timeouty BUSY = ");
    g_out->print(swsBusyTimeoutCount());
    g_out->print("\r\n");
    g_out->print("+OK STAT\r\n");
}

/* --- WEL: czy WREN w ogole dochodzi do kosci --------------------------------- */
static void cmdWel() {
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    uint8_t sr = 0;
    if (swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("SR1 przed WREN: 0x");
        printHex8(sr);
        printSr1Bits(sr);
    }

    const uint8_t wren[1] = {SWS_FLASH_WREN_CMD};
    uint8_t lo = 0, hi = 0;
    if (!swsFlashRawTx(wren, 1, &lo, &hi)) {
        g_out->print("-ERR transakcja WREN nieudana\r\n");
        return;
    }
    g_out->print("WREN: [0x0d] po CS w dol = 0x");
    printHex8(lo);
    g_out->print(", po CS w gore = 0x");
    printHex8(hi);
    if (!(hi & SWS_MSPI_CS)) g_out->print("  (CS nie wstal!)");
    g_out->print("\r\n");

    delay(2);
    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("-ERR odczyt SR1 nieudany\r\n");
        return;
    }
    g_out->print("SR1 po WREN:    0x");
    printHex8(sr);
    printSr1Bits(sr);
    g_out->print((sr & 0x02) ? "+OK WEL=1 - kosc przyjmuje WREN\r\n"
                            : "-ERR WEL=0 - WREN nie dotarl do kosci\r\n");
}

static void cmdWrsr(uint8_t val) {
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    if (!swsFlashWriteStatusReg(val)) {
        g_out->print("-ERR zapis rejestru statusu nieudany\r\n");
        return;
    }
    delay(5);

    uint8_t sr = 0;
    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("-ERR odczyt SR1 nieudany\r\n");
        return;
    }
    g_out->print("SR1 po zapisie 0x");
    printHex8(val);
    g_out->print(" = 0x");
    printHex8(sr);
    printSr1Bits(sr);
    g_out->print((sr & 0xFC) == (val & 0xFC) ? "+OK WRSR - rejestr przyjety\r\n"
                                             : "-ERR WRSR - rejestr nie przyjal wartosci\r\n");
}

/* --- UNLOCK: zdjecie blokad BP3..BP0 z kosci flash --------------------------- */
static void cmdUnlock() {
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    uint8_t sr = 0;
    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("-ERR odczyt SR1 nieudany\r\n");
        return;
    }
    g_out->print("SR1 przed: 0x");
    printHex8(sr);
    printSr1Bits(sr);

    if ((sr & 0x3C) == 0) {
        g_out->print("+OK kosc juz odblokowana (BP3..BP0 = 0)\r\n");
        return;
    }

    const uint8_t wren[1] = {SWS_FLASH_WREN_CMD};
    if (!swsFlashRawTx(wren, 1, nullptr, nullptr)) {
        g_out->print("-ERR WREN nieudany\r\n");
        return;
    }
    delay(2);

    const uint8_t val = sr & ~0x3Cu;   /* kasuj BP3..BP0, reszte zostaw */
    if (!swsFlashWriteStatusReg(val)) {
        g_out->print("-ERR WRSR nieudany\r\n");
        return;
    }
    delay(5);

    uint8_t rb = 0;
    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &rb, 1, false)) {
        g_out->print("-ERR odczyt SR1 po WRSR nieudany\r\n");
        return;
    }
    g_out->print("SR1 po:    0x");
    printHex8(rb);
    printSr1Bits(rb);
    g_out->print((rb & 0x3C) == 0 ? "+OK UNLOCK - blokady zdjete\r\n"
                                  : "-ERR UNLOCK - blokady nadal aktywne\r\n");
}

/* --- FCMD/FCMDR: dowolna komenda do kosci ------------------------------------ */
static void cmdFcmd(const uint8_t *b, uint32_t n, uint32_t readN) {
    if (!n) {
        g_out->print("-ERR brak bajtow\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    g_out->print("TX:");
    printHexBytes(b, n);
    g_out->print("\r\n");

    if (readN) {
        if (readN > SWS_SWS_BURST) readN = SWS_SWS_BURST;
        static uint8_t out[SWS_SWS_BURST];
        if (!swsFlashRawTxRead(b, n, out, readN)) {
            g_out->print("-ERR transakcja nieudana\r\n");
            return;
        }
        g_out->print("RX:");
        printHexBytes(out, readN);
        g_out->print(" |");
        for (uint32_t i = 0; i < readN; i++)
            g_out->print((out[i] >= 32 && out[i] < 127) ? (char)out[i] : '.');
        g_out->print("|\r\n");
    } else {
        uint8_t lo = 0, hi = 0;
        if (!swsFlashRawTx(b, n, &lo, &hi)) {
            g_out->print("-ERR transakcja nieudana\r\n");
            return;
        }
        g_out->print("[0x0d] po CS w dol = 0x");
        printHex8(lo);
        g_out->print(", po CS w gore = 0x");
        printHex8(hi);
        if (!(hi & SWS_MSPI_CS)) g_out->print("  (CS nie wstal!)");
        g_out->print("\r\n");
    }
    g_out->print("+OK FCMD\r\n");
}

/* --- ERASE1: kasowanie jednego sektora z pelna diagnostyka ------------------- */
static void cmdErase1(uint32_t addr) {
    addr &= ~(uint32_t)(SWS_FLASH_SECTOR - 1);
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    uint8_t sr = 0;
    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("-ERR odczyt SR1 nieudany\r\n");
        return;
    }
    g_out->print("SR1 przed:  0x");
    printHex8(sr);
    printSr1Bits(sr);

    if (!swsFlashWren()) {
        g_out->print("-ERR WREN nieudany\r\n");
        return;
    }
    if (swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("SR1 po WREN:0x");
        printHex8(sr);
        printSr1Bits(sr);
    } else {
        g_out->print("-ERR odczyt SR1 po WREN nieudany\r\n");
    }

    uint8_t cmd[4];
    cmd[0] = SWS_FLASH_ERASE4K_CMD;
    cmd[1] = (uint8_t)(addr >> 16);
    cmd[2] = (uint8_t)(addr >> 8);
    cmd[3] = (uint8_t)addr;

    g_out->print("Kasowanie sektora 0x");
    g_out->print(addr, HEX);
    g_out->print(" (komenda 0x");
    printHex8(SWS_FLASH_ERASE4K_CMD);
    g_out->print(" + adres):\r\nTX:");
    printHexBytes(cmd, 4);
    g_out->print("\r\n");

    uint8_t lo = 0, hi = 0;
    if (!swsFlashRawTx(cmd, 4, &lo, &hi)) {
        g_out->print("-ERR transakcja kasowania nieudana\r\n");
        return;
    }
    g_out->print("[0x0d] po CS w dol = 0x");
    printHex8(lo);
    g_out->print(", po CS w gore = 0x");
    printHex8(hi);
    g_out->print("\r\n");

    uint32_t ms = 0;
    bool saw = false;
    if (!waitWipClear(6000, &ms, &saw)) {
        g_out->print("-ERR odczyt SR1 w petli nieudany\r\n");
        return;
    }
    g_out->print("WIP=1 widziane: ");
    g_out->print(saw ? "tak" : "NIE");
    g_out->print(saw ? " (kosc kasowala)" : " (komenda zignorowana albo kosc gotowa od razu)");
    g_out->print(", czas do WIP=0: ");
    g_out->print(ms);
    g_out->print(" ms (realne kasowanie 4K = ~45 ms)\r\n");

    if (!swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("-ERR odczyt SR1 nieudany\r\n");
        return;
    }
    g_out->print("SR1 po:     0x");
    printHex8(sr);
    printSr1Bits(sr);

    uint8_t buf[16];
    if (!swsFlashRead(addr, buf, sizeof(buf))) {
        g_out->print("-ERR odczyt kontrolny nieudany\r\n");
        return;
    }
    g_out->print("Pierwsze 16 B sektora: ");
    printHexBytes(buf, sizeof(buf));
    g_out->print("\r\n");

    bool blank = true;
    for (uint32_t i = 0; i < sizeof(buf); i++)
        if (buf[i] != 0xFF) blank = false;

    if (blank) {
        g_out->print("+OK ERASE1 - sektor pusty (0xFF), kasowanie zadzialalo\r\n");
    } else if (saw) {
        g_out->print("-ERR ERASE1 - kosc raportowala prace, ale zawartosc sie nie zmienila (ochrona/WP#?)\r\n");
    } else {
        g_out->print("-ERR ERASE1 - kasowanie zignorowane (brak WEL albo komenda nie dotarla)\r\n");
    }
}

/* --- WRPG: zapis strony z natychmiastowym odczytem kontrolnym ---------------- */
static void cmdWrpg(uint32_t addr, const uint8_t *data, uint32_t n) {
    if (!n || n > SWS_FLASH_PAGE) {
        g_out->print("-ERR WRPG: 1..256 bajtow danych\r\n");
        return;
    }
    if ((addr & (SWS_FLASH_PAGE - 1)) + n > SWS_FLASH_PAGE) {
        g_out->print("-ERR WRPG: zapis nie moze przejsc przez granice strony\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;
    delay(2);

    uint8_t sr = 0;
    if (swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("SR1 przed:  0x");
        printHex8(sr);
        printSr1Bits(sr);
    }
    if (!swsFlashWren()) {
        g_out->print("-ERR WREN nieudany\r\n");
        return;
    }
    if (swsFlashStatusCmd(SWS_FLASH_STATUS_CMD, &sr, 1, false)) {
        g_out->print("SR1 po WREN:0x");
        printHex8(sr);
        printSr1Bits(sr);
    }

    static uint8_t tx[4 + SWS_FLASH_PAGE];
    tx[0] = SWS_FLASH_WRITE_CMD;
    tx[1] = (uint8_t)(addr >> 16);
    tx[2] = (uint8_t)(addr >> 8);
    tx[3] = (uint8_t)addr;
    memcpy(tx + 4, data, n);

    g_out->print("Zapis 0x");
    g_out->print(n);
    g_out->print(" B pod 0x");
    g_out->print(addr, HEX);
    g_out->print(" (komenda 0x");
    printHex8(SWS_FLASH_WRITE_CMD);
    g_out->print(" + adres + dane):\r\nTX:");
    printHexBytes(tx, 4);
    g_out->print("+ ");
    printHexBytes(data, n);
    g_out->print("\r\n");

    uint8_t lo = 0, hi = 0;
    if (!swsFlashRawTx(tx, 4 + n, &lo, &hi)) {
        g_out->print("-ERR transakcja zapisu nieudana\r\n");
        return;
    }
    g_out->print("[0x0d] po CS w dol = 0x");
    printHex8(lo);
    g_out->print(", po CS w gore = 0x");
    printHex8(hi);
    g_out->print("\r\n");

    uint32_t ms = 0;
    bool saw = false;
    if (!waitWipClear(3000, &ms, &saw)) {
        g_out->print("-ERR odczyt SR1 w petli nieudany\r\n");
        return;
    }
    g_out->print("WIP=1 widziane: ");
    g_out->print(saw ? "tak" : "NIE");
    g_out->print(", czas do WIP=0: ");
    g_out->print(ms);
    g_out->print(" ms (zapis strony typowo <3 ms, wiec NIE jest to rozstrzygajace)\r\n");

    static uint8_t rb[SWS_FLASH_PAGE];
    if (!swsFlashRead(addr, rb, n)) {
        g_out->print("-ERR odczyt kontrolny nieudany\r\n");
        return;
    }
    g_out->print("Odczyt kontrolny:  ");
    printHexBytes(rb, n);
    g_out->print("\r\n");

    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; i++)
        if (rb[i] != data[i]) bad++;

    if (bad == 0) {
        g_out->print("+OK WRPG - wszystkie ");
        g_out->print(n);
        g_out->print(" bajtow zgodne\r\n");
    } else {
        g_out->print("-ERR WRPG - niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print("/");
        g_out->print(n);
        g_out->print("\r\n");
    }
}

/* --- MSPIR/MSPIW: dostep do rejestrow i SRAM ukladu -------------------------- */
static void cmdMspir(uint32_t reg, uint32_t n) {
    if (!n) n = 1;
    if (n > 32) n = 32;
    if (!swsLinkTest(swsDivFromUnit())) return;

    uint8_t b[32];
    if (!swsReadReg(reg, b, n)) {
        g_out->print("-ERR odczyt nieudany\r\n");
        return;
    }
    g_out->print("Odczyt [0x");
    g_out->print(reg, HEX);
    g_out->print("] ");
    g_out->print(n);
    g_out->print(" B:");
    printHexBytes(b, n);
    g_out->print("\r\n+OK MSPIR\r\n");
}

static void cmdMspiw(uint32_t reg, const uint8_t *data, uint32_t n) {
    if (!n || n > 32) {
        g_out->print("-ERR MSPIW: 1..32 bajty\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;

    if (!swsWriteReg(reg, data, n)) {
        g_out->print("-ERR zapis nieudany\r\n");
        return;
    }
    uint8_t rb[32];
    if (!swsReadReg(reg, rb, n)) {
        g_out->print("-ERR odczyt kontrolny nieudany\r\n");
        return;
    }
    g_out->print("Zapis [0x");
    g_out->print(reg, HEX);
    g_out->print("]:");
    printHexBytes(data, n);
    g_out->print("\r\n");
    g_out->print("Odczyt       :");
    printHexBytes(rb, n);
    g_out->print("\r\n");

    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; i++)
        if (rb[i] != data[i]) bad++;
    if (bad) {
        g_out->print("-ERR MSPIW - niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print("/");
        g_out->print(n);
        g_out->print("\r\n");
    } else {
        g_out->print("+OK MSPIW\r\n");
    }
}

/* --- SRAMTEST: stopa bledow lacza SWS bez dotykania flasha ------------------- */
static void cmdSramtest(uint32_t n) {
    if (!n) n = 32;
    if (n > SWS_FLASH_PAGE) n = SWS_FLASH_PAGE;
    if (!swsLinkTest(swsDivFromUnit())) return;

    const uint32_t base = 0x40000;   /* SRAM ukladu w przestrzeni SWS (adresy CPU - 0x800000) */
    static uint8_t wr[SWS_FLASH_PAGE], rd[SWS_FLASH_PAGE];

    const uint8_t pat[4] = {0x00, 0xFF, 0xAA, 0x55};
    uint32_t totalBits = 0;
    uint32_t totalBytesBad = 0;

    for (int k = 0; k < 4; k++) {
        memset(wr, pat[k], n);
        if (!swsWriteReg(base, wr, n) || !swsReadReg(base, rd, n)) {
            g_out->print("-ERR operacja na SRAM nieudana\r\n");
            return;
        }
        uint32_t badBits = 0;
        uint32_t badBytes = 0;
        for (uint32_t i = 0; i < n; i++) {
            uint8_t x = (uint8_t)(wr[i] ^ rd[i]);
            if (!x) continue;
            badBytes++;
            while (x) {
                badBits += x & 1;
                x >>= 1;
            }
        }
        totalBits += badBits;
        totalBytesBad += badBytes;
        g_out->print("Wzorzec 0x");
        printHex8(pat[k]);
        g_out->print(": bledne bity = ");
        g_out->print(badBits);
        g_out->print("/");
        g_out->print(n * 8);
        g_out->print(", bledne bajty = ");
        g_out->print(badBytes);
        g_out->print("/");
        g_out->print(n);
        g_out->print("\r\n");
    }

    for (uint32_t i = 0; i < n; i++) wr[i] = (uint8_t)(i * 7 + 0x5A);
    if (!swsWriteReg(base, wr, n) || !swsReadReg(base, rd, n)) {
        g_out->print("-ERR operacja na SRAM nieudana\r\n");
        return;
    }
    uint32_t bad = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (rd[i] != wr[i]) bad++;
    }
    g_out->print("Wzorzec adresowy: blednych bajtow = ");
    g_out->print(bad);
    g_out->print("/");
    g_out->print(n);
    g_out->print("\r\n");

    totalBytesBad += bad;

    g_out->print("Razem: bledne bity (4 wzorce) = ");
    g_out->print(totalBits);
    g_out->print(", bledne bajty w 5 probach = ");
    g_out->print(totalBytesBad);
    g_out->print(", n = ");
    g_out->print(n);
    g_out->print("\r\n");
    g_out->print(totalBytesBad ? "+OK SRAMTEST - sa bledy transmisji, lacze zyje (liczby wyzej)\r\n"
                               : "+OK SRAMTEST - 0 bledow, lacze SWS bez zarzutu\r\n");
}

/* --- MEMTEST: test wlasnej pamieci ESP32 (PSRAM + SRAM), nie ukladu celu ------ */
static uint32_t memPatCheck(const uint8_t *buf, uint32_t n, uint8_t pat,
                            uint32_t *firstOff) {
    uint32_t bad = 0;
    bool haveFirst = false;
    for (uint32_t i = 0; i < n; i++) {
        if (buf[i] != pat) {
            bad++;
            if (!haveFirst) { *firstOff = i; haveFirst = true; }
        }
    }
    return bad;
}

static void memRunPatterns(const char *what, uint8_t *buf, uint32_t n) {
    const uint8_t pats[4] = {0xAA, 0x55, 0x00, 0xFF};
    for (int k = 0; k < 4; k++) {
        memset(buf, pats[k], n);
        uint32_t first = 0xFFFFFFFF;
        uint32_t bad = memPatCheck(buf, n, pats[k], &first);
        g_out->print(what);
        g_out->print(" wzorzec ");
        printHex8(pats[k]);
        g_out->print(": blednych bajtow = ");
        g_out->print(bad);
        if (bad) {
            g_out->print(" (pierwszy pod +0x");
            g_out->print(first, HEX);
            g_out->print(")");
        }
        g_out->print("\r\n");
    }

    /* Walking-1 / walking-0 na pierwszych 64 KB (szybki test adresacji). */
    uint32_t words = (n > (64u * 1024u)) ? (64u * 1024u) / 4 : n / 4;
    if (words) {
        uint32_t *w = (uint32_t *)buf;
        uint32_t badW = 0;
        for (uint32_t bit = 0; bit < 32; bit++) {
            uint32_t v = 1u << bit;
            for (uint32_t i = 0; i < words; i++) w[i] = v;
            for (uint32_t i = 0; i < words; i++) if (w[i] != v) badW++;
        }
        for (uint32_t bit = 0; bit < 32; bit++) {
            uint32_t v = ~(1u << bit);
            for (uint32_t i = 0; i < words; i++) w[i] = v;
            for (uint32_t i = 0; i < words; i++) if (w[i] != v) badW++;
        }
        g_out->print(what);
        g_out->print(" walking-1/0 (");
        g_out->print(words * 4);
        g_out->print(" B): blednych slow = ");
        g_out->print(badW);
        g_out->print("\r\n");
    }
}

static void cmdMemtest(uint32_t kb) {
    const uint32_t psram = ESP.getPsramSize();
    g_out->print("MEMTEST: PSRAM ");
    g_out->print(psram);
    g_out->print(" B, wolna sterta ");
    g_out->print((unsigned)ESP.getFreeHeap());
    g_out->print(" B\r\n");

    if (psram) {
        uint32_t want = psram / 2;
        if (want > (4u * 1024u * 1024u)) want = 4u * 1024u * 1024u;
        if (kb) {
            uint32_t asked = kb * 1024u;
            if (asked > psram / 2) asked = psram / 2;
            want = asked;
        }
        if (want < (16u * 1024u)) want = 16u * 1024u;

        uint32_t t0 = millis();
        uint8_t *buf = (uint8_t *)ps_malloc(want);
        if (buf) {
            g_out->print("PSRAM: testuje ");
            g_out->print(want);
            g_out->print(" B...\r\n");
            memRunPatterns("PSRAM", buf, want);
            g_out->print("+OK MEMTEST PSRAM ");
            g_out->print(want);
            g_out->print(" B, czas ");
            g_out->print((uint32_t)(millis() - t0));
            g_out->print(" ms\r\n");
            free(buf);
        } else {
            g_out->print("-ERR nie udalo sie zaalokowac ");
            g_out->print(want);
            g_out->print(" B w PSRAM\r\n");
        }
    }

    uint32_t freeHeap = (uint32_t)ESP.getFreeHeap();
    uint32_t sram = freeHeap > (64u * 1024u) ? (64u * 1024u) : freeHeap / 2;
    if (sram >= 4096u) {
        uint8_t *sbuf = (uint8_t *)malloc(sram);
        if (sbuf) {
            g_out->print("SRAM: testuje ");
            g_out->print(sram);
            g_out->print(" B...\r\n");
            memRunPatterns("SRAM", sbuf, sram);
            g_out->print("+OK MEMTEST SRAM ");
            g_out->print(sram);
            g_out->print(" B\r\n");
            free(sbuf);
        } else {
            g_out->print("-ERR nie udalo sie zaalokowac ");
            g_out->print(sram);
            g_out->print(" B w SRAM\r\n");
        }
    } else {
        g_out->print("SRAM: pominieto (za malo wolnej sterty)\r\n");
    }
}

/* --- CAP: logic analyzer na jednym pinie (gleboki bufor PSRAM) --------------- */
static void cmdCapture(uint32_t pin, uint32_t samples, uint32_t div) {
    if (pin > 48) {
        g_out->print("-ERR pin 0..48\r\n");
        return;
    }
    if (samples == 0) samples = 200000;
    uint32_t psFree = (uint32_t)ESP.getFreePsram();
    if (samples > psFree) {
        g_out->print("-ERR za duzo probek (wolne PSRAM ");
        g_out->print(psFree);
        g_out->print(" B)\r\n");
        return;
    }
    uint8_t *buf = (uint8_t *)ps_malloc(samples);
    if (!buf) {
        g_out->print("-ERR brak PSRAM\r\n");
        return;
    }

    pinMode(pin, INPUT);
    uint32_t t0 = micros();
    if (div <= 1) {
        for (uint32_t i = 0; i < samples; i++) buf[i] = digitalRead(pin) ? 1 : 0;
    } else {
        for (uint32_t i = 0; i < samples; i++) {
            buf[i] = digitalRead(pin) ? 1 : 0;
            delayMicroseconds(div - 1);
        }
    }
    uint32_t us = micros() - t0;

    g_out->print("+CAP pin=");
    g_out->print(pin);
    g_out->print(" probek=");
    g_out->print(samples);
    g_out->print(" czas=");
    g_out->print(us);
    g_out->print(" us (");
    if (us) g_out->print((uint32_t)((uint64_t)samples * 1000000u / us));
    else g_out->print('?');
    g_out->print(" probek/s)\r\n");

    uint32_t edges = 0, high = 0, run = 0;
    uint32_t minHigh = 0xFFFFFFFFu, maxHigh = 0;
    uint32_t minLow = 0xFFFFFFFFu, maxLow = 0;
    uint8_t last = buf[0];
    for (uint32_t i = 0; i < samples; i++) {
        if (buf[i]) high++;
        if (buf[i] == last) { run++; continue; }
        if (last) {
            edges++;
            if (run < minHigh) minHigh = run;
            if (run > maxHigh) maxHigh = run;
        } else {
            if (run < minLow) minLow = run;
            if (run > maxLow) maxLow = run;
        }
        last = buf[i];
        run = 1;
    }
    if (last) {
        if (run < minHigh) minHigh = run;
        if (run > maxHigh) maxHigh = run;
    } else {
        if (run < minLow) minLow = run;
        if (run > maxLow) maxLow = run;
    }

    g_out->print("krawedzi=");
    g_out->print(edges);
    g_out->print(" HIGH=");
    g_out->print(high);
    g_out->print("/");
    g_out->print(samples);
    g_out->print(" (");
    if (samples) g_out->print((uint32_t)((uint64_t)high * 100u / samples));
    g_out->print("%)\r\n");
    if (maxHigh) {
        g_out->print("HIGH min=");
        g_out->print(minHigh);
        g_out->print(" max=");
        g_out->print(maxHigh);
        g_out->print(" probek\r\n");
    }
    if (maxLow) {
        g_out->print("LOW  min=");
        g_out->print(minLow);
        g_out->print(" max=");
        g_out->print(maxLow);
        g_out->print(" probek\r\n");
    }
    g_out->print("+OK CAP\r\n");
    free(buf);
}

/* --- OSC: mini-oscyloskop ADC (gleboki bufor PSRAM) -------------------------- */
static void cmdOsc(uint32_t pin, uint32_t samples, uint32_t us) {
    if (pin > 48) {
        g_out->print("-ERR pin 0..48\r\n");
        return;
    }
    if (samples == 0) samples = 2000;
    if (samples > 65535) samples = 65535;
    uint16_t *buf = (uint16_t *)ps_malloc(samples * 2u);
    if (!buf) {
        g_out->print("-ERR brak PSRAM\r\n");
        return;
    }

    pinMode(pin, INPUT);
    analogSetPinAttenuation(pin, ADC_11db);
    uint32_t t0 = micros();
    for (uint32_t i = 0; i < samples; i++) {
        buf[i] = (uint16_t)analogRead(pin);
        if (us) delayMicroseconds(us);
    }
    uint32_t elapsed = micros() - t0;

    uint32_t minV = 65535u, maxV = 0;
    uint64_t sum = 0;
    for (uint32_t i = 0; i < samples; i++) {
        uint16_t v = buf[i];
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        sum += v;
    }
    uint32_t avg = (uint32_t)(sum / samples);

    g_out->print("+OSC pin=");
    g_out->print(pin);
    g_out->print(" probek=");
    g_out->print(samples);
    g_out->print(" czas=");
    g_out->print(elapsed);
    g_out->print(" us min=");
    g_out->print(minV);
    g_out->print(" max=");
    g_out->print(maxV);
    g_out->print(" avg=");
    g_out->print(avg);
    g_out->print("\r\n");

    uint32_t n = samples < 4096 ? samples : 4096;
    g_out->print("+ADC ");
    for (uint32_t i = 0; i < n; i++) {
        g_out->print(buf[i]);
        if (i + 1 < n) g_out->print(',');
    }
    g_out->print("\r\n");
    free(buf);
}

/* --- MSPIWATCH: czy rejestry MSPI zmieniaja sie bez naszego udzialu ---------- */
static void cmdMspiwatch(uint32_t ms) {
    if (!ms) ms = 2000;
    if (ms > 10000) ms = 10000;
    if (!swsLinkTest(swsDivFromUnit())) return;

    g_out->print("Nasluch rejestrow MSPI przez ");
    g_out->print(ms);
    g_out->print(" ms...\r\n");

    uint8_t lastC = 0xEE, lastD = 0xEE, lastM = 0xEE;
    uint32_t changes = 0;
    uint32_t samples = 0;
    const uint32_t t0 = millis();

    while ((uint32_t)(millis() - t0) < ms) {
        uint8_t c = 0, d = 0, m = 0;
        if (swsReadReg(0x000C, &c, 1) && swsReadReg(0x000D, &d, 1) && swsReadReg(0x00B3, &m, 1)) {
            samples++;
            if (c != lastC || d != lastD || m != lastM) {
                changes++;
                if (changes <= 20) {
                    g_out->print("  t=");
                    g_out->print(millis() - t0);
                    g_out->print(" ms  [0x0c]=0x");
                    printHex8(c);
                    g_out->print(" [0x0d]=0x");
                    printHex8(d);
                    g_out->print(" [0x00b3]=0x");
                    printHex8(m);
                    g_out->print("\r\n");
                }
                lastC = c;
                lastD = d;
                lastM = m;
            }
        }
        delay(20);
    }

    g_out->print("Prob: ");
    g_out->print(samples);
    g_out->print(", zmian: ");
    g_out->print(changes);
    g_out->print("\r\n");
    g_out->print(changes > 1
        ? "-ERR MSPIWATCH - rejestry MSPI zmieniaja sie same: CPU lub kosc pracuje w tle!\r\n"
        : "+OK MSPIWATCH - rejestry stabilne (nic nie zakloca nam dostepu do flasha)\r\n");
}

/* --- HALT: aktywacja + weryfikacja zatrzymania CPU --------------------------- */
static void cmdHalt(uint32_t ms) {
    if (ms < 50) ms = 300;
    g_out->print("Aktywacja: reset + wpis 0x05 do [0x0602], przytrzymanie ");
    g_out->print(ms);
    g_out->print(" ms...\r\n");

    if (!swsActivate(ms)) {
        g_out->print("-ERR aktywacja nieudana (brak odpowiedzi SWS)\r\n");
        return;
    }

    uint8_t cpu[2] = {0, 0};
    if (!swsReadReg(0x0602, cpu, 2)) {
        g_out->print("-ERR odczyt [0x0602] nieudany\r\n");
        return;
    }
    g_out->print("[0x0602] = 0x");
    printHex8(cpu[1]);
    printHex8(cpu[0]);
    g_out->print("\r\n");

    uint8_t div = 0;
    g_out->print("[0x00b2] = ");
    if (swsReadReg(0x00B2, &div, 1)) {
        g_out->print("0x");
        printHex8(div);
        g_out->print(" (oczekiwane 0x");
        printHex8(swsDivFromUnit());
        g_out->print(")\r\n");
    } else {
        g_out->print("BLAD\r\n");
    }

    g_out->print(cpu[0] == 0x05 ? "+OK HALT - CPU zatrzymany\r\n"
                                : "-ERR HALT - [0x0602] != 0x05, CPU moze dzialac!\r\n");
}

static void cmdBpoll(uint32_t on) {
    swsSetBusyPoll(on != 0);
    g_out->print("BPOLL = ");
    g_out->print(swsGetBusyPoll() ? "1" : "0");
    g_out->print(" (czekanie na BUSY przed zapisem bajtu do [0x0c])\r\n+OK BPOLL\r\n");
}

/* --- RDHEX: podglad flasha --------------------------------------------------- */
static void cmdRdhex(uint32_t addr, uint32_t len) {
    if (!len) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if (len > 256) len = 256;
    if (!swsLinkTest(swsDivFromUnit())) return;

    uint8_t buf[16];
    for (uint32_t off = 0; off < len;) {
        uint32_t c = len - off;
        if (c > sizeof(buf)) c = sizeof(buf);
        if (!swsFlashRead(addr + off, buf, c)) {
            g_out->print("-ERR odczyt nieudany\r\n");
            return;
        }
        g_out->print("0x");
        outPrintf("%06X: ", (unsigned)(addr + off));
        printHexBytes(buf, c);
        g_out->print(" |");
        for (uint32_t i = 0; i < c; i++)
            g_out->print((buf[i] >= 32 && buf[i] < 127) ? (char)buf[i] : '.');
        g_out->print("|\r\n");
        off += c;
    }
    g_out->print("+OK RDHEX\r\n");
}

/* --- LISTEN: podglad tego, co uklad nadaje po UART --------------------------- */

/* Znaki sterujace jako \r \n \t, resztę nieczytelną jako \xNN - żeby w logu
 * było widać dokładnie, co przyszło (bez psucia konsoli). */
static void listenEscape(const uint8_t *b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) {
        const uint8_t c = b[i];
        if (c == '\r' || c == '\n' || c == '\t') {
            g_out->print('\\');
            g_out->print(c == '\r' ? 'r' : (c == '\n' ? 'n' : 't'));
        } else if (c >= 32 && c < 127) {
            g_out->write(c);
        } else {
            outPrintf("\\x%02X", c);
        }
    }
}

/* Wypisuje odebrany wiersz od razu na konsole hosta (podglad na zywo). */
static void listenLiveFlush(uint8_t *line, uint32_t &len, uint32_t tms, uint32_t *lines) {
    if (!len) return;
    outPrintf("  > t=%5u ", (unsigned)tms);
    listenEscape(line, len);
    g_out->print("\r\n");
    len = 0;
    (*lines)++;
}

/* LISTEN [ms] [baud] [NONE|RST|GO]
 *   NONE - tylko nasluch, nic nie dotyka ukladu
 *   RST  - impuls resetu celu, potem nasluch (firmware startuje od 0x0)
 *   GO   - aktywacja SWS + [0x0602]=0x88 (CPU ReBoot), tak jak po programowaniu
 *
 * UWAGA: każda aktywacja SWS wpisuje [0x0602]=0x05, czyli ZATRZYMUJE CPU celu.
 * Dlatego po trybie GO nie wolno już wysłać żadnej innej komendy SWS przed
 * nasłuchem - inaczej uklad znowu stanie.
 */
static void cmdListen(uint32_t ms, uint32_t baud, const char *action) {
    if (!ms) ms = 2000;
    if (ms > 600000) ms = 600000;
    if (!baud) baud = 115200;

    g_out->print("Nasluch UART: GPIO");
    g_out->print(UART_RX_PIN);
    g_out->print(" @");
    g_out->print(baud);
    g_out->print(", ");
    g_out->print(ms);
    g_out->print(" ms, tryb ");
    g_out->print(action);
    g_out->print("\r\n");

    if (!strcasecmp(action, "GO")) {
        if (!swsLinkTest(swsDivFromUnit())) return;
        const uint8_t go = 0x88;
        if (!swsWriteReg(0x0602, &go, 1)) {
            g_out->print("-ERR zapis [0x0602] nieudany\r\n");
            return;
        }
        uint8_t rb = 0;
        if (!swsReadReg(0x0602, &rb, 1)) {
            g_out->print("-ERR odczyt kontrolny [0x0602] nieudany\r\n");
            return;
        }
        outPrintf("  [0x0602]: zapis 0x88 -> odczyt 0x%02X "
                      "(bit 7 kasuje sie sam; 0x08 = CPU Go)\r\n", rb);
    } else if (!strcasecmp(action, "RST")) {
        swsResetTargetPulse(50);
        delay(150);
    } else if (strcasecmp(action, "NONE")) {
        g_out->print("-ERR tryb musi byc NONE, RST albo GO\r\n");
        return;
    }

    static uint8_t buf[8192];
    uint32_t n = 0;
    uint32_t total = 0;

    /* Bufor linii dla podgladu NA ZYWO: kazdy odebrany wiersz leci od razu do
     * konsoli (host go wyswietla w trakcie nasluchu), a rownolegle wszystko
     * ladauje w `buf` na koncowy zrzut HEX. */
    static uint8_t line[160];
    uint32_t lineLen = 0;
    uint32_t lines = 0;
    uint32_t lastByte = 0;

    Serial1.end();
    delay(2);
    const int idle = digitalRead(UART_RX_PIN);
    g_out->print("  --- podglad na zywo (wiersze w trakcie nasluchu) ---\r\n");

    Serial1.begin(baud, SERIAL_8N1, UART_RX_PIN, -1);
    const uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        while (Serial1.available()) {
            const int c = Serial1.read();
            if (c < 0) break;
            total++;
            if (n < sizeof(buf)) buf[n++] = (uint8_t)c;

            line[lineLen++] = (uint8_t)c;
            lastByte = millis();
            /* Wiersz konczy \n albo zapelnienie bufora - \r sam nie konczy,
             * bo para \r\n dawalaby dwa wiersze na jedna linie danych. */
            if (c == '\n' || lineLen >= sizeof(line)) {
                listenLiveFlush(line, lineLen, millis() - t0, &lines);
            }
        }
        vTaskDelay(1);
        /* Rozdzielenie po ciszy: strumien bez \n (albo urwany wiersz) i tak
         * trafia do podgladu po 40 ms przerwy. */
        if (lineLen && (millis() - lastByte) > 40) {
            listenLiveFlush(line, lineLen, millis() - t0, &lines);
        }
    }
    Serial1.end();

    listenLiveFlush(line, lineLen, millis() - t0, &lines);

    g_out->print("  bajtow: ");
    g_out->print(total);
    g_out->print(", wierszy: ");
    g_out->print(lines);
    if (total > n) {
        g_out->print(" (zapisano pierwsze ");
        g_out->print(n);
        g_out->print(")");
    }
    g_out->print("; stan linii przed nasluchem: ");
    g_out->print(idle);
    g_out->print("\r\n");

    if (n) {
        g_out->print("  HEX:\r\n");
        for (uint32_t off = 0; off < n && off < 512; off += 32) {
            uint32_t c = n - off;
            if (c > 32) c = 32;
            outPrintf("    %04X:", (unsigned)off);
            printHexBytes(buf + off, c);
            g_out->print("\r\n");
        }
        if (n > 512) g_out->print("    ... (hex obciety do 512 B)\r\n");
    }
    g_out->print("+OK LISTEN\r\n");
}

/* READW <adr> <len> - odczyt flash BAJT PO BAJCIE.
 * Odczyt wielobajtowy idzie przez FIFO i przekłamuje ~2% bajtów (zależnie od
 * danych), więc do weryfikacji i kopii używamy ścieżki pojedynczego bajtu -
 * wolniejszej (~0,5 ms/B), ale powtarzalnej. Host czyta to jak READ, tylko
 * nagłówek jest "+DATAB". */
static void cmdReadW(uint32_t start, uint32_t len) {
    if (!len) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if (!swsLinkTest(swsDivFromUnit())) return;

    g_out->print("+DATAB ");
    g_out->print(len);
    g_out->print("\r\n");

    static uint8_t buf[64];
    uint32_t off = 0;
    while (off < len) {
        uint32_t c = len - off;
        if (c > sizeof(buf)) c = sizeof(buf);
        bool ok = true;
        for (uint32_t i = 0; i < c; i++) {
            if (!swsFlashRead(start + off + i, buf + i, 1)) {
                ok = false;   // błąd w trakcie - host wykryje krótki odczyt
                break;
            }
        }
        if (!ok) break;
        g_out->write(buf, c);
        off += c;
    }
}

/* --- komendy SPI flash (25xx, klips SOIC-8) ---------------------------------- */

static void cmdSpiDetect() {
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        g_out->print("-ERR brak kosci SPI (JEDEC nie odpowiada)\r\n");
        return;
    }
    g_out->print("+OK JEDEC 0x");
    g_out->print(jedec, HEX);
    uint32_t sz = spiFlashChipSize(jedec);
    g_out->print(" -> ");
    g_out->print(spiFlashJedecName(jedec));
    if (sz > SPI_FLASH_MAX_ADDR) {
        g_out->print(" (adresacja 3B obsluguje max 16 MB)");
    }
    g_out->print("\r\n");
}

static void cmdSpiBlank(uint32_t start, uint32_t len) {
    if (len == 0) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if ((uint64_t)start + len > SPI_FLASH_MAX_ADDR) {
        g_out->print("-ERR poza zakresem adresacji 3B (max 16 MB)\r\n");
        return;
    }
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        g_out->print("-ERR brak kosci SPI\r\n");
        return;
    }
    g_out->print("SPI blank check 0x");
    g_out->print(start, HEX);
    g_out->print(" + ");
    g_out->print(len);
    g_out->print(" B...\r\n");
    uint32_t first = 0, nonBlank = 0;
    if (!spiFlashBlankCheck(start, len, &first, &nonBlank)) {
        g_out->print("-ERR odczyt SPI nieudany\r\n");
        return;
    }
    if (nonBlank == 0) {
        g_out->print("+OK obszar czysty (same 0xFF)\r\n");
    } else {
        g_out->print("-ERR niepustych bajtow: ");
        g_out->print(nonBlank);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(first, HEX);
        g_out->print(")\r\n");
    }
}

static void cmdSpiRead(uint32_t start, uint32_t len) {
    if (len == 0) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if ((uint64_t)start + len > SPI_FLASH_MAX_ADDR) {
        g_out->print("-ERR poza zakresem adresacji 3B (max 16 MB)\r\n");
        return;
    }
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        g_out->print("-ERR brak kosci SPI\r\n");
        return;
    }
    g_out->print("+DATA ");
    g_out->print(len);
    g_out->print("\r\n");

    static uint8_t buf[512];
    uint32_t a = start, rem = len;
    while (rem) {
        uint32_t c = rem < sizeof(buf) ? rem : (uint32_t)sizeof(buf);
        if (!spiFlashRead(a, buf, c)) break;  // host wykryje krótki odczyt
        g_out->write(buf, c);
        a += c;
        rem -= c;
    }
}

static void cmdSpiHex(uint32_t start, uint32_t len) {
    if (len == 0 || len > 256) {
        g_out->print("-ERR dlugosc 1..256\r\n");
        return;
    }
    uint8_t buf[256];
    if (!spiFlashRead(start, buf, len)) {
        g_out->print("-ERR odczyt nieudany\r\n");
        return;
    }
    for (uint32_t off = 0; off < len;) {
        uint32_t c = len - off;
        if (c > 16) c = 16;
        g_out->print("0x");
        outPrintf("%06X: ", (unsigned)(start + off));
        printHexBytes(buf + off, c);
        g_out->print(" |");
        for (uint32_t i = 0; i < c; i++)
            g_out->print((buf[off + i] >= 32 && buf[off + i] < 127) ? (char)buf[off + i] : '.');
        g_out->print("|\r\n");
        off += c;
    }
    g_out->print("+OK SPIHEX\r\n");
}

static void cmdSpiErase(uint32_t start, uint32_t len) {
    if (len == 0) {
        g_out->print("-ERR dlugosc 0\r\n");
        return;
    }
    if ((uint64_t)start + len > SPI_FLASH_MAX_ADDR) {
        g_out->print("-ERR poza zakresem adresacji 3B (max 16 MB)\r\n");
        return;
    }
    uint32_t s = start & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    uint32_t e = (start + len + SPI_FLASH_SECTOR - 1) & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    for (uint32_t a = s; a < e; a += SPI_FLASH_SECTOR) {
        if (!spiFlashEraseSector(a)) {
            g_out->print("-ERR kasowanie sektora 0x");
            g_out->print(a, HEX);
            g_out->print("\r\n");
            return;
        }
    }
    g_out->print("+OK\r\n");
}

static void cmdSpiEraseChip() {
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        g_out->print("-ERR brak kosci SPI\r\n");
        return;
    }
    g_out->print("Kasowanie calej kosci SPI (moze trwac kilkadziesiat sekund)...\r\n");
    if (spiFlashEraseChip()) g_out->print("+OK\r\n");
    else g_out->print("-ERR kasowanie nieudane\r\n");
}

static void cmdSpiWrite(uint32_t addr, uint32_t size) {
    if (size == 0 || (uint64_t)addr + size > SPI_FLASH_MAX_ADDR) {
        g_out->print("-ERR rozmiar/adres poza zakresem (max 16 MB)\r\n");
        return;
    }
    if (addr % SPI_FLASH_SECTOR) {
        g_out->print("-ERR adres musi byc wyrownany do 4 KB\r\n");
        return;
    }
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        g_out->print("-ERR brak kosci SPI\r\n");
        return;
    }
    spiRxActive = true;
    spiRxRemaining = size;
    spiRxAddr = addr;
    g_out->print("+READY\r\n");
}

/* --- karta microSD (Catalex, drugi SPI) ------------------------------------- */
static void cmdSdTest() {
    sdCardProbe(*g_out);
}

static void cmdSdInfo() {
    if (!sdCardBegin()) {
        g_out->print("-ERR brak karty microSD (sprawdz zasilanie 5V i piny)\r\n");
        return;
    }
    g_out->print("+OK SD typ=");
    g_out->print(sdCardTypeName());
    g_out->print(" pojemnosc=");
    g_out->print((unsigned long long)sdCardTotalBytes());
    g_out->print(" B, zajete=");
    g_out->print((unsigned long long)sdCardUsedBytes());
    g_out->print(" B, wolne=");
    g_out->print((unsigned long long)(sdCardTotalBytes() - sdCardUsedBytes()));
    g_out->print(" B\r\n");
}

static void cmdSdLs(const char *path) {
    if (!sdCardBegin()) {
        g_out->print("-ERR brak karty microSD\r\n");
        return;
    }
    g_out->print("+DATA\r\n");
    if (!sdCardLs(*g_out, path ? path : "/")) {
        g_out->print("-ERR nie mozna otworzyc katalogu\r\n");
        return;
    }
    g_out->print("+OK\r\n");
}

static void cmdSdCat(const char *path) {
    if (!sdCardBegin()) {
        g_out->print("-ERR brak karty microSD\r\n");
        return;
    }
    g_out->print("+DATA\r\n");
    if (!sdCardCat(*g_out, path)) {
        g_out->print("-ERR nie mozna otworzyc pliku\r\n");
        return;
    }
    g_out->print("\r\n+OK\r\n");
}

static void cmdSdAppend(const char *path, const char *text) {
    if (!sdCardBegin()) {
        g_out->print("-ERR brak karty microSD\r\n");
        return;
    }
    if (!path || !text) {
        g_out->print("-ERR uzycie: SDAPPEND <plik> <tekst...>\r\n");
        return;
    }
    char line[256];
    size_t t = strlen(text);
    if (t > sizeof(line) - 2) t = sizeof(line) - 2;
    memcpy(line, text, t);
    line[t] = '\r';
    line[t + 1] = '\n';
    if (sdCardWrite(path, (const uint8_t *)line, t + 2, true)) {
        g_out->print("+OK SDAPPEND\r\n");
    } else {
        g_out->print("-ERR zapis nieudany\r\n");
    }
}

static void cmdSdDel(const char *path) {
    if (!sdCardBegin()) {
        g_out->print("-ERR brak karty microSD\r\n");
        return;
    }
    if (sdCardDelete(path)) {
        g_out->print("+OK SDDEL\r\n");
    } else {
        g_out->print("-ERR usuniecie nieudane\r\n");
    }
}

void dispatch(char *line) {
    char *p = line;
    char *cmd = nextTok(p);
    if (!cmd) return;

    if (!strcasecmp(cmd, "HELP") || !strcmp(cmd, "?")) {
        cmdHelp();
    } else if (!strcasecmp(cmd, "PINS")) {
        cmdPins();
    } else if (!strcasecmp(cmd, "CHIP")) {
        cmdChip();
    } else if (!strcasecmp(cmd, "CHIPID") || !strcasecmp(cmd, "ID")) {
        cmdChipId();
    } else if (!strcasecmp(cmd, "SWSTEST")) {
        uint32_t d = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, d);
        cmdSwstest(d);
    } else if (!strcasecmp(cmd, "PHASE")) {
        uint32_t q = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, q) && q >= 1 && q <= 60) {
            swsSetSampleQuarter(q);
            g_out->print("+OK PHASE ");
            g_out->print(swsGetSampleQuarter());
            g_out->print("/4 jednostki (");
            g_out->print(q * swsGetUnitUs() / 4.0, 2);
            g_out->print(" us)\r\n");
        } else {
            g_out->print("-ERR uzycie: PHASE <cwiartki 1..60> (10 = 2,5 jednostki)\r\n");
        }
    } else if (!strcasecmp(cmd, "WIDTHS")) {
        uint32_t n = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, n);
        cmdWidths(n);
    } else if (!strcasecmp(cmd, "WAVE")) {
        uint32_t n = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, n);
        cmdWave(n);
    } else if (!strcasecmp(cmd, "JEDEC")) {
        uint32_t jedec = 0;
        if (jedecRobust(&jedec)) {
            g_out->print("+OK JEDEC 0x");
            g_out->print(jedec, HEX);
            g_out->print("\r\n");
        } else {
            g_out->print("-ERR odczyt JEDEC nieudany\r\n");
        }
    } else if (!strcasecmp(cmd, "UNIT")) {
        uint32_t us = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, us) && us > 0 && us <= 4) {
            swsSetUnitUs(us);
            g_out->print("+OK UNIT ");
            g_out->print(swsGetUnitUs());
            g_out->print(" us, docelowy [0x00b2]=");
            g_out->print(swsDivFromUnit());
            g_out->print("\r\n");
        } else {
            g_out->print("-ERR uzycie: UNIT <us> (1..4; 4 = dzielnik 127, 3,97 us)\r\n");
        }
    } else if (!strcasecmp(cmd, "VMEAS")) {
        uint32_t pinArg = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, pinArg)) cmdVmeas((int)pinArg);
        else g_out->print("-ERR uzycie: VMEAS <pin> (GPIO1..GPIO10)\r\n");
    } else if (!strcasecmp(cmd, "VSCAN")) {
        cmdVscan();
    } else if (!strcasecmp(cmd, "PWM")) {
        uint32_t pin = 0, f = 1000, d = 50;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        char *t3 = nextTok(p);
        if (parseU32(t1, pin) && parseU32(t2, f) && parseU32(t3, d)) cmdPwm(pin, f, d);
        else g_out->print("-ERR uzycie: PWM <pin> <freq Hz> <wypelnienie 0..100>\r\n");
    } else if (!strcasecmp(cmd, "PWMSTOP")) {
        uint32_t pin = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, pin)) cmdPwmStop(pin);
        else g_out->print("-ERR uzycie: PWMSTOP <pin>\r\n");
    } else if (!strcasecmp(cmd, "PROBE")) {
        cmdProbe();
    } else if (!strcasecmp(cmd, "RESET")) {
        swsResetTargetPulse(50);
        g_out->print("+OK\r\n");
    } else if (!strcasecmp(cmd, "READ")) {
        uint32_t a, l;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdRead(a, l);
        else g_out->print("-ERR uzycie: READ <adr> <len>\r\n");
    } else if (!strcasecmp(cmd, "ERASE")) {
        uint32_t a, l;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdErase(a, l);
        else g_out->print("-ERR uzycie: ERASE <start> <len>\r\n");
    } else if (!strcasecmp(cmd, "WRITE")) {
        uint32_t s, o = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, s)) {
            if (t2) parseU32(t2, o);
            cmdWrite(s, o);
        } else {
            g_out->print("-ERR uzycie: WRITE <rozmiar> [adr]\r\n");
        }
    } else if (!strcasecmp(cmd, "STAT")) {
        cmdStat();
    } else if (!strcasecmp(cmd, "UNLOCK")) {
        cmdUnlock();
    } else if (!strcasecmp(cmd, "WEL")) {
        cmdWel();
    } else if (!strcasecmp(cmd, "WRSR")) {
        uint32_t v = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, v)) cmdWrsr((uint8_t)v);
        else g_out->print("-ERR uzycie: WRSR <hex, np. WRSR 0>\r\n");
    } else if (!strcasecmp(cmd, "FCMD") || !strcasecmp(cmd, "FCMDR")) {
        const bool wantRead = !strcasecmp(cmd, "FCMDR");
        uint32_t readN = 0;
        char *t1 = nextTok(p);
        if (wantRead) {
            if (!parseU32(t1, readN)) {
                g_out->print("-ERR uzycie: FCMDR <n> <hex...>\r\n");
                return;
            }
        }
        static uint8_t bytes[SWS_FLASH_PAGE];
        const uint32_t n = parseHexAll(p, bytes, sizeof(bytes));
        cmdFcmd(bytes, n, wantRead ? readN : 0);
    } else if (!strcasecmp(cmd, "ERASE1")) {
        uint32_t a = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, a)) cmdErase1(a);
        else g_out->print("-ERR uzycie: ERASE1 <adr>\r\n");
    } else if (!strcasecmp(cmd, "WRPG")) {
        uint32_t a = 0;
        char *t1 = nextTok(p);
        if (!parseU32(t1, a)) {
            g_out->print("-ERR uzycie: WRPG <adr> <hex...>\r\n");
            return;
        }
        static uint8_t bytes[SWS_FLASH_PAGE];
        const uint32_t n = parseHexAll(p, bytes, sizeof(bytes));
        cmdWrpg(a, bytes, n);
    } else if (!strcasecmp(cmd, "MSPIR")) {
        uint32_t reg = 0, n = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, reg)) {
            if (t2) parseU32(t2, n);
            cmdMspir(reg, n);
        } else {
            g_out->print("-ERR uzycie: MSPIR <adr> [n]\r\n");
        }
    } else if (!strcasecmp(cmd, "MSPIW")) {
        uint32_t reg = 0;
        char *t1 = nextTok(p);
        if (!parseU32(t1, reg)) {
            g_out->print("-ERR uzycie: MSPIW <adr> <hex...>\r\n");
            return;
        }
        uint8_t bytes[32];
        const uint32_t n = parseHexAll(p, bytes, sizeof(bytes));
        cmdMspiw(reg, bytes, n);
    } else if (!strcasecmp(cmd, "SRAMTEST")) {
        uint32_t n = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, n);
        cmdSramtest(n);
    } else if (!strcasecmp(cmd, "MEMTEST")) {
        uint32_t kb = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, kb);
        cmdMemtest(kb);
    } else if (!strcasecmp(cmd, "CAP")) {
        uint32_t pin = 0, samples = 0, div = 0;
        char *t1 = nextTok(p); if (t1) parseU32(t1, pin);
        char *t2 = nextTok(p); if (t2) parseU32(t2, samples);
        char *t3 = nextTok(p); if (t3) parseU32(t3, div);
        cmdCapture(pin, samples, div);
    } else if (!strcasecmp(cmd, "OSC")) {
        uint32_t pin = 0, samples = 0, us = 0;
        char *t1 = nextTok(p); if (t1) parseU32(t1, pin);
        char *t2 = nextTok(p); if (t2) parseU32(t2, samples);
        char *t3 = nextTok(p); if (t3) parseU32(t3, us);
        cmdOsc(pin, samples, us);
    } else if (!strcasecmp(cmd, "MSPIWATCH")) {
        uint32_t ms = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, ms);
        cmdMspiwatch(ms);
    } else if (!strcasecmp(cmd, "HALT")) {
        uint32_t ms = 0;
        char *t1 = nextTok(p);
        if (t1) parseU32(t1, ms);
        cmdHalt(ms);
    } else if (!strcasecmp(cmd, "BPOLL")) {
        uint32_t on = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, on)) cmdBpoll(on);
        else g_out->print("-ERR uzycie: BPOLL <0|1>\r\n");
    } else if (!strcasecmp(cmd, "RDHEX")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdRdhex(a, l);
        else g_out->print("-ERR uzycie: RDHEX <adr> <len>\r\n");
    } else if (!strcasecmp(cmd, "LISTEN")) {
        uint32_t ms = 0, baud = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        char *t3 = nextTok(p);
        if (t1) parseU32(t1, ms);
        if (t2) parseU32(t2, baud);
        cmdListen(ms, baud, t3 ? t3 : "NONE");
    } else if (!strcasecmp(cmd, "READW")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdReadW(a, l);
        else g_out->print("-ERR uzycie: READW <adr> <len>\r\n");
    } else if (!strcasecmp(cmd, "AREG")) {
        uint32_t a = 0;
        char *t1 = nextTok(p);
        if (parseU32(t1, a)) cmdAreg(a);
        else g_out->print("-ERR uzycie: AREG <adr 0x00..0xFF>\r\n");
    } else if (!strcasecmp(cmd, "AWR")) {
        uint32_t a = 0, v = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, v)) cmdAwrite(a, (uint8_t)v);
        else g_out->print("-ERR uzycie: AWR <adr> <wartosc 0..255>\r\n");
    } else if (!strcasecmp(cmd, "ADUMP")) {
        cmdAdump();
    } else if (!strcasecmp(cmd, "SPI")) {
        cmdSpiDetect();
    } else if (!strcasecmp(cmd, "SPIREAD")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdSpiRead(a, l);
        else g_out->print("-ERR uzycie: SPIREAD <adr> <len>\r\n");
    } else if (!strcasecmp(cmd, "SPIBLANK")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdSpiBlank(a, l);
        else g_out->print("-ERR uzycie: SPIBLANK <adr> <len>\r\n");
    } else if (!strcasecmp(cmd, "SPIHEX")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdSpiHex(a, l);
        else g_out->print("-ERR uzycie: SPIHEX <adr> <len (max 256)>\r\n");
    } else if (!strcasecmp(cmd, "SPIERASE")) {
        uint32_t a = 0, l = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, l)) cmdSpiErase(a, l);
        else g_out->print("-ERR uzycie: SPIERASE <start> <len>\r\n");
    } else if (!strcasecmp(cmd, "SPIERASECHIP")) {
        cmdSpiEraseChip();
    } else if (!strcasecmp(cmd, "SPIWRITE")) {
        uint32_t a = 0, s = 0;
        char *t1 = nextTok(p);
        char *t2 = nextTok(p);
        if (parseU32(t1, a) && parseU32(t2, s)) cmdSpiWrite(a, s);
        else g_out->print("-ERR uzycie: SPIWRITE <adr> <rozmiar>\r\n");
    } else if (!strcasecmp(cmd, "SD")) {
        cmdSdInfo();
    } else if (!strcasecmp(cmd, "SDTEST")) {
        cmdSdTest();
    } else if (!strcasecmp(cmd, "SDLS")) {
        cmdSdLs(nextTok(p));
    } else if (!strcasecmp(cmd, "SDCAT")) {
        cmdSdCat(nextTok(p));
    } else if (!strcasecmp(cmd, "SDDEL")) {
        cmdSdDel(nextTok(p));
    } else if (!strcasecmp(cmd, "SDAPPEND")) {
        char *path = nextTok(p);
        while (*p == ' ' || *p == '\t') p++;
        cmdSdAppend(path, *p ? p : nullptr);
    } else if (!strcasecmp(cmd, "WIFISCAN")) {
        cmdWifiScan();
    } else if (!strcasecmp(cmd, "WIFISTAT")) {
        cmdWifiStat();
    } else if (!strcasecmp(cmd, "WIFIJOIN")) {
        cmdWifiJoin(p);
    } else if (!strcasecmp(cmd, "WIFICLEAR")) {
        cmdWifiForget();
    } else {
        g_out->print("-ERR nieznana komenda (HELP = lista)\r\n");
    }
}

/* --- setup / loop ------------------------------------------------------------ */
void setup() {
    Serial.begin(115200);
    delay(200);

    g_out->print("\r\n");
    g_out->print("=== SWS Programmer (TLSR825x) ===\r\n");
    g_out->print("Piny: ");
    cmdPins();
    g_out->print("Konsola na COM11 @ 115200. Komenda HELP = pomoc.\r\n");
    g_out->print("UWAGA: przed podlaczeniem sprawdz opis P2 w README.md.\r\n\r\n");

    swsMasterInit();
    g_out->print("SWS: bit-bang, UNIT ");
    g_out->print(swsGetUnitUs());
    g_out->print(" us (zmien komenda UNIT)\r\n");
    g_out->print("Uzyj SWSTEST aby sprawdzic lacze z ukladem.\r\n\r\n");

    spiFlashBegin();
    g_out->print("SPI flash: gotowy (SCK=");
    g_out->print(SPI_SCK_PIN);
    g_out->print(" MISO=");
    g_out->print(SPI_MISO_PIN);
    g_out->print(" MOSI=");
    g_out->print(SPI_MOSI_PIN);
    g_out->print(" CS=");
    g_out->print(SPI_CS_PIN);
    g_out->print("). Komenda SPI = wykryj kosc.\r\n\r\n");
    g_out->print("PSRAM: ");
    g_out->print((unsigned)ESP.getPsramSize());
    g_out->print(" bajtow\r\n\r\n");

    if (sdCardBegin()) {
        g_out->print("SD: ");
        g_out->print(sdCardTypeName());
        g_out->print(" ");
        g_out->print((unsigned long long)sdCardTotalBytes());
        g_out->print(" B (komenda SD = szczegoly)\r\n");
    } else {
        g_out->print("SD: brak karty (komenda SD = ponow wykrycie)\r\n");
    }
    g_out->print("\r\n");

    webuiSetup();
}

void loop() {
    webuiLoop();

    if (spiRxActive) {
        spiHandleRxChunk();
        return;
    }

    if (rxActive) {
        handleRxChunk();
        return;
    }

    while (Serial.available()) {
        int c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (lineLen) {
                lineBuf[lineLen] = 0;
                dispatch(lineBuf);
                lineLen = 0;
            }
        } else if (c >= 32 && lineLen < sizeof(lineBuf) - 1) {
            lineBuf[lineLen++] = (char)c;
        }
        /* WRITE/SPIWRITE właśnie przeszedł w tryb odbioru binarnego -
           przerwij czytanie linii, żeby nie zjeść nadchodzących danych. */
        if (rxActive || spiRxActive) break;
    }
}
