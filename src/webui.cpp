/* =============================================================================
 * Warstwa WWW programatora SWS (TLSR825x).
 *
 * ESP32-S3 tworzy SoftAP "SWS-Programmer" (http://192.168.4.1) z prostym
 * panelem do: detekcji ukladu, weryfikacji obrazu .bin i wgrania firmware
 * (kasowanie + zapis + weryfikacja + reset). Konsola COM11 dziala nadal.
 *
 * UWAGA: wgrywanie zmienia pamiec flash ukladu docelowego. Serwer wymaga
 * jawnej zgody (parametr consent=1) przy kazdej operacji zapisu.
 * =============================================================================
 */

#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <Wire.h>
#include <OneWire.h>

#include "config.h"
#include "sws_flash.h"
#include "sws_master.h"
#include "spi_flash.h"
#include "sd_card.h"
#include "webui.h"

static WebServer server(80);

/* Wspolny bufor odbioru pliku .bin (SWS + SPI). Rozmiar = wiekszy z limitow. */
#define WEB_BUF_SIZE ((WEB_MAX_FW) > (SPI_WEB_MAX) ? (WEB_MAX_FW) : (SPI_WEB_MAX))

static uint8_t *upBuf = nullptr;
static size_t upLen = 0;
static bool upOverflow = false;
static String upName;

/* --- pierscieniowy bufor logow (zywy podglad w WWW) -------------------------- */
#define LOG_RING_SIZE 16384u   // potega 2 - indeks = seq & (SIZE-1)
static char logRing[LOG_RING_SIZE];
static uint32_t logSeq = 0;    // laczna liczba dolaczonych bajtow

void webLogWrite(const uint8_t *p, size_t n) {
    for (size_t i = 0; i < n; i++) {
        logRing[logSeq & (LOG_RING_SIZE - 1)] = (char)p[i];
        logSeq++;
    }
}

static String webLogSince(uint32_t after) {
    uint32_t have = logSeq;
    uint32_t oldest = have > LOG_RING_SIZE ? have - LOG_RING_SIZE : 0;
    uint32_t start = after > oldest ? after : oldest;
    if (start >= have) return String();
    uint32_t n = have - start;
    if (n > 8192) { start = have - 8192; n = 8192; }  // limit jednej paczki
    String s;
    s.reserve(n);
    for (uint32_t i = 0; i < n; i++) {
        s += logRing[(start + i) & (LOG_RING_SIZE - 1)];
    }
    return s;
}

/* Przechwytuje wyjscie komendy do: Serial + bufor logow + opcjonalny String
 * (odpowiedz HTTP). Dzieki temu okno logow w WWW widzi postep na zywo. */
class LogPrint : public Print {
    String *cap;
    size_t capMax;
public:
    LogPrint(String *c, size_t max) : cap(c), capMax(max) {}
    size_t write(uint8_t c) override {
        Serial.write(c);
        webLogWrite(&c, 1);
        if (cap && cap->length() < capMax) *cap += (char)c;
        return 1;
    }
    size_t write(const uint8_t *p, size_t n) override {
        Serial.write(p, n);
        webLogWrite(p, n);
        if (cap) {
            size_t room = capMax - cap->length();
            if (n > room) n = room;
            if (n) cap->concat((const char *)p, n);
        }
        return n;
    }
};

/* --- stan odbioru pliku na karte SD (streaming, bez trzymania w RAM) -------- */
static File sdUpFile;
static bool sdUpOpen = false;
static bool sdUpError = false;
static uint32_t sdUpWritten = 0;
static String sdUpTarget;

static String sanitizeName(const String &name) {
    const char *p = name.c_str();
    const char *slash = strrchr(p, '/');
    if (!slash) slash = strrchr(p, '\\');
    if (slash) p = slash + 1;
    String r;
    for (; *p && r.length() < 48; p++) {
        char c = *p;
        if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
            (c >= 'a' && c <= 'z') || c == '.' || c == '-' || c == '_') {
            r += c;
        }
    }
    if (!r.length()) r = "upload.bin";
    return r;
}

static void jsonEscape(String &j, const String &s) {
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '"':  j += "\\\""; break;
            case '\\': j += "\\\\"; break;
            case '\n': j += "\\n"; break;
            case '\r': j += "\\r"; break;
            case '\t': j += "\\t"; break;
            default:
                if ((unsigned char)c < 0x20) {
                    char buf[7];
                    snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
                    j += buf;
                } else {
                    j += c;
                }
        }
    }
}

/* ------------------------------------------------------------------------- */
/* UART terminal (Serial1 -> cel: TX=17, RX=18) — interaktywny terminal WWW. */
#define UART_RING_SIZE 16384u
static uint8_t  uartRing[UART_RING_SIZE];
static uint32_t uartSeq = 0;         // liczba bajtów odebranych od celu
static bool     uartActive = false;
static uint32_t uartBaud = 115200;

static bool     uartLogOn = false;
static String   uartLogPath = "/uart.log";
static uint8_t  uartLogBuf[512];
static uint32_t uartLogLen = 0;
static uint32_t uartLogLast = 0;

static void uartLogFlush() {
    if (!uartLogOn || uartLogLen == 0) return;
    if (sdCardBegin()) sdCardWrite(uartLogPath.c_str(), uartLogBuf, uartLogLen, true);
    uartLogLen = 0;
    uartLogLast = millis();
}

static void uartPoll() {
    if (!uartActive) return;
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        uartRing[uartSeq & (UART_RING_SIZE - 1)] = b;
        uartSeq++;
        if (uartLogOn) {
            uartLogBuf[uartLogLen++] = b;
            if (uartLogLen >= sizeof(uartLogBuf)) uartLogFlush();
        }
    }
    if (uartLogOn && uartLogLen && (millis() - uartLogLast) > 500) uartLogFlush();
}

static void uartStart(uint32_t baud) {
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartBaud = baud;
    uartSeq = 0;
    uartActive = true;
}

static void uartStop() {
    uartLogFlush();
    uartActive = false;
    Serial1.end();
}

/* Auto-baud: mierzy najkrotszy impuls LOW na linii RX (bit startu / bitu 0)
 * i dopasowuje do standardowych predkosci. Wymaga, zeby cel wysylal dane. */
static uint32_t uartDetectBaud(uint32_t timeoutMs) {
    static const uint32_t stdBaud[] = {
        300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400,
        57600, 74880, 115200, 230400, 250000, 460800, 921600, 1000000,
        1500000, 2000000, 3000000
    };
    const size_t stdN = sizeof(stdBaud) / sizeof(stdBaud[0]);

    pinMode(UART_RX_PIN, INPUT_PULLUP);
    uint32_t t0 = millis();
    while (digitalRead(UART_RX_PIN) == LOW && (millis() - t0) < timeoutMs) { /* idle */ }

    uint32_t shortest = 0xFFFFFFFF;
    uint32_t lastEdge = micros();
    bool prev = digitalRead(UART_RX_PIN);
    while ((millis() - t0) < timeoutMs) {
        bool cur = digitalRead(UART_RX_PIN);
        if (cur != prev) {
            uint32_t now = micros();
            if (prev == LOW) {                 /* zakonczyl sie impuls LOW */
                uint32_t w = now - lastEdge;
                if (w > 0 && w < shortest) shortest = w;
            }
            lastEdge = now;
            prev = cur;
        }
    }
    if (shortest == 0xFFFFFFFF) return 0;

    uint32_t raw = 1000000u / shortest;        /* przyblizony baud */
    uint32_t best = stdBaud[0];
    uint32_t bestDiff = raw > best ? raw - best : best - raw;
    for (size_t i = 1; i < stdN; i++) {
        uint32_t d = raw > stdBaud[i] ? raw - stdBaud[i] : stdBaud[i] - raw;
        if (d < bestDiff) { bestDiff = d; best = stdBaud[i]; }
    }
    return best;
}

static String uartSince(uint32_t after) {
    uint32_t have = uartSeq;
    uint32_t oldest = have > UART_RING_SIZE ? have - UART_RING_SIZE : 0;
    uint32_t start = after > oldest ? after : oldest;
    if (start >= have) return String();
    uint32_t n = have - start;
    if (n > 4096) { start = have - 4096; n = 4096; }
    String s;
    s.reserve(n);
    for (uint32_t i = 0; i < n; i++) s += (char)uartRing[(start + i) & (UART_RING_SIZE - 1)];
    return s;
}

/* ------------------------------------------------------------------------- */
/* RS485 (Serial2: TX=33/DI, RX=34/RO, DE=35) — półdupleks. */
#define RS485_RING_SIZE 16384u
static uint8_t  rsRing[RS485_RING_SIZE];
static uint32_t rsSeq = 0;
static bool     rsActive = false;
static uint32_t rsBaud = 9600;

static void rsPoll() {
    if (!rsActive) return;
    while (Serial2.available()) {
        rsRing[rsSeq & (RS485_RING_SIZE - 1)] = (uint8_t)Serial2.read();
        rsSeq++;
    }
}

static void rsStart(uint32_t baud) {
    Serial2.end();
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);   // odbiór
    Serial2.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    rsBaud = baud;
    rsSeq = 0;
    rsActive = true;
}

static void rsStop() {
    rsActive = false;
    Serial2.end();
    digitalWrite(RS485_DE_PIN, LOW);
}

static String rsSince(uint32_t after) {
    uint32_t have = rsSeq;
    uint32_t oldest = have > RS485_RING_SIZE ? have - RS485_RING_SIZE : 0;
    uint32_t start = after > oldest ? after : oldest;
    if (start >= have) return String();
    uint32_t n = have - start;
    if (n > 4096) { start = have - 4096; n = 4096; }
    String s;
    s.reserve(n);
    for (uint32_t i = 0; i < n; i++) s += (char)rsRing[(start + i) & (RS485_RING_SIZE - 1)];
    return s;
}

static void rsSend(const uint8_t *p, size_t n) {
    if (!rsActive || !n) return;
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(25);
    Serial2.write(p, n);
    Serial2.flush();
    delayMicroseconds(25);
    digitalWrite(RS485_DE_PIN, LOW);
}

/* ------------------------------------------------------------------------- */
/* 1-Wire (OneWire na ONE_WIRE_PIN) — skan szyny + odczyt DS18B20. */
static OneWire owBus(ONE_WIRE_PIN);

static const char *owFamilyName(uint8_t fam) {
    switch (fam) {
        case 0x28: return "DS18B20 (temp.)";
        case 0x22: return "DS1822 (temp.)";
        case 0x10: return "DS18S20 (temp.)";
        case 0x3B: return "DS1825 (temp.)";
        case 0x01: return "DS1990 (iButton)";
        default:   return "nieznany";
    }
}

static String owRomHex(const uint8_t *addr) {
    String s;
    char b[3];
    for (int i = 0; i < 8; i++) {   // ROM: family (addr[0]) jako pierwszy
        snprintf(b, sizeof(b), "%02X", addr[i]);
        s += b;
    }
    return s;
}

/* Odczyt temperatury DS18B20 wg adresu ROM (nullptr = pierwszy na szynie). */
static bool owReadDs18b20(const uint8_t *rom, float &c) {
    uint8_t addr[8];
    if (rom) {
        memcpy(addr, rom, 8);
    } else {
        owBus.reset_search();
        if (!owBus.search(addr)) return false;
        if (addr[0] != 0x28 && addr[0] != 0x22 && addr[0] != 0x10) return false;
    }
    owBus.reset();
    owBus.select(addr);
    owBus.write(0x44, 1);            // start konwersji (zasilanie pasożytnicze)
    uint32_t t0 = millis();
    while (millis() - t0 < 800) {
        if (owBus.read_bit()) break; // konwersja skończona
        delay(10);
    }
    owBus.reset();
    owBus.select(addr);
    owBus.write(0xBE);               // scratchpad
    uint8_t d[9];
    for (int i = 0; i < 9; i++) d[i] = owBus.read();
    if (OneWire::crc8(d, 8) != d[8]) return false;
    int16_t raw = (d[1] << 8) | d[0];
    c = raw / 16.0f;
    return true;
}

/* ------------------------------------------------------------------------- */
/* I²C (Wire na I2C_SDA/I2C_SCL) — skan magistrali + odczyt rejestru. */
static bool i2cReady = false;

static void i2cBegin() {
    if (i2cReady) return;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    i2cReady = true;
}

/* ------------------------------------------------------------------------- */
/* GPIO — wolne piny do ręcznego sterowania. */
static const int gpioPins[] = {1, 2, 7, 36, 37, 38, 39, 40, 47, 48};
#define GPIO_NUM ((int)(sizeof(gpioPins) / sizeof(gpioPins[0])))

static bool gpioInList(int pin) {
    for (int i = 0; i < GPIO_NUM; i++) if (gpioPins[i] == pin) return true;
    return false;
}

/* ------------------------------------------------------------------------- */
/* ESP bridge (esptool przez TCP) — flashowanie obcych ESP8266/ESP32. */
static WiFiServer espServer(ESP_BRIDGE_PORT);
static WiFiClient espClient;
static bool espBridged = false;

static void espEnterBoot() {
    pinMode(ESP_IO0_PIN, OUTPUT);
    pinMode(ESP_EN_PIN, OUTPUT);
    digitalWrite(ESP_IO0_PIN, LOW);   // tryb download
    delay(20);
    digitalWrite(ESP_EN_PIN, LOW);    // wyłącz układ
    delay(60);
    digitalWrite(ESP_EN_PIN, HIGH);   // reset -> boot w trybie download
    delay(280);
    digitalWrite(ESP_IO0_PIN, HIGH);  // zwolnij IO0 (bootloader już zatrzaśnięty)
}

static void espBridgeHandle() {
    if (espBridged) {
        if (!espClient.connected()) {
            espBridged = false;
            espClient.stop();
            uartStop();
            g_out->print("ESP bridge: klient rozłączony.\r\n");
            return;
        }
        while (espClient.available()) {
            uint8_t b = (uint8_t)espClient.read();
            Serial1.write(b);
        }
        while (Serial1.available()) {
            uint8_t b = (uint8_t)Serial1.read();
            espClient.write(b);
            webLogWrite(&b, 1);       // postęp w oknie logów WWW
        }
        return;
    }

    WiFiClient c = espServer.accept();
    if (!c) return;

    espClient = c;
    g_out->print("ESP bridge: klient podłączony, wciskam cel w tryb bootloadera...\r\n");
    espEnterBoot();
    uartStart(115200);                // esptool: ESP32 i ESP8266 sync na 115200
    espBridged = true;
}

/* ------------------------------------------------------------------------- */
static void onUpload() {
    HTTPUpload &upload = server.upload();

    if (upload.name == "firmware") {  // obraz .bin do RAM (SWS / SPI flash)
        if (upload.status == UPLOAD_FILE_START) {
            upName = upload.filename;
            upLen = 0;
            upOverflow = false;
            free(upBuf);
            upBuf = (uint8_t *)ps_malloc(WEB_BUF_SIZE);  // PSRAM (N8R2 = 2 MB)
            if (!upBuf) upOverflow = true;
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (!upBuf || upLen + upload.currentSize > WEB_BUF_SIZE) {
                upOverflow = true;
            } else {
                memcpy(upBuf + upLen, upload.buf, upload.currentSize);
                upLen += upload.currentSize;
            }
        }
        return;
    }

    if (upload.name == "sdfile") {  // plik z PC -> karta SD (streaming)
        if (upload.status == UPLOAD_FILE_START) {
            sdUpOpen = false;
            sdUpError = false;
            sdUpWritten = 0;
            sdUpTarget = "";
            if (!sdCardBegin()) { sdUpError = true; return; }
            String dir = server.arg("path");
            if (!dir.length()) dir = "/";
            if (!dir.endsWith("/")) dir += "/";
            sdUpTarget = dir + sanitizeName(upload.filename);
            sdUpFile = SD.open(sdUpTarget, FILE_WRITE);
            if (!sdUpFile) {
                sdUpError = true;
            } else {
                sdUpOpen = true;
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (sdUpOpen && upload.currentSize) {
                if (sdUpFile.write((const uint8_t *)upload.buf, upload.currentSize) !=
                    upload.currentSize) {
                    sdUpError = true;
                } else {
                    sdUpWritten += upload.currentSize;
                }
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (sdUpOpen) { sdUpFile.close(); sdUpOpen = false; }
        }
        return;
    }
}

/* --- WiFi STA (zapis konfiguracji w Preferences) ----------------------------- */
static Preferences wifiPrefs;
static String staSsid = "";
static String staPass = "";

static void wifiLoad() {
    if (!wifiPrefs.begin("swsprog", true)) return;
    staSsid = wifiPrefs.getString("ssid", "");
    staPass = wifiPrefs.getString("pass", "");
    wifiPrefs.end();
}

static void wifiSave(const String &ssid, const String &pass) {
    staSsid = ssid;
    staPass = pass;
    if (wifiPrefs.begin("swsprog", false)) {
        if (ssid.length()) {
            wifiPrefs.putString("ssid", ssid);
            wifiPrefs.putString("pass", pass);
        } else {
            wifiPrefs.clear();
        }
        wifiPrefs.end();
    }
}

/* Publiczny dostęp do konfiguracji STA (dla konsoli — fallback, gdy nie można
 * wejść na AP przez przeglądarkę). */
String wifiStaGetSsid() { return staSsid; }
bool   wifiStaConnected() { return WiFi.status() == WL_CONNECTED; }
String wifiStaGetIp() { return WiFi.localIP().toString(); }

void wifiJoin(const String &ssid, const String &pass) {
    wifiSave(ssid, pass);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD);
    WiFi.begin(ssid.c_str(), pass.c_str());
}

void wifiForget() {
    wifiSave("", "");
    WiFi.disconnect(true);
}

/* Wyniki ostatniego skanu — kopiowane do zwykłych String/int, bo po zmianie
 * trybu Wi-Fi (AP_STA -> STA -> AP_STA) bufor sterownika bywa czyszczony. */
#define WIFI_SCAN_MAX 48
#define WIFI_SCAN_STALE_MS    20000UL  /* po tym czasie wynik skanu uznajemy za nieaktualny */
#define WIFI_SCAN_INTERVAL_MS 30000UL  /* okres skanu w tle */
static String wifiScanSsid[WIFI_SCAN_MAX];
static int    wifiScanRssi[WIFI_SCAN_MAX];
static bool   wifiScanOpen[WIFI_SCAN_MAX];
static int    wifiScanCount = 0;
static uint32_t wifiScanStamp = 0;  /* millis() ostatniego udanego skanu */
static uint8_t  wifiScanPath  = 0;  /* 0 = skan przy podniesionym AP, 1 = fallback STA */

static int wifiScanCollect(int n) {
    if (n < 0) n = 0;
    if (n > WIFI_SCAN_MAX) n = WIFI_SCAN_MAX;
    for (int i = 0; i < n; i++) {
        wifiScanSsid[i] = WiFi.SSID(i);
        wifiScanRssi[i] = WiFi.RSSI(i);
        wifiScanOpen[i] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN);
    }
    wifiScanCount = n;
    WiFi.scanDelete();
    if (n > 0) wifiScanStamp = millis();
    return n;
}

/* Skan sieci Wi-Fi. Najpierw próbujemy skanu BEZ zmiany trybu — AP zostaje
 * podniesione, więc telefon podłączony do AP nie traci połączenia. Dopiero
 * gdy radio zwraca 0 (znany problem ESP32-S3 przy aktywnym softAP), schodzimy
 * do trybu STA, skanujemy i przywracamy AP_STA + softAP + ewentualny STA. */
int wifiScanNow() {
    wifiScanPath = 0;

    WiFi.scanDelete();
    delay(80);

    int n = 0;
    for (int attempt = 0; attempt < 3 && n <= 0; attempt++) {
        n = WiFi.scanNetworks();
        if (n <= 0) delay(150);
    }

    if (n > 0) {
        return wifiScanCollect(n);
    }

    /* Fallback: skan w trybie STA (na chwilę zdejmuje softAP). */
    wifiScanPath = 1;
    const String prevSsid = staSsid;
    const String prevPass = staPass;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(200);

    n = wifiScanCollect(WiFi.scanNetworks());

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD);
    if (prevSsid.length()) {
        WiFi.begin(prevSsid.c_str(), prevPass.c_str());
    }
    return n;
}

uint8_t wifiScanPathGet() { return wifiScanPath; }

bool wifiScanGet(int i, String &ssid, int &rssi, bool &openNet) {
    if (i < 0 || i >= wifiScanCount) return false;
    ssid = wifiScanSsid[i];
    rssi = wifiScanRssi[i];
    openNet = wifiScanOpen[i];
    return true;
}

/* ------------------------------------------------------------------------- */
static uint32_t parseAddr() {
    String a = server.arg("addr");
    if (!a.length()) return 0;
    return (uint32_t)strtoul(a.c_str(), nullptr, 0);
}

static bool haveConsent() {
    return server.arg("consent") == "1";
}

static void sendCaptured(int code, const String &cap) {
    server.send(code, "text/plain; charset=utf-8", cap);
}

/* Odczyt zwrotny i porownanie z wzorcem. Zwraca 0 = OK, -1 = blad odczytu. */
static int fwVerify(uint32_t addr, const uint8_t *data, uint32_t len,
                    uint32_t *firstBad, uint32_t *badCount) {
    uint8_t tmp[SWS_SWS_BURST];
    *firstBad = 0xFFFFFFFF;
    *badCount = 0;
    uint32_t off = 0;
    while (off < len) {
        uint32_t c = len - off < sizeof(tmp) ? len - off : (uint32_t)sizeof(tmp);
        if (!swsFlashRead(addr + off, tmp, c)) return -1;
        for (uint32_t i = 0; i < c; i++) {
            if (tmp[i] != data[off + i]) {
                if (*badCount == 0) *firstBad = off + i;
                (*badCount)++;
            }
        }
        off += c;
    }
    return 0;
}

/* Kasowanie sektorow + zapis stronami + weryfikacja. Zwraca 0 = OK. */
static int fwWrite(uint32_t addr, const uint8_t *data, uint32_t len,
                   uint32_t *firstBad, uint32_t *badCount) {
    uint32_t s = addr & ~(uint32_t)(SWS_FLASH_SECTOR - 1);
    uint32_t e = (addr + len + SWS_FLASH_SECTOR - 1) & ~(uint32_t)(SWS_FLASH_SECTOR - 1);
    for (uint32_t a = s; a < e; a += SWS_FLASH_SECTOR) {
        if (!swsFlashEraseSector(a)) return -1;
    }

    uint32_t off = 0;
    while (off < len) {
        uint32_t n = len - off < SWS_FLASH_PAGE ? len - off : (uint32_t)SWS_FLASH_PAGE;
        bool ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            ok = swsFlashWriteRange(addr + off, data + off, n, false);
            if (ok) {
                uint8_t chk[SWS_FLASH_PAGE];
                ok = swsFlashRead(addr + off, chk, n) && memcmp(chk, data + off, n) == 0;
            }
        }
        if (!ok) return -1;
        off += n;
    }

    return fwVerify(addr, data, len, firstBad, badCount);
}

/* ------------------------------------------------------------------------- */
static void handleRoot() {
    const char *html = R"webhtml(<!doctype html>
<html lang="pl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SWS Programmer</title>
<style>
 *{box-sizing:border-box}
 body{font-family:system-ui,Segoe UI,Arial,sans-serif;margin:0;padding:0;background:#111;color:#e8e8e8}
 .app{display:flex;min-height:100vh}
 .sidebar{width:190px;background:#151515;border-right:1px solid #2e2e2e;padding:12px;flex-shrink:0}
 .sidebar h1{font-size:15px;margin:0 0 12px;color:#7fd1ff}
 .sidebar button{display:block;width:100%;text-align:left;background:#1b1b1b;color:#cfcfcf;border:1px solid #2e2e2e;border-radius:6px;padding:9px 12px;font-size:13px;cursor:pointer;margin:4px 0}
 .sidebar button:hover{background:#242424}
 .sidebar button.active{background:#2563eb;color:#fff;border-color:#2563eb}
 .main{flex:1;display:flex;flex-direction:column;min-width:0}
 .statusbar{font-size:12px;color:#9fc9ff;padding:10px 16px;border-bottom:1px solid #2e2e2e;background:#151515}
 .content{flex:1;padding:16px;overflow:auto}
 .panel{display:none}
 .panel.active{display:block}
 .panel{background:#1b1b1b;border:1px solid #2e2e2e;border-radius:8px;padding:14px;margin-bottom:14px}
 .m{color:#8a8a8a;font-size:12px;margin-bottom:10px}
 h2{font-size:14px;margin:0 0 10px;color:#7fd1ff}
 button{background:#2563eb;color:#fff;border:0;border-radius:6px;padding:8px 12px;font-size:13px;cursor:pointer;margin:2px}
 button:disabled{background:#3a3a3a;color:#777;cursor:not-allowed}
 button.warn{background:#b45309} button.danger{background:#b91c1c} button.green{background:#15803d}
 input[type=text],input[type=password],input[type=file]{width:100%;box-sizing:border-box;padding:8px;margin:6px 0;background:#111;color:#e8e8e8;border:1px solid #2e2e2e;border-radius:6px}
 input[type=checkbox]{margin:8px 4px}
 table.pins{width:100%;border-collapse:collapse;font-size:12px}
 table.pins th,table.pins td{border:1px solid #2e2e2e;padding:5px 8px;text-align:left}
 table.pins th{background:#222;color:#9fc9ff}
 .logwrap{padding:12px 16px;border-top:1px solid #2e2e2e;background:#151515}
 .logbar{display:flex;gap:8px;margin-bottom:8px;align-items:center}
 #log{background:#000;border:1px solid #2e2e2e;border-radius:6px;padding:10px;min-height:160px;max-height:260px;overflow:auto;font:12px/1.45 Consolas,monospace;white-space:pre-wrap;word-break:break-all;margin:0}
 .net{border-bottom:1px solid #2e2e2e;padding:4px 0;display:flex;justify-content:space-between;align-items:center;gap:8px}
 .term{background:#000;border:1px solid #2e2e2e;border-radius:6px;padding:10px;min-height:120px;max-height:220px;overflow:auto;font:12px/1.45 Consolas,monospace;white-space:pre-wrap;word-break:break-all;margin:8px 0 0}
 .row{display:flex;gap:8px;flex-wrap:wrap}
 @media(max-width:640px){.app{flex-direction:column}.sidebar{width:100%;display:flex;flex-wrap:wrap;gap:4px}.sidebar h1{width:100%;margin:0}.sidebar button{width:auto;margin:2px}}
</style></head><body>
<div class="app">
<div class="sidebar">
<h1>SWS Programmer</h1>
<button id="nav-pins" class="active" onclick="showTab('pins')">📌 Piny</button>
<button id="nav-tlsr" onclick="showTab('tlsr')">🔌 TLSR825x</button>
<button id="nav-spi" onclick="showTab('spi')">💾 SPI Flash</button>
<button id="nav-uart" onclick="showTab('uart')">📟 UART</button>
<button id="nav-rs485" onclick="showTab('rs485')">🔁 RS485</button>
<button id="nav-ow" onclick="showTab('ow')">🌡️ 1-Wire</button>
<button id="nav-i2c" onclick="showTab('i2c')">🔗 I²C</button>
<button id="nav-gpio" onclick="showTab('gpio')">⚡ GPIO</button>
<button id="nav-esp" onclick="showTab('esp')">📡 ESP (esptool)</button>
<button id="nav-sd" onclick="showTab('sd')">💳 Karta SD</button>
<button id="nav-diag" onclick="showTab('diag')">🩺 Diagnostyka</button>
<button id="nav-wifi" onclick="showTab('wifi')">📶 WiFi</button>
</div>
<div class="main">
<div id="statusbar" class="statusbar">Ładowanie statusu…</div>
<div class="content">

<section class="panel active" id="panel-pins">
<h2>Piny programatora (legenda)</h2>
<table class="pins">
<tr><th>Funkcja</th><th>ESP32-S3</th><th>Uwagi</th></tr>
<tr><td>SWS (dane programowania Telink)</td><td>GPIO @SWS@</td><td>przez 470 Ω do P2 pin 3</td></tr>
<tr><td>RST (reset celu)</td><td>GPIO @RST@</td><td>przez 470 Ω do P2 pin 4</td></tr>
<tr><td>UART celu TX</td><td>GPIO @UTX@</td><td>do RX urządzenia (3,3 V)</td></tr>
<tr><td>UART celu RX</td><td>GPIO @URX@</td><td>do TX urządzenia (3,3 V)</td></tr>
<tr><td>RS485 TX (DI)</td><td>GPIO @RTX@</td><td rowspan="3">przez transceiver MAX485/SN75176</td></tr>
<tr><td>RS485 RX (RO)</td><td>GPIO @RRX@</td></tr>
<tr><td>RS485 DE/RE (kierunek)</td><td>GPIO @RDE@</td></tr>
<tr><td>SPI Flash SCK</td><td>GPIO @SCK@</td><td rowspan="4">kość 25xx / klips SOIC-8</td></tr>
<tr><td>SPI Flash MISO</td><td>GPIO @SMI@</td></tr>
<tr><td>SPI Flash MOSI</td><td>GPIO @SMO@</td></tr>
<tr><td>SPI Flash CS</td><td>GPIO @SCS@</td></tr>
<tr><td>SD SCK</td><td>GPIO @DSCK@</td><td rowspan="4">moduł Catalex microSD (HSPI)</td></tr>
<tr><td>SD MISO</td><td>GPIO @DMI@</td></tr>
<tr><td>SD MOSI</td><td>GPIO @DMO@</td></tr>
<tr><td>SD CS</td><td>GPIO @DCS@</td></tr>
<tr><td>1-Wire (DS18B20 itp.)</td><td>GPIO @OW@</td><td>pull-up 4,7 kΩ do 3,3 V</td></tr>
<tr><td>I²C SDA</td><td>GPIO @ISDA@</td><td rowspan="2">pull-up 4,7 kΩ do 3,3 V</td></tr>
<tr><td>I²C SCL</td><td>GPIO @ISCL@</td></tr>
<tr><td>ESP cel — IO0 (boot)</td><td>GPIO @EIO0@</td><td rowspan="2">do flashowania obcego ESP (esptool)</td></tr>
<tr><td>ESP cel — EN (reset)</td><td>GPIO @EEN@</td></tr>
<tr><td>ESP bridge TCP</td><td colspan="2">port @EBR@ · esptool.py --port socket://IP:@EBR@</td></tr>
</table>
</section>

<section class="panel" id="panel-tlsr">
<h2>TLSR825x (SWS) — detekcja i firmware</h2>
<div class="row">
<button onclick="run('CHIP')">Wykryj układ (CHIP)</button>
<button onclick="run('CHIPID')">Detekcja chipu (ID)</button>
<button onclick="run('SWSTEST')">Test SWS</button>
<button onclick="run('STAT')">Status pamięci</button>
</div>
<div class="m">Rejestry analogowe (TLSR8258) — odczyt/zapis, np. kalibracja ADC:</div>
<div class="row">
<label style="flex:1">Rejestr A (hex):<input type="text" id="areg" value="0"></label>
<label style="flex:1">Wartość (hex):<input type="text" id="aval" value="0"></label>
</div>
<div class="row">
<button onclick="aRead()">Odczyt A</button>
<button class="warn" onclick="aWrite()">Zapis A</button>
<button onclick="aDump()">Zrzut 0x00–0xFF</button>
</div>
<pre id="anaout" class="m mono"></pre>
<input type="file" id="fw">
<label>Adres (hex, domyślnie 0):</label>
<input type="text" id="addr" value="0">
<label><input type="checkbox" id="consent" onchange="syncConsent()">
Potwierdzam — ZAPISAĆ do pamięci układu (kasowanie + zapis)</label>
<div class="row">
<button onclick="verify()">Verify (porównanie)</button>
<button id="btnFlash" class="danger" onclick="flash()" disabled>Flash (kasowanie + zapis + reset)</button>
</div>
<div class="m">Limit przez WWW: @MAXFW@ B.</div>
</section>

<section class="panel" id="panel-spi">
<h2>SPI Flash (SOIC-8)</h2>
<div class="m">SCK=@SCK@ MISO=@SMI@ MOSI=@SMO@ CS=@SCS@ · SOIC-8: 6=CLK 2=DO 5=DI 1=CS 8=VCC 4=GND · limit WWW @SPIMAX@ B</div>
<div class="row">
<button onclick="run('SPI')">Wykryj kość</button>
<button onclick="spiBlank()">Blank check (cała kość)</button>
</div>
<div class="row">
<label style="flex:1">Adres (hex):<input type="text" id="spiadr" value="0"></label>
<label style="flex:1">Rozmiar (hex):<input type="text" id="spilen" value="1000"></label>
</div>
<div class="row">
<button onclick="spiRead()">Odczyt → pobierz .bin</button>
<button class="warn" onclick="spiErase()">Kasuj sektory</button>
<button class="danger" onclick="spiEraseChip()">Kasuj całą kość</button>
</div>
<input type="file" id="spifw">
<label><input type="checkbox" id="spiconsent" onchange="spiSync()">
Potwierdzam — ZAPISAĆ do kości SPI</label>
<div class="row">
<button onclick="spiVerify()">Verify SPI</button>
<button id="btnSpiFlash" class="danger" onclick="spiFlash()" disabled>Flash SPI</button>
</div>
</section>

<section class="panel" id="panel-uart">
<h2>UART (terminal)</h2>
<div class="m">TX=@UTX@ (→ RX celu) · RX=@URX@ (← TX celu) · 3,3 V · wspólny GND</div>
<div class="row">
<label>Baud:<input type="number" id="uartbaud" value="115200" min="300" max="3000000"></label>
<button class="green" onclick="uartStart()">Start</button>
<button class="warn" onclick="uartStop()">Stop</button>
<button onclick="uartClear()">Wyczyść</button>
</div>
<div class="row">
<button onclick="uartAutobaud()">Auto-baud</button>
<button onclick="uartSdLog(1)">Log RX → SD</button>
<button class="warn" onclick="uartSdLog(0)">Stop log</button>
<label style="flex:1">Plik logu:<input type="text" id="uartlogpath" value="/uart.log"></label>
</div>
<div class="row">
<input type="text" id="uarttx" placeholder="tekst do wysłania (Enter = wyślij)" style="flex:1">
<button onclick="uartSendTxt()">Wyślij tekst</button>
</div>
<div class="row">
<input type="text" id="uarthex" placeholder="HEX np. 414243" style="flex:1">
<button onclick="uartSendHex()">Wyślij HEX</button>
</div>
<pre id="uartrx" class="term">— UART nieaktywny —</pre>
</section>

<section class="panel" id="panel-rs485">
<h2>RS485 (półdupleks)</h2>
<div class="m">TX/DI=@RTX@ · RX/RO=@RRX@ · DE/RE=@RDE@ · przez transceiver MAX485/SN75176 · A/B do magistrali</div>
<div class="row">
<label>Baud:<input type="number" id="rs485baud" value="9600" min="300" max="3000000"></label>
<button class="green" onclick="rs485Start()">Start</button>
<button class="warn" onclick="rs485Stop()">Stop</button>
<button onclick="rs485Clear()">Wyczyść</button>
</div>
<div class="row">
<input type="text" id="rs485tx" placeholder="tekst do wysłania" style="flex:1">
<button onclick="rs485SendTxt()">Wyślij tekst</button>
</div>
<div class="row">
<input type="text" id="rs485hex" placeholder="HEX np. 010300000002" style="flex:1">
<button onclick="rs485SendHex()">Wyślij HEX</button>
</div>
<pre id="rs485rx" class="term">— RS485 nieaktywny —</pre>
</section>

<section class="panel" id="panel-ow">
<h2>1-Wire (GPIO @OW@)</h2>
<div class="m">DS18B20 / DS18S20 / DS1822 / iButton · pull-up 4,7 kΩ do 3,3 V</div>
<div class="row">
<button onclick="owScan()">Skanuj szynę</button>
<button onclick="owTemp()">Odczyt temp. (pierwszy czujnik)</button>
</div>
<div id="owlist" class="m"></div>
</section>

<section class="panel" id="panel-i2c">
<h2>I²C (SDA=@ISDA@, SCL=@ISCL@)</h2>
<div class="m">Pull-up 4,7 kΩ do 3,3 V · skan 1..126</div>
<div class="row"><button onclick="i2cScan()">Skanuj magistralę</button></div>
<div id="i2clist" class="m"></div>
<div class="row">
<label>Adres:<input type="text" id="i2caddr" value="0x40"></label>
<label>Rejestr:<input type="text" id="i2creg" value="0"></label>
<label>Len:<input type="number" id="i2clen" value="8" min="1" max="64"></label>
<button onclick="i2cRead()">Odczytaj rejestr</button>
</div>
<div id="i2cres" class="m"></div>
<div class="m">EEPROM 24xx (24C01..24C512) — SDA/SCL, adres zwykle 0x50:</div>
<div class="row">
<label>Dev:<input type="text" id="eedev" value="0x50"></label>
<label>Offset:<input type="text" id="eeoff" value="0"></label>
<label>Len:<input type="number" id="eelen" value="16" min="1" max="256"></label>
<label>Adresacja:
<select id="eeabits"><option value="16" selected>16-bit</option><option value="8">8-bit</option></select></label>
</div>
<div class="row">
<button onclick="eeRead()">Odczyt EEPROM</button>
<label style="flex:1">Zapis HEX:<input type="text" id="eehex" placeholder="np. DEADBEEF"></label>
<button class="warn" onclick="eeWrite()">Zapis EEPROM</button>
</div>
<div id="eeres" class="m"></div>
</section>

<section class="panel" id="panel-gpio">
<h2>GPIO (wolne piny)</h2>
<div class="m">Wolne GPIO: 1, 2, 7, 36, 37, 38, 39, 40, 47, 48. Unikaj 0/3/45/46 (strapping) i używanych.</div>
<div id="gpiopanel" class="m"></div>
<div class="m">PWM (generator sygnału):</div>
<div class="row">
<label>Pin:<input type="number" id="pwmpin" value="1" min="1" max="48"></label>
<label>Hz:<input type="number" id="pwmfreq" value="1000" min="1" max="1000000"></label>
<label>%:<input type="number" id="pwmduty" value="50" min="0" max="100"></label>
<button onclick="pwmStart()">Start PWM</button>
<button class="warn" onclick="pwmStop()">Stop</button>
</div>
</section>

<section class="panel" id="panel-esp">
<h2>ESP (esptool) — flashowanie obcych ESP32/ESP8266</h2>
<div class="m">IO0=@EIO0@ · EN=@EEN@ · UART TX/RX=@UTX@/@URX@ · mostek TCP na porcie @EBR@</div>
<div id="espstatus" class="m">…</div>
<div class="row">
<button onclick="espBoot()">Wejdź w bootloader</button>
<button onclick="espReset()">Reset (normalny boot)</button>
<button onclick="espStatus()">Odśwież</button>
</div>
<div class="m">
Podłącz: IO0 cel → GPIO @EIO0@, EN cel → GPIO @EEN@, TX cel → @URX@, RX cel → @UTX@, GND wspólny.<br>
Na PC: <code>esptool.py --port socket://IP:@EBR@ flash_id</code><br>
Gdy esptool otworzy port, programator sam wciśnie cel w bootloader i zmostkuje TCP↔UART. Do flashowania:
<code>esptool.py --port socket://IP:@EBR@ --baud 460800 write_flash 0x0 firmware.bin</code>
</div>
</section>

<section class="panel" id="panel-diag">
<h2>Diagnostyka</h2>
<div class="row">
<button onclick="run('PINS')">PINS</button>
<button onclick="run('CHIP')">CHIP</button>
<button onclick="run('SWSTEST')">SWSTEST</button>
<button onclick="run('STAT')">STAT</button>
<button onclick="run('JEDEC')">JEDEC</button>
<button onclick="run('SPI')">SPI (detekcja)</button>
<button onclick="run('PROBE')">PROBE (UART)</button>
<button onclick="run('VSCAN')">VSCAN (napięcia)</button>
<button onclick="run('VMEAS')">VMEAS</button>
<button onclick="run('WAVE')">WAVE</button>
</div>
<div class="m">Wyniki poleceń trafiają do okna logów na dole strony.</div>
</section>

<section class="panel" id="panel-sd">
<h2>Karta microSD</h2>
<div id="sdinfo" class="m">…</div>
<div class="row">
<input type="file" id="sdfile">
<button class="green" onclick="sdUpload()">Upload z PC → SD</button>
<button onclick="sdRefresh()">Odśwież listę</button>
<button onclick="run('SDTEST')">Diagnostyka SD</button>
</div>
<div id="sdlist" class="m"></div>
<label><input type="checkbox" id="sdconsent">
Potwierdzam — SFORMATOWAĆ kartę (usuwa wszystkie dane)</label>
<button class="danger" onclick="sdFormat()">Formatuj kartę</button>
</section>

<section class="panel" id="panel-wifi">
<h2>WiFi (dodaj programator do sieci)</h2>
<div id="wifistatus" class="m">…</div>
<div class="row">
<button onclick="wifiScan()">Skanuj sieci</button>
<button onclick="wifiStatus()">Odśwież status</button>
</div>
<div id="wifilist" class="m"></div>
<label>SSID:</label>
<input type="text" id="wifissid" placeholder="nazwa sieci WiFi">
<label>Hasło:</label>
<input type="password" id="wifipass" placeholder="hasło sieci WiFi">
<div class="row">
<button class="green" onclick="wifiSave()">Zapisz i połącz</button>
<button class="warn" onclick="wifiClear()">Usuń zapisane WiFi</button>
</div>
</section>

</div>
<div class="logwrap">
<div class="logbar">
<input type="text" id="cmd" style="flex:1;margin:0" placeholder="np. SPI, SPIREAD 0 100, JEDEC, CHIP, STAT, SDLS">
<button onclick="runCmd()">Wykonaj</button>
<button class="warn" onclick="clearLog()">Wyczyść okno</button>
</div>
<pre id="log">Ładowanie logu…</pre>
</div>
</div>
</div>

<script>
function showTab(id){
  document.querySelectorAll('.panel').forEach(function(p){p.classList.remove('active');});
  document.querySelectorAll('.sidebar button').forEach(function(b){b.classList.remove('active');});
  var panel=document.getElementById('panel-'+id);
  var nav=document.getElementById('nav-'+id);
  if(panel)panel.classList.add('active');
  if(nav)nav.classList.add('active');
  if(id!=='pins'&&id!=='wifi')window.scrollTo(0,0);
}
const log=document.getElementById('log');
let logCursor=0;
function logMsg(m){
  log.textContent+=m;
  log.scrollTop=log.scrollHeight;
}
function clearLog(){log.textContent='';logCursor=0;}
async function pollLog(){
  try{
    const r=await fetch('/api/log?after='+logCursor);
    if(!r.ok)return;
    const j=await r.json();
    if(j.seq>logCursor){
      logCursor=j.seq;
      if(j.text){
        const stick=log.scrollTop+log.clientHeight>=log.scrollHeight-8;
        log.textContent+=j.text;
        if(stick)log.scrollTop=log.scrollHeight;
      }
    }
  }catch(e){}
}
setInterval(pollLog,400);
function syncConsent(){document.getElementById('btnFlash').disabled=!document.getElementById('consent').checked;}
function spiSync(){document.getElementById('btnSpiFlash').disabled=!document.getElementById('spiconsent').checked;}
async function run(c){
  logMsg('> '+c+'\n');
  try{
    const r=await fetch('/api/run',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'c='+encodeURIComponent(c)});
    const t=await r.text();
    if(t && t.length)logMsg(t+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function runCmd(){const v=document.getElementById('cmd').value.trim();if(v)run(v);}
async function upload(path){
  const f=document.getElementById('fw').files[0];
  const a=document.getElementById('addr').value.trim();
  if(!f){logMsg('-ERR wybierz plik .bin\n');return;}
  logMsg('Wysyłam '+f.name+' ('+f.size+' B)...\n');
  const fd=new FormData();
  fd.append('firmware',f);
  fd.append('addr',a||'0');
  if(path==='/api/flash')fd.append('consent',document.getElementById('consent').checked?'1':'0');
  try{
    const r=await fetch(path,{method:'POST',body:fd});
    if(!r.ok)logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function verify(){upload('/api/verify');}
function flash(){upload('/api/flash');}
async function spiRead(){
  const a=document.getElementById('spiadr').value.trim()||'0';
  const l=document.getElementById('spilen').value.trim()||'1000';
  logMsg('Odczyt SPI (0x'+a+', 0x'+l+')...\n');
  try{
    const r=await fetch('/api/spiread?addr='+encodeURIComponent(a)+'&len='+encodeURIComponent(l));
    if(!r.ok){logMsg(await r.text()+'\n');return;}
    const blob=await r.blob();
    const u=URL.createObjectURL(blob);
    const x=document.createElement('a');x.href=u;x.download='spi.bin';document.body.appendChild(x);x.click();x.remove();
    URL.revokeObjectURL(u);
    logMsg('Pobrano '+blob.size+' B.\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function spiBlank(){
  const a=document.getElementById('spiadr').value.trim()||'0';
  logMsg('Blank check SPI (adres 0x'+a+', cała kość)...\n');
  try{
    const r=await fetch('/api/spiblank?addr='+encodeURIComponent(a),{method:'POST'});
    if(!r.ok){logMsg(await r.text()+'\n');}else{logMsg(await r.text()+'\n');}
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function aRead(){
  const a=document.getElementById('areg').value.trim()||'0';
  const out=document.getElementById('anaout');
  out.textContent='Odczyt A['+a+']...';
  try{
    const r=await fetch('/api/areg?a='+encodeURIComponent(a));
    const t=await r.text();
    out.textContent=t;
    logMsg(t+'\n');
  }catch(e){out.textContent='BŁĄD: '+e;}
}
async function aWrite(){
  const a=document.getElementById('areg').value.trim()||'0';
  const v=document.getElementById('aval').value.trim()||'0';
  const out=document.getElementById('anaout');
  out.textContent='Zapis A['+a+']=0x'+v+'...';
  try{
    const r=await fetch('/api/awr?a='+encodeURIComponent(a)+'&v='+encodeURIComponent(v),{method:'POST'});
    const t=await r.text();
    out.textContent=t;
    logMsg(t+'\n');
  }catch(e){out.textContent='BŁĄD: '+e;}
}
async function aDump(){
  const out=document.getElementById('anaout');
  out.textContent='Zrzut rejestrów analogowych...';
  try{
    const r=await fetch('/api/adump');
    const t=await r.text();
    out.textContent=t;
    logMsg(t+'\n');
  }catch(e){out.textContent='BŁĄD: '+e;}
}
async function spiErase(){
  if(!document.getElementById('spiconsent').checked){logMsg('-ERR zaznacz zgodę na kasowanie\n');return;}
  const a=document.getElementById('spiadr').value.trim()||'0';
  const l=document.getElementById('spilen').value.trim()||'1000';
  logMsg('Kasowanie SPI...\n');
  try{
    const r=await fetch('/api/spierase?addr='+encodeURIComponent(a)+'&len='+encodeURIComponent(l)+'&consent=1',{method:'POST'});
    if(!r.ok)logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function spiEraseChip(){
  if(!document.getElementById('spiconsent').checked){logMsg('-ERR zaznacz zgodę na kasowanie\n');return;}
  logMsg('Kasowanie całej kości SPI...\n');
  try{
    const r=await fetch('/api/spierasechip?consent=1',{method:'POST'});
    if(!r.ok)logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function spiUpload(path){
  const f=document.getElementById('spifw').files[0];
  const a=document.getElementById('spiadr').value.trim()||'0';
  if(!f){logMsg('-ERR wybierz plik .bin\n');return;}
  logMsg('Wysyłam '+f.name+' ('+f.size+' B)...\n');
  const fd=new FormData();
  fd.append('firmware',f);
  fd.append('addr',a);
  if(path==='/api/spiwrite')fd.append('consent',document.getElementById('spiconsent').checked?'1':'0');
  try{
    const r=await fetch(path,{method:'POST',body:fd});
    if(!r.ok)logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function spiVerify(){spiUpload('/api/spiverify');}
function spiFlash(){spiUpload('/api/spiwrite');}
function fmt(n){
  if(n>=1073741824)return (n/1073741824).toFixed(2)+' GB';
  if(n>=1048576)return (n/1048576).toFixed(1)+' MB';
  if(n>=1024)return (n/1024).toFixed(0)+' kB';
  return n+' B';
}
async function sdRefresh(){
  try{
    const r=await fetch('/api/sdinfo');
    const j=await r.json();
    document.getElementById('sdinfo').textContent=j.mounted?
      ('Karta '+j.type+' · '+fmt(j.total)+' łącznie · '+fmt(j.used)+' zajęte'):
      'Brak karty SD';
    sdList();
  }catch(e){}
}
async function sdList(){
  try{
    const r=await fetch('/api/sdls');
    const j=await r.json();
    const d=document.getElementById('sdlist');
    if(!j.ok){d.innerHTML='<div class="m">'+j.err+'</div>';return;}
    let h='<table class="pins"><tr><th>Nazwa</th><th>Rozmiar</th><th></th></tr>';
    for(const f of j.files){
      h+='<tr><td>'+(f.d?'📁 ':'')+f.n+'</td><td>'+(f.d?'—':fmt(f.s))+'</td><td>'+
         (f.d?'':'<button data-n="'+f.n.replace(/"/g,'&quot;')+'" onclick="sdDl(this.dataset.n)">Pobierz</button> '+
                 '<button class="danger" data-n="'+f.n.replace(/"/g,'&quot;')+'" onclick="sdDel(this.dataset.n)">Usuń</button>')+'</td></tr>';
    }
    h+='</table>';
    d.innerHTML=h;
  }catch(e){}
}
async function sdUpload(){
  const f=document.getElementById('sdfile').files[0];
  if(!f){logMsg('-ERR wybierz plik\n');return;}
  logMsg('Upload na SD: '+f.name+' ('+f.size+' B)...\n');
  const fd=new FormData();
  fd.append('sdfile',f);
  try{
    const r=await fetch('/api/sdupload',{method:'POST',body:fd});
    if(!r.ok){logMsg(await r.text()+'\n');}else{sdRefresh();}
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function sdDl(n){window.location.href='/api/sddownload?file='+encodeURIComponent(n);}
async function sdDel(n){
  if(!confirm('Usunąć '+n+'?'))return;
  try{
    const r=await fetch('/api/sddelete?file='+encodeURIComponent(n),{method:'POST'});
    if(!r.ok)logMsg(await r.text()+'\n');
    sdRefresh();
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function sdFormat(){
  if(!document.getElementById('sdconsent').checked){logMsg('-ERR zaznacz zgodę na formatowanie\n');return;}
  if(!confirm('Sformatować kartę SD? WSZYSTKIE DANE ZOSTANĄ USUNIĘTE!'))return;
  logMsg('Formatowanie SD...\n');
  try{
    const r=await fetch('/api/sdformat?consent=1',{method:'POST'});
    if(!r.ok){logMsg(await r.text()+'\n');}else{sdRefresh();}
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function wifiScan(){
  logMsg('Pobieram listę sieci...\n');
  document.getElementById('wifilist').innerHTML='<div class="m">Pobieranie…</div>';
  try{
    const r=await fetch('/api/wifiscan');
    const j=await r.json();
    const d=document.getElementById('wifilist');
    if(!j.ok||!j.networks.length){d.innerHTML='<div class="m">Brak sieci — spróbuj ponownie za chwilę</div>';return;}
    let h='';
    for(const n of j.networks){
      h+='<div class="net"><span>'+n.ssid+' ('+n.rssi+' dBm'+(n.auth?' · 🔒':'')+')</span>'+
         '<button data-ssid="'+n.ssid.replace(/"/g,'&quot;')+'" onclick="wifiPick(this.dataset.ssid)">Wybierz</button></div>';
    }
    if(j.age_ms)h+='<div class="m">Zeskanowano '+Math.round(j.age_ms/1000)+' s temu</div>';
    d.innerHTML=h;
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function wifiPick(s){document.getElementById('wifissid').value=s;}
async function wifiStatus(){
  try{
    const r=await fetch('/api/wifistatus');
    const j=await r.json();
    document.getElementById('wifistatus').textContent=
      'AP: '+j.ap+' (hasło 12345678) · STA: '+(j.sta?j.sta:'(brak)')+
      (j.connected?(' · IP: '+j.ip):(j.sta?' · łączenie…':''));
  }catch(e){}
}
async function wifiSave(){
  const s=document.getElementById('wifissid').value.trim();
  const p=document.getElementById('wifipass').value;
  if(!s){logMsg('-ERR podaj SSID\n');return;}
  logMsg('Zapisuję WiFi: '+s+'...\n');
  try{
    const r=await fetch('/api/wifisave?ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p),{method:'POST'});
    if(!r.ok){logMsg(await r.text()+'\n');}else{logMsg(await r.text()+'\n');wifiStatus();}
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function wifiClear(){
  try{
    const r=await fetch('/api/wifisave?ssid=',{method:'POST'});
    logMsg(await r.text()+'\n');wifiStatus();
  }catch(e){}
}
/* ---------- UART ---------- */
let uartCursor=0, uartTimer=null;
function uartEl(){return document.getElementById('uartrx');}
function uartClear(){uartEl().textContent='';uartCursor=0;}
async function uartStart(){
  const b=document.getElementById('uartbaud').value||'115200';
  uartClear();
  try{const r=await fetch('/api/uart?action=start&baud='+encodeURIComponent(b),{method:'POST'});logMsg(await r.text()+'\n');}catch(e){logMsg('BŁĄD: '+e+'\n');}
  startUartPoll();
}
async function uartStop(){
  stopUartPoll();
  try{const r=await fetch('/api/uart?action=stop',{method:'POST'});logMsg(await r.text()+'\n');}catch(e){}
}
function startUartPoll(){
  if(uartTimer)return;
  uartTimer=setInterval(uartPoll,300);
  uartPoll();
}
function stopUartPoll(){if(uartTimer){clearInterval(uartTimer);uartTimer=null;}}
async function uartPoll(){
  try{
    const r=await fetch('/api/uartrx?after='+uartCursor);
    const j=await r.json();
    if(j.seq>uartCursor){
      uartCursor=j.seq;
      if(j.data){
        const el=uartEl();
        if(el.textContent==='— UART nieaktywny —')el.textContent='';
        const stick=el.scrollTop+el.clientHeight>=el.scrollHeight-8;
        el.textContent+=j.data;
        if(stick)el.scrollTop=el.scrollHeight;
      }
    }
    if(!j.active)stopUartPoll();
  }catch(e){}
}
async function uartSend(txt,hex){
  try{
    const r=await fetch('/api/uart/send?txt='+encodeURIComponent(txt||'')+'&hex='+encodeURIComponent(hex||''),{method:'POST'});
    logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function uartSendTxt(){uartSend(document.getElementById('uarttx').value,'');}
function uartSendHex(){uartSend('',document.getElementById('uarthex').value.trim());}
async function uartAutobaud(){
  logMsg('Auto-baud: mierzę bit startu na RX...\n');
  try{
    const r=await fetch('/api/uart/autobaud',{method:'POST'});
    const t=await r.text();
    logMsg(t+'\n');
    const m=t.match(/(\d{3,})/);
    if(m)document.getElementById('uartbaud').value=m[1];
    startUartPoll();
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function uartSdLog(on){
  const p=document.getElementById('uartlogpath').value.trim()||'/uart.log';
  try{
    const r=await fetch('/api/uart/sdlog?en='+(on?'1':'0')+'&path='+encodeURIComponent(p),{method:'POST'});
    logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
/* ---------- RS485 ---------- */
let rs485Cursor=0, rs485Timer=null;
function rs485El(){return document.getElementById('rs485rx');}
function rs485Clear(){rs485El().textContent='';rs485Cursor=0;}
async function rs485Start(){
  const b=document.getElementById('rs485baud').value||'9600';
  rs485Clear();
  try{const r=await fetch('/api/rs485?action=start&baud='+encodeURIComponent(b),{method:'POST'});logMsg(await r.text()+'\n');}catch(e){logMsg('BŁĄD: '+e+'\n');}
  startRs485Poll();
}
async function rs485Stop(){
  stopRs485Poll();
  try{const r=await fetch('/api/rs485?action=stop',{method:'POST'});logMsg(await r.text()+'\n');}catch(e){}
}
function startRs485Poll(){
  if(rs485Timer)return;
  rs485Timer=setInterval(rs485Poll,300);
  rs485Poll();
}
function stopRs485Poll(){if(rs485Timer){clearInterval(rs485Timer);rs485Timer=null;}}
async function rs485Poll(){
  try{
    const r=await fetch('/api/rs485rx?after='+rs485Cursor);
    const j=await r.json();
    if(j.seq>rs485Cursor){
      rs485Cursor=j.seq;
      if(j.data){
        const el=rs485El();
        if(el.textContent==='— RS485 nieaktywny —')el.textContent='';
        const stick=el.scrollTop+el.clientHeight>=el.scrollHeight-8;
        el.textContent+=j.data;
        if(stick)el.scrollTop=el.scrollHeight;
      }
    }
    if(!j.active)stopRs485Poll();
  }catch(e){}
}
async function rs485Send(txt,hex){
  try{
    const r=await fetch('/api/rs485/send?txt='+encodeURIComponent(txt||'')+'&hex='+encodeURIComponent(hex||''),{method:'POST'});
    logMsg(await r.text()+'\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function rs485SendTxt(){rs485Send(document.getElementById('rs485tx').value,'');}
function rs485SendHex(){rs485Send('',document.getElementById('rs485hex').value.trim());}
/* ---------- 1-Wire ---------- */
async function owScan(){
  const d=document.getElementById('owlist');
  d.innerHTML='<div class="m">Skanuję…</div>';
  try{
    const r=await fetch('/api/owscan');
    const j=await r.json();
    if(!j.devices.length){d.innerHTML='<div class="m">Brak urządzeń 1-Wire (sprawdź pull-up 4,7 kΩ)</div>';return;}
    let h='<table class="pins"><tr><th>ROM</th><th>Typ</th><th></th></tr>';
    for(const x of j.devices){
      h+='<tr><td>'+x.rom+'</td><td>'+x.family+'</td><td>'+
         (x.rom.startsWith('28')||x.rom.startsWith('22')||x.rom.startsWith('10')?
          '<button data-rom="'+x.rom+'" onclick="owTempRom(this.dataset.rom)">Odczyt temp.</button>':'')+
         '</td></tr>';
    }
    h+='</table>';
    d.innerHTML=h;
  }catch(e){d.innerHTML='<div class="m">BŁĄD: '+e+'</div>';}
}
async function owTempRom(rom){
  try{
    const r=await fetch('/api/owtemp?rom='+encodeURIComponent(rom||''));
    const j=await r.json();
    if(j.ok)logMsg('1-Wire '+rom+' = '+j.c+' °C ('+j.f+' °F)\n');
    else logMsg('1-Wire: błąd odczytu (CRC/brak czujnika)\n');
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
function owTemp(){owTempRom('');}
/* ---------- I²C ---------- */
async function i2cScan(){
  const d=document.getElementById('i2clist');
  d.innerHTML='<div class="m">Skanuję…</div>';
  try{
    const r=await fetch('/api/i2cscan');
    const j=await r.json();
    if(!j.devices.length){d.innerHTML='<div class="m">Brak urządzeń I²C (sprawdź pull-upy SDA/SCL)</div>';return;}
    let h='<div class="m">Znaleziono:</div><table class="pins"><tr><th>Adres</th><th></th></tr>';
    for(const x of j.devices){
      h+='<tr><td>'+x.addr+' ('+x.dec+')</td><td><button data-a="'+x.addr+'" onclick="i2cPick(this.dataset.a)">Wybierz</button></td></tr>';
    }
    h+='</table>';
    d.innerHTML=h;
  }catch(e){d.innerHTML='<div class="m">BŁĄD: '+e+'</div>';}
}
function i2cPick(a){document.getElementById('i2caddr').value=a;}
async function i2cRead(){
  const a=document.getElementById('i2caddr').value.trim()||'0x40';
  const g=document.getElementById('i2creg').value.trim()||'0';
  const l=document.getElementById('i2clen').value||'8';
  try{
    const r=await fetch('/api/i2cread?addr='+encodeURIComponent(a)+'&reg='+encodeURIComponent(g)+'&len='+encodeURIComponent(l));
    const j=await r.json();
    document.getElementById('i2cres').textContent=j.ok?('Odczyt @'+a+' reg '+g+': '+j.hex):'Błąd odczytu (brak ACK / zły adres)';
  }catch(e){document.getElementById('i2cres').textContent='BŁĄD: '+e;}
}
/* ---------- EEPROM 24xx ---------- */
async function eeRead(){
  const d=document.getElementById('eeres');
  const dev=document.getElementById('eedev').value.trim()||'0x50';
  const off=document.getElementById('eeoff').value.trim()||'0';
  const len=document.getElementById('eelen').value||'16';
  const abits=document.getElementById('eeabits').value||'16';
  d.textContent='Odczyt…';
  try{
    const r=await fetch('/api/eeprom/read?dev='+encodeURIComponent(dev)+'&off='+encodeURIComponent(off)+'&len='+encodeURIComponent(len)+'&abits='+encodeURIComponent(abits));
    const j=await r.json();
    d.textContent=j.ok?('EEPROM @'+dev+' +0x'+off+': '+j.hex):'Błąd odczytu (brak ACK / zły adres)';
  }catch(e){d.textContent='BŁĄD: '+e;}
}
async function eeWrite(){
  const d=document.getElementById('eeres');
  const dev=document.getElementById('eedev').value.trim()||'0x50';
  const off=document.getElementById('eeoff').value.trim()||'0';
  const hex=document.getElementById('eehex').value.trim();
  const abits=document.getElementById('eeabits').value||'16';
  if(!hex){d.textContent='Podaj HEX do zapisu';return;}
  d.textContent='Zapis…';
  try{
    const r=await fetch('/api/eeprom/write?dev='+encodeURIComponent(dev)+'&off='+encodeURIComponent(off)+'&hex='+encodeURIComponent(hex)+'&abits='+encodeURIComponent(abits),{method:'POST'});
    d.textContent=await r.text();
  }catch(e){d.textContent='BŁĄD: '+e;}
}
/* ---------- GPIO ---------- */
async function gpioScan(){
  const d=document.getElementById('gpiopanel');
  try{
    const r=await fetch('/api/gpio/get');
    const j=await r.json();
    let h='<table class="pins"><tr><th>Pin</th><th>Tryb</th><th>Wartość</th><th>Ustaw</th></tr>';
    for(const p of j.pins){
      h+='<tr><td>GPIO '+p.pin+'</td><td>'+
         '<select id="gmode'+p.pin+'">'+
         '<option value="in">Wejście</option>'+
         '<option value="in_pullup">Wejście pull-up</option>'+
         '<option value="in_pulldown">Wejście pull-down</option>'+
         '<option value="out">Wyjście</option></select></td>'+
         '<td id="gval'+p.pin+'">'+(p.val?'HIGH':'LOW')+'</td>'+
         '<td><button onclick="gpioSet('+p.pin+',document.getElementById(\'gmode'+p.pin+'\').value,0)">Ustaw</button> '+
         '<button onclick="gpioToggle('+p.pin+')">Toggle</button></td></tr>';
    }
    h+='</table>';
    d.innerHTML=h;
    gpioRefreshVals();
  }catch(e){d.innerHTML='<div class="m">BŁĄD: '+e+'</div>';}
}
async function gpioRefreshVals(){
  try{
    const r=await fetch('/api/gpio/get');
    const j=await r.json();
    for(const p of j.pins){
      const el=document.getElementById('gval'+p.pin);
      if(el)el.textContent=p.val?'HIGH':'LOW';
    }
  }catch(e){}
}
async function gpioSet(pin,mode,val){
  try{
    const r=await fetch('/api/gpio/set?pin='+pin+'&mode='+mode+'&value='+val,{method:'POST'});
    logMsg(await r.text()+'\n');
    gpioRefreshVals();
  }catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function gpioToggle(pin){
  try{
    const r=await fetch('/api/gpio/get');
    const j=await r.json();
    const p=j.pins.find(function(x){return x.pin===pin;});
    if(p){gpioSet(pin,'out',p.val?0:1);}
  }catch(e){}
}
function pwmStart(){
  const p=document.getElementById('pwmpin').value||'1';
  const f=document.getElementById('pwmfreq').value||'1000';
  const d=document.getElementById('pwmduty').value||'50';
  run('PWM '+p+' '+f+' '+d);
}
function pwmStop(){
  const p=document.getElementById('pwmpin').value||'1';
  run('PWMSTOP '+p);
}
/* ---------- ESP bridge ---------- */
async function espStatus(){
  try{
    const r=await fetch('/api/esp/status');
    const j=await r.json();
    document.getElementById('espstatus').textContent=
      'Mostek: '+(j.bridged?'AKTYWNY (klient podłączony)':'nieaktywny')+
      ' · port '+j.port+' · IO0='+j.io0+' EN='+j.en+' UART='+j.uart;
  }catch(e){}
}
async function espBoot(){
  try{const r=await fetch('/api/esp/boot',{method:'POST'});logMsg(await r.text()+'\n');}catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function espReset(){
  try{const r=await fetch('/api/esp/reset',{method:'POST'});logMsg(await r.text()+'\n');}catch(e){logMsg('BŁĄD: '+e+'\n');}
}
async function refreshStatus(){
  try{
    const r=await fetch('/api/status');
    const j=await r.json();
    document.getElementById('statusbar').innerHTML=
      'AP: <b>'+j.ap+'</b> · '+j.ip+
      ' · STA: '+(j.sta?j.sta:'(brak)')+(j.sta_ip?(' · '+j.sta_ip):'')+
      ' · PSRAM '+(j.psram/1048576).toFixed(1)+' MB'+
      (j.sd?' · SD ✔':' · SD ✖');
  }catch(e){}
}
syncConsent();
spiSync();
pollLog();
sdRefresh();
wifiStatus();
refreshStatus();
gpioScan();
espStatus();
setInterval(refreshStatus,5000);
setInterval(espStatus,2000);
document.getElementById('uarttx').addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();uartSendTxt();}});
document.getElementById('rs485tx').addEventListener('keydown',function(e){if(e.key==='Enter'){e.preventDefault();rs485SendTxt();}});
</script></body></html>)webhtml";

    String h(html);
    h.replace("@SWS@", String(SWS_PIN));
    h.replace("@RST@", String(RST_PIN));
    h.replace("@UTX@", String(UART_TX_PIN));
    h.replace("@URX@", String(UART_RX_PIN));
    h.replace("@RTX@", String(RS485_TX_PIN));
    h.replace("@RRX@", String(RS485_RX_PIN));
    h.replace("@RDE@", String(RS485_DE_PIN));
    h.replace("@SCK@", String(SPI_SCK_PIN));
    h.replace("@SMI@", String(SPI_MISO_PIN));
    h.replace("@SMO@", String(SPI_MOSI_PIN));
    h.replace("@SCS@", String(SPI_CS_PIN));
    h.replace("@DSCK@", String(SD_SCK_PIN));
    h.replace("@DMI@", String(SD_MISO_PIN));
    h.replace("@DMO@", String(SD_MOSI_PIN));
    h.replace("@DCS@", String(SD_CS_PIN));
    h.replace("@OW@", String(ONE_WIRE_PIN));
    h.replace("@ISDA@", String(I2C_SDA_PIN));
    h.replace("@ISCL@", String(I2C_SCL_PIN));
    h.replace("@EIO0@", String(ESP_IO0_PIN));
    h.replace("@EEN@", String(ESP_EN_PIN));
    h.replace("@EBR@", String(ESP_BRIDGE_PORT));
    h.replace("@MAXFW@", String(WEB_MAX_FW));
    h.replace("@SPIMAX@", String(SPI_WEB_MAX));

    server.send(200, "text/html; charset=utf-8", h);
}

static void handleStatus() {
    bool connected = WiFi.status() == WL_CONNECTED;
    bool sd = sdCardMounted();
    char j[448];
    snprintf(j, sizeof(j),
             "{\"ap\":\"%s\",\"ip\":\"%s\",\"sta\":\"%s\",\"sta_ip\":\"%s\","
             "\"uptime_ms\":%lu,\"free_heap\":%u,\"psram\":%u,"
             "\"sws_pin\":%d,\"rst_pin\":%d,\"flash_size\":%u,"
             "\"sd\":%s,\"sd_type\":\"%s\",\"sd_total\":%llu,\"sd_used\":%llu}",
             WEB_AP_SSID, WiFi.softAPIP().toString().c_str(),
             staSsid.c_str(),
             connected ? WiFi.localIP().toString().c_str() : "",
             (unsigned long)millis(), (unsigned)ESP.getFreeHeap(),
             (unsigned)ESP.getPsramSize(),
             SWS_PIN, RST_PIN, (unsigned)SWS_FLASH_SIZE,
             sd ? "true" : "false", sdCardTypeName(),
             (unsigned long long)sdCardTotalBytes(),
             (unsigned long long)sdCardUsedBytes());
    server.send(200, "application/json", j);
}

/* Deklaracja wstepna - definicja nizej (przy obsludze WiFi). */
static bool formDecodeArg(const String &body, const char *name, String &out);

static void handleRun() {
    String cmd = server.arg("c");
    if (!cmd.length()) {
        /* Niektorzy klienci (fetch bez Content-Type) wysylaja body text/plain,
         * przez co arg("c") jest puste, a cala tresc ląduje w arg("plain")
         * jako "c=KOMENDA". Wyciagamy stamtad komende. */
        String body = server.arg("plain");
        if (body.length()) formDecodeArg(body, "c", cmd);
    }
    if (!cmd.length()) {
        server.send(400, "text/plain; charset=utf-8", "-ERR brak komendy (pole: c)");
        return;
    }
    if (cmd.length() > 400) cmd = cmd.substring(0, 400);

    char buf[512];
    cmd.toCharArray(buf, sizeof(buf));

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    dispatch(buf);
    g_out = saved;

    sendCaptured(200, cap);
}

static void handleVerify() {
    if (!upBuf || !upLen || upOverflow) {
        server.send(400, "text/plain; charset=utf-8",
                    upOverflow ? "-ERR plik za duzy (limit WEB_MAX_FW)" : "-ERR brak pliku (pole: firmware)");
        return;
    }
    uint32_t addr = parseAddr();
    if ((uint64_t)addr + upLen > SWS_FLASH_SIZE) {
        server.send(400, "text/plain; charset=utf-8", "-ERR adres+rozmiar poza flash");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    uint8_t div = swsUnitToDiv(swsGetUnitUs());
    if (!swsLinkTest(div, true)) {
        g_out = saved;
        sendCaptured(500, cap);
        return;
    }

    uint32_t firstBad = 0, bad = 0;
    g_out->print("Weryfikacja: ");
    g_out->print(upName);
    g_out->print(" (");
    g_out->print(upLen);
    g_out->print(" B) pod 0x");
    g_out->print(addr, HEX);
    g_out->print("\r\n");

    if (fwVerify(addr, upBuf, upLen, &firstBad, &bad) != 0) {
        g_out->print("-ERR odczyt flash nieudany\r\n");
    } else if (bad == 0) {
        g_out->print("+OK obraz zgodny (");
        g_out->print(upLen);
        g_out->print(" B)\r\n");
    } else {
        g_out->print("-ERR niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print("/");
        g_out->print(upLen);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(firstBad, HEX);
        g_out->print(")\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

static void handleFlash() {
    if (!haveConsent()) {
        server.send(403, "text/plain; charset=utf-8",
                    "-ERR brak zgody na zapis (consent=1)");
        return;
    }
    if (!upBuf || !upLen || upOverflow) {
        server.send(400, "text/plain; charset=utf-8",
                    upOverflow ? "-ERR plik za duzy (limit WEB_MAX_FW)" : "-ERR brak pliku (pole: firmware)");
        return;
    }
    uint32_t addr = parseAddr();
    if ((uint64_t)addr + upLen > SWS_FLASH_SIZE) {
        server.send(400, "text/plain; charset=utf-8", "-ERR adres+rozmiar poza flash");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    uint8_t div = swsUnitToDiv(swsGetUnitUs());
    g_out->print("Aktywacja SWS...\r\n");
    if (!swsLinkTest(div, true)) {
        g_out = saved;
        sendCaptured(500, cap);
        return;
    }

    g_out->print("Kasowanie + zapis + weryfikacja: ");
    g_out->print(upName);
    g_out->print(" (");
    g_out->print(upLen);
    g_out->print(" B) pod 0x");
    g_out->print(addr, HEX);
    g_out->print("\r\n");

    uint32_t firstBad = 0, bad = 0;
    if (fwWrite(addr, upBuf, upLen, &firstBad, &bad) != 0) {
        g_out->print("-ERR zapis nieudany (link SWS?)\r\n");
    } else if (bad == 0) {
        g_out->print("+OK wgrano ");
        g_out->print(upLen);
        g_out->print(" B, weryfikacja zgodna\r\n");
        swsResetTargetPulse(50);
        g_out->print("+OK reset celu\r\n");
    } else {
        g_out->print("-ERR po zapisie niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(firstBad, HEX);
        g_out->print(")\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

/* --- SPI flash (SOIC-8) --------------------------------------------------- */
static uint32_t parseLen(const char *name, uint32_t def) {
    String s = server.arg(name);
    if (!s.length()) return def;
    return (uint32_t)strtoul(s.c_str(), nullptr, 0);
}

static void handleSpiDetect() {
    char buf[8] = "SPI";
    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    dispatch(buf);
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSpiBlank() {
    uint32_t addr = parseAddr();
    uint32_t len = parseLen("len", 0);
    if (len == 0) {
        uint32_t jedec = 0;
        if (spiFlashJedecId(&jedec)) len = spiFlashChipSize(jedec);
    }
    if (len == 0) {
        server.send(400, "text/plain; charset=utf-8", "-ERR brak rozmiaru i nieznana kosc");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    uint32_t first = 0, nonBlank = 0;
    g_out->print("SPI blank check 0x");
    g_out->print(addr, HEX);
    g_out->print(" + ");
    g_out->print(len);
    g_out->print(" B...\r\n");
    if (!spiFlashBlankCheck(addr, len, &first, &nonBlank)) {
        g_out->print("-ERR odczyt SPI nieudany\r\n");
    } else if (nonBlank == 0) {
        g_out->print("+OK obszar czysty (same 0xFF)\r\n");
    } else {
        g_out->print("-ERR niepustych bajtow: ");
        g_out->print(nonBlank);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(first, HEX);
        g_out->print(")\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

/* --- rejestry analogowe TLSR8258 ------------------------------------------- */
static void handleAreg() {
    uint32_t a = 0;
    if (server.hasArg("a")) a = (uint32_t)strtoul(server.arg("a").c_str(), nullptr, 0);
    char buf[32];
    snprintf(buf, sizeof(buf), "AREG %lu", (unsigned long)a);
    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    dispatch(buf);
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleAwrite() {
    uint32_t a = 0, v = 0;
    if (server.hasArg("a")) a = (uint32_t)strtoul(server.arg("a").c_str(), nullptr, 0);
    if (server.hasArg("v")) v = (uint32_t)strtoul(server.arg("v").c_str(), nullptr, 0);
    char buf[40];
    snprintf(buf, sizeof(buf), "AWR %lu %lu", (unsigned long)a, (unsigned long)v);
    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    dispatch(buf);
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleAdump() {
    char buf[8] = "ADUMP";
    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    dispatch(buf);
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSpiRead() {
    uint32_t addr = parseAddr();
    uint32_t len = parseLen("len", 256);
    if (len == 0 || (uint64_t)addr + len > SPI_FLASH_MAX_ADDR) {
        server.send(400, "text/plain; charset=utf-8", "-ERR zly adres/rozmiar");
        return;
    }
    uint32_t jedec = 0;
    if (!spiFlashJedecId(&jedec)) {
        server.send(500, "text/plain; charset=utf-8", "-ERR brak kosci SPI");
        return;
    }
    server.setContentLength(len);
    server.sendHeader("Content-Disposition", "attachment; filename=\"spi.bin\"");
    server.send(200, "application/octet-stream", "");

    uint8_t buf[512];
    uint32_t a = addr;
    uint32_t rem = len;
    while (rem) {
        uint32_t c = rem < sizeof(buf) ? rem : (uint32_t)sizeof(buf);
        if (!spiFlashRead(a, buf, c)) {
            /* Zachowaj deklarowana dlugosc - dopelnij zerami, zeby nie
               zawiesic klienta na krotszej odpowiedzi. */
            memset(buf, 0, sizeof(buf));
            c = rem < sizeof(buf) ? rem : (uint32_t)sizeof(buf);
        }
        server.sendContent((const char *)buf, c);
        a += c;
        rem -= c;
    }
}

static void handleSpiVerify() {
    if (!upBuf || !upLen || upOverflow) {
        server.send(400, "text/plain; charset=utf-8",
                    upOverflow ? "-ERR plik za duzy (limit SPI_WEB_MAX)" : "-ERR brak pliku (pole: firmware)");
        return;
    }
    uint32_t addr = parseAddr();
    if ((uint64_t)addr + upLen > SPI_FLASH_MAX_ADDR) {
        server.send(400, "text/plain; charset=utf-8", "-ERR adres+rozmiar poza flash");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    uint32_t firstBad = 0, bad = 0;
    g_out->print("SPI weryfikacja: ");
    g_out->print(upName);
    g_out->print(" (");
    g_out->print(upLen);
    g_out->print(" B) pod 0x");
    g_out->print(addr, HEX);
    g_out->print("\r\n");

    if (!spiFlashVerify(addr, upBuf, upLen, &firstBad, &bad)) {
        g_out->print("-ERR odczyt SPI nieudany\r\n");
    } else if (bad == 0) {
        g_out->print("+OK zgodne (");
        g_out->print(upLen);
        g_out->print(" B)\r\n");
    } else {
        g_out->print("-ERR niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print("/");
        g_out->print(upLen);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(firstBad, HEX);
        g_out->print(")\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSpiWrite() {
    if (!haveConsent()) {
        server.send(403, "text/plain; charset=utf-8",
                    "-ERR brak zgody na zapis (consent=1)");
        return;
    }
    if (!upBuf || !upLen || upOverflow) {
        server.send(400, "text/plain; charset=utf-8",
                    upOverflow ? "-ERR plik za duzy (limit SPI_WEB_MAX)" : "-ERR brak pliku (pole: firmware)");
        return;
    }
    uint32_t addr = parseAddr();
    if ((uint64_t)addr + upLen > SPI_FLASH_MAX_ADDR) {
        server.send(400, "text/plain; charset=utf-8", "-ERR adres+rozmiar poza flash");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    g_out->print("SPI kasowanie + zapis + weryfikacja: ");
    g_out->print(upName);
    g_out->print(" (");
    g_out->print(upLen);
    g_out->print(" B) pod 0x");
    g_out->print(addr, HEX);
    g_out->print("\r\n");

    if (!spiFlashWriteRange(addr, upBuf, upLen)) {
        g_out->print("-ERR zapis SPI nieudany\r\n");
        g_out = saved;
        sendCaptured(500, cap);
        return;
    }

    uint32_t firstBad = 0, bad = 0;
    if (!spiFlashVerify(addr, upBuf, upLen, &firstBad, &bad)) {
        g_out->print("-ERR odczyt SPI po zapisie nieudany\r\n");
    } else if (bad == 0) {
        g_out->print("+OK wgrano ");
        g_out->print(upLen);
        g_out->print(" B, weryfikacja zgodna\r\n");
    } else {
        g_out->print("-ERR po zapisie niezgodnych bajtow: ");
        g_out->print(bad);
        g_out->print(" (pierwszy pod 0x");
        g_out->print(firstBad, HEX);
        g_out->print(")\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSpiErase() {
    if (!haveConsent()) {
        server.send(403, "text/plain; charset=utf-8",
                    "-ERR brak zgody na kasowanie (consent=1)");
        return;
    }
    uint32_t addr = parseAddr();
    uint32_t len = parseLen("len", SPI_FLASH_SECTOR);
    if (len == 0 || (uint64_t)addr + len > SPI_FLASH_MAX_ADDR) {
        server.send(400, "text/plain; charset=utf-8", "-ERR zly adres/rozmiar");
        return;
    }

    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;

    uint32_t s = addr & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    uint32_t e = (addr + len + SPI_FLASH_SECTOR - 1) & ~(uint32_t)(SPI_FLASH_SECTOR - 1);
    uint32_t n = 0;
    bool ok = true;
    for (uint32_t a = s; a < e; a += SPI_FLASH_SECTOR) {
        if (!spiFlashEraseSector(a)) { ok = false; break; }
        n++;
    }
    if (!ok) {
        g_out->print("-ERR kasowanie sektora nieudane (po ");
        g_out->print(n);
        g_out->print(" sektorach)\r\n");
    } else {
        g_out->print("+OK skasowano ");
        g_out->print(n);
        g_out->print(" sektorow (0x");
        g_out->print(s, HEX);
        g_out->print(" .. 0x");
        g_out->print(e, HEX);
        g_out->print(")\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSpiEraseChip() {
    if (!haveConsent()) {
        server.send(403, "text/plain; charset=utf-8",
                    "-ERR brak zgody na kasowanie (consent=1)");
        return;
    }
    String cap;
    LogPrint hp(&cap, 60000);
    Print *saved = g_out;
    g_out = &hp;
    if (spiFlashEraseChip()) {
        g_out->print("+OK skasowano cala kosc\r\n");
    } else {
        g_out->print("-ERR kasowanie kosci nieudane\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

/* --- log na zywo ------------------------------------------------------------ */
static void handleLog() {
    uint32_t after = (uint32_t)strtoul(server.arg("after").c_str(), nullptr, 0);
    String t = webLogSince(after);
    String j = "{\"seq\":";
    j += logSeq;
    j += ",\"text\":\"";
    jsonEscape(j, t);
    j += "\"}";
    server.send(200, "application/json", j);
}

/* --- karta SD --------------------------------------------------------------- */
static void handleSdInfo() {
    bool mounted = sdCardMounted();
    char j[192];
    snprintf(j, sizeof(j),
             "{\"mounted\":%s,\"type\":\"%s\",\"total\":%llu,\"used\":%llu}",
             mounted ? "true" : "false", sdCardTypeName(),
             (unsigned long long)sdCardTotalBytes(),
             (unsigned long long)sdCardUsedBytes());
    server.send(200, "application/json", j);
}

static void handleSdLs() {
    if (!sdCardMounted()) {
        server.send(200, "application/json",
                    "{\"ok\":false,\"err\":\"brak karty SD\",\"files\":[]}");
        return;
    }
    File dir = SD.open("/");
    if (!dir || !dir.isDirectory()) {
        if (dir) dir.close();
        server.send(200, "application/json",
                    "{\"ok\":false,\"err\":\"nie mozna otworzyc /\",\"files\":[]}");
        return;
    }
    String j = "{\"ok\":true,\"files\":[";
    bool first = true;
    File f = dir.openNextFile();
    while (f) {
        if (!first) j += ',';
        first = false;
        j += "{\"n\":\"";
        jsonEscape(j, f.name());
        j += "\",\"s\":";
        j += String((unsigned long long)f.size());
        j += ",\"d\":";
        j += f.isDirectory() ? "true" : "false";
        j += '}';
        f = dir.openNextFile();
    }
    j += "]}";
    dir.close();
    server.send(200, "application/json", j);
}

static void handleSdDownload() {
    if (!sdCardMounted()) {
        server.send(500, "text/plain; charset=utf-8", "-ERR brak karty SD");
        return;
    }
    String name = server.arg("file");
    if (!name.length() || name.indexOf('/') >= 0 || name == "." || name == "..") {
        server.send(400, "text/plain; charset=utf-8", "-ERR zla nazwa pliku");
        return;
    }
    String path = "/" + name;
    File f = SD.open(path, FILE_READ);
    if (!f || f.isDirectory()) {
        if (f) f.close();
        server.send(404, "text/plain; charset=utf-8", "-ERR brak pliku");
        return;
    }
    size_t total = f.size();
    server.setContentLength(total);
    server.sendHeader("Content-Disposition", "attachment; filename=\"" + name + "\"");
    server.send(200, "application/octet-stream", "");

    uint8_t buf[512];
    while (f.available()) {
        int n = f.read(buf, sizeof(buf));
        if (n <= 0) break;
        server.sendContent((const char *)buf, (size_t)n);
    }
    f.close();
}

/* Wlasciwy odbior pliku robi onUpload() (pole "sdfile"). Ta funkcja uruchamia
 * sie PO zakonczeniu uploadu i zwraca wynik. */
static void handleSdUpload() {
    String cap;
    LogPrint hp(&cap, 4096);
    Print *saved = g_out;
    g_out = &hp;
    if (sdUpError || !sdUpWritten) {
        g_out->print("-ERR zapis na SD nieudany\r\n");
        g_out = saved;
        sendCaptured(500, cap);
        return;
    }
    g_out->print("+OK zapisano ");
    g_out->print(sdUpTarget);
    g_out->print(" (");
    g_out->print(sdUpWritten);
    g_out->print(" B)\r\n");
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSdDelete() {
    if (!sdCardMounted()) {
        server.send(500, "text/plain; charset=utf-8", "-ERR brak karty SD");
        return;
    }
    String name = server.arg("file");
    if (!name.length() || name.indexOf('/') >= 0) {
        server.send(400, "text/plain; charset=utf-8", "-ERR zla nazwa pliku");
        return;
    }
    String path = "/" + name;
    String cap;
    LogPrint hp(&cap, 4096);
    Print *saved = g_out;
    g_out = &hp;
    if (sdCardDelete(path.c_str())) {
        g_out->print("+OK usunieto ");
        g_out->print(name);
        g_out->print("\r\n");
    } else {
        g_out->print("-ERR nie mozna usunac ");
        g_out->print(name);
        g_out->print("\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleSdFormat() {
    if (!haveConsent()) {
        server.send(403, "text/plain; charset=utf-8",
                    "-ERR brak zgody na formatowanie (consent=1)");
        return;
    }
    String cap;
    LogPrint hp(&cap, 4096);
    Print *saved = g_out;
    g_out = &hp;
    g_out->print("Formatowanie karty SD (FAT)...\r\n");
    if (sdCardFormat(hp)) {
        g_out->print("+OK sformatowano i zamontowano\r\n");
    } else {
        g_out->print("-ERR formatowanie nieudane\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleWifiScan() {
    /* Wyniki pochodzą z cache (skan w tle) — kliknięcie "Skanuj" nie zdejmuje
     * AP. Świeży skan uruchamiamy tylko, gdy cache jest pusty lub przeterminowany. */
    if (wifiScanCount == 0 || (millis() - wifiScanStamp) > WIFI_SCAN_STALE_MS) {
        wifiScanNow();
    }
    String j = "{\"ok\":true,\"age_ms\":";
    j += (wifiScanCount > 0) ? String(millis() - wifiScanStamp) : "0";
    j += ",\"networks\":[";
    for (int i = 0; i < wifiScanCount; i++) {
        if (i) j += ',';
        j += "{\"ssid\":\"";
        jsonEscape(j, wifiScanSsid[i]);
        j += "\",\"rssi\":";
        j += String(wifiScanRssi[i]);
        j += ",\"auth\":";
        j += wifiScanOpen[i] ? "false" : "true";
        j += '}';
    }
    j += "]}";
    server.send(200, "application/json", j);
}

static void handleWifiStatus() {
    bool connected = WiFi.status() == WL_CONNECTED;
    char j[256];
    snprintf(j, sizeof(j),
             "{\"ap\":\"%s\",\"sta\":\"%s\",\"connected\":%s,\"ip\":\"%s\"}",
             WEB_AP_SSID, staSsid.c_str(),
             connected ? "true" : "false",
             connected ? WiFi.localIP().toString().c_str() : "");
    server.send(200, "application/json", j);
}

static int hexNib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

/* Dekodowanie formularza: '+' -> spacja, %XX -> bajt. */
static String formDecode(const String &s) {
    String out;
    out.reserve(s.length());
    for (int i = 0; i < (int)s.length(); i++) {
        char c = s[i];
        if (c == '+') {
            out += ' ';
        } else if (c == '%' && i + 2 < (int)s.length()) {
            out += (char)((hexNib(s[i + 1]) << 4) | hexNib(s[i + 2]));
            i += 2;
        } else {
            out += c;
        }
    }
    return out;
}

static bool formDecodeArg(const String &body, const char *name, String &out) {
    out = "";
    if (!body.length()) return false;
    int pos = body.indexOf(String(name) + "=");
    if (pos < 0) return false;
    pos += (int)strlen(name) + 1;
    int end = body.indexOf('&', pos);
    if (end < 0) end = body.length();
    out = formDecode(body.substring(pos, end));
    return true;
}

static void handleWifiSave() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");

    /* Fallback: klienci, którzy wysyłają dane tylko w body (form-urlencoded)
     * bez Content-Type, nie trafiają do arg(); parsujemy wtedy body ręcznie. */
    if (!ssid.length()) {
        String body = server.arg("plain");
        if (body.length()) {
            formDecodeArg(body, "ssid", ssid);
            formDecodeArg(body, "pass", pass);
        }
    }
    ssid.trim();

    String cap;
    LogPrint hp(&cap, 4096);
    Print *saved = g_out;
    g_out = &hp;

    if (!ssid.length()) {
        wifiSave("", "");
        WiFi.disconnect(true);
        WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD);
        g_out->print("+OK usunieto zapisane WiFi (tylko AP)\r\n");
    } else {
        wifiSave(ssid, pass);
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD);
        WiFi.begin(ssid.c_str(), pass.c_str());
        g_out->print("+OK zapisano, lacze z \"");
        g_out->print(ssid);
        g_out->print("\"...\r\n");
    }

    g_out = saved;
    sendCaptured(200, cap);
}

/* ------------------------------------------------------------------------- */
/* UART terminal */
static void handleUart() {
    String action = server.arg("action");
    uint32_t baud = server.arg("baud").toInt();
    if (!baud) baud = 115200;

    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (action == "start") {
        uartStart(baud);
        g_out->print("+OK UART start ");
        g_out->print(baud);
        g_out->print(" 8N1 (TX=");
        g_out->print(UART_TX_PIN);
        g_out->print(", RX=");
        g_out->print(UART_RX_PIN);
        g_out->print(")\r\n");
    } else if (action == "stop") {
        uartStop();
        g_out->print("+OK UART stop\r\n");
    } else {
        g_out->print("-ERR akcja: start|stop\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleUartSend() {
    String txt = server.arg("txt");
    String hex = server.arg("hex");
    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (!uartActive) {
        g_out->print("-ERR UART nieaktywny\r\n");
    } else if (hex.length()) {
        uint8_t buf[256];
        int n = 0;
        for (int i = 0; i + 1 < (int)hex.length() && n < (int)sizeof(buf); i += 2) {
            buf[n++] = (uint8_t)((hexNib(hex[i]) << 4) | hexNib(hex[i + 1]));
        }
        Serial1.write(buf, n);
        g_out->print("+OK wyslano ");
        g_out->print(n);
        g_out->print(" bajtow HEX\r\n");
    } else if (txt.length()) {
        Serial1.print(txt);
        g_out->print("+OK wyslano tekst\r\n");
    } else {
        g_out->print("-ERR brak danych (txt lub hex)\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleUartAutobaud() {
    uint32_t baud = uartDetectBaud(2500);
    if (!baud) {
        server.send(200, "text/plain; charset=utf-8",
                    "-ERR nie wykryto zadnego zbocza na RX - uruchom cel i wyslij dane");
        return;
    }
    uartStart(baud);
    String out = "+OK auto-baud: wykryto ";
    out += baud;
    out += " 8N1 (UART uruchomiony)\r\n";
    server.send(200, "text/plain; charset=utf-8", out);
}

static void handleUartSdlog() {
    int en = server.arg("en").toInt();
    String path = server.arg("path");
    if (!path.length()) path = "/uart.log";

    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (en) {
        uartLogFlush();
        uartLogPath = path;
        uartLogOn = true;
        uartLogLast = millis();
        if (sdCardBegin()) {
            g_out->print("+OK logowanie RX -> ");
            g_out->print(path);
            g_out->print("\r\n");
        } else {
            uartLogOn = false;
            g_out->print("-ERR brak karty SD - logowanie wylaczone\r\n");
        }
    } else {
        uartLogFlush();
        uartLogOn = false;
        g_out->print("+OK logowanie RX wylaczone\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleUartRx() {
    uint32_t after = strtoul(server.arg("after").c_str(), NULL, 10);
    String body = "{\"seq\":" + String(uartSeq) + ",\"active\":" + (uartActive ? "true" : "false") + ",\"data\":\"";
    jsonEscape(body, uartSince(after));
    body += "\"}";
    server.send(200, "application/json", body);
}

/* ------------------------------------------------------------------------- */
/* RS485 */
static void handleRs485() {
    String action = server.arg("action");
    uint32_t baud = server.arg("baud").toInt();
    if (!baud) baud = 9600;

    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (action == "start") {
        rsStart(baud);
        g_out->print("+OK RS485 start ");
        g_out->print(baud);
        g_out->print(" 8N1 (TX/DI=");
        g_out->print(RS485_TX_PIN);
        g_out->print(", RX/RO=");
        g_out->print(RS485_RX_PIN);
        g_out->print(", DE=");
        g_out->print(RS485_DE_PIN);
        g_out->print(")\r\n");
    } else if (action == "stop") {
        rsStop();
        g_out->print("+OK RS485 stop\r\n");
    } else {
        g_out->print("-ERR akcja: start|stop\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleRs485Send() {
    String txt = server.arg("txt");
    String hex = server.arg("hex");
    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (!rsActive) {
        g_out->print("-ERR RS485 nieaktywny\r\n");
    } else if (hex.length()) {
        uint8_t buf[256];
        int n = 0;
        for (int i = 0; i + 1 < (int)hex.length() && n < (int)sizeof(buf); i += 2) {
            buf[n++] = (uint8_t)((hexNib(hex[i]) << 4) | hexNib(hex[i + 1]));
        }
        rsSend(buf, n);
        g_out->print("+OK wyslano ");
        g_out->print(n);
        g_out->print(" bajtow HEX\r\n");
    } else if (txt.length()) {
        rsSend((const uint8_t *)txt.c_str(), txt.length());
        g_out->print("+OK wyslano tekst\r\n");
    } else {
        g_out->print("-ERR brak danych (txt lub hex)\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleRs485Rx() {
    uint32_t after = strtoul(server.arg("after").c_str(), NULL, 10);
    String body = "{\"seq\":" + String(rsSeq) + ",\"active\":" + (rsActive ? "true" : "false") + ",\"data\":\"";
    jsonEscape(body, rsSince(after));
    body += "\"}";
    server.send(200, "application/json", body);
}

/* ------------------------------------------------------------------------- */
/* 1-Wire */
static void handleOwScan() {
    String j = "{\"devices\":[";
    uint8_t addr[8];
    owBus.reset_search();
    bool first = true;
    while (owBus.search(addr)) {
        if (OneWire::crc8(addr, 7) == addr[7]) {
            if (!first) j += ',';
            first = false;
            j += "{\"rom\":\"";
            jsonEscape(j, owRomHex(addr));
            j += "\",\"family\":\"";
            jsonEscape(j, owFamilyName(addr[0]));
            j += "\"}";
        }
    }
    owBus.reset_search();
    j += "]}";
    server.send(200, "application/json", j);
}

static void handleOwTemp() {
    String rom = server.arg("rom");
    uint8_t addr[8];
    uint8_t *use = NULL;
    if (rom.length() == 16) {
        for (int i = 0; i < 8; i++) {
            int hi = hexNib(rom[i * 2]);
            int lo = hexNib(rom[i * 2 + 1]);
            addr[i] = (uint8_t)((hi << 4) | lo);
        }
        use = addr;
    }
    float c = 0;
    bool ok = owReadDs18b20(use, c);
    String j = "{\"ok\":";
    j += ok ? "true" : "false";
    if (ok) {
        j += ",\"c\":" + String(c, 2);
        j += ",\"f\":" + String(c * 9.0f / 5.0f + 32.0f, 2);
    }
    j += "}";
    server.send(200, "application/json", j);
}

/* ------------------------------------------------------------------------- */
/* I²C */
static void handleI2cScan() {
    i2cBegin();
    String j = "{\"devices\":[";
    bool first = true;
    for (int a = 1; a < 127; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) {
            if (!first) j += ',';
            first = false;
            char b[16];
            snprintf(b, sizeof(b), "{\"addr\":\"0x%02X\",\"dec\":%d}", a, a);
            j += b;
        }
    }
    j += "]}";
    server.send(200, "application/json", j);
}

static void handleI2cRead() {
    i2cBegin();
    int addr = (int)strtoul(server.arg("addr").c_str(), NULL, 16);
    if (addr == 0) addr = (int)server.arg("dec").toInt();
    int reg = (int)strtoul(server.arg("reg").c_str(), NULL, 16);
    int len = server.arg("len").toInt();
    if (len < 1) len = 1;
    if (len > 64) len = 64;

    String j = "{\"ok\":false}";
    if (addr < 1 || addr > 126) {
        server.send(200, "application/json", j);
        return;
    }

    Wire.beginTransmission(addr);
    Wire.write((uint8_t)reg);
    if (Wire.endTransmission() != 0) {
        server.send(200, "application/json", j);
        return;
    }
    if (Wire.requestFrom(addr, len) < len) {
        server.send(200, "application/json", j);
        return;
    }
    String bytes;
    for (int i = 0; i < len; i++) {
        uint8_t v = Wire.read();
        char b[4];
        snprintf(b, sizeof(b), "%02X", v);
        bytes += b;
    }
    j = "{\"ok\":true,\"hex\":\"";
    jsonEscape(j, bytes);
    j += "\"}";
    server.send(200, "application/json", j);
}

/* EEPROM I²C 24xx (24C01..24C512). abits=8 (24C01/02) lub 16 (24C04+). */
static bool eeprom24Read(uint8_t dev, uint16_t off, uint8_t *buf, uint16_t n, uint8_t abits) {
    i2cBegin();
    Wire.beginTransmission(dev);
    if (abits == 16) Wire.write((uint8_t)(off >> 8));
    Wire.write((uint8_t)(off & 0xFF));
    if (Wire.endTransmission(false) != 0) return false;   /* bez STOP - repeated start */
    if (Wire.requestFrom((uint8_t)dev, (uint8_t)n) < n) return false;
    for (uint16_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}

static bool eeprom24Write(uint8_t dev, uint16_t off, const uint8_t *buf, uint16_t n, uint8_t abits) {
    i2cBegin();
    uint16_t done = 0;
    while (done < n) {
        Wire.beginTransmission(dev);
        if (abits == 16) Wire.write((uint8_t)((off + done) >> 8));
        Wire.write((uint8_t)((off + done) & 0xFF));
        uint16_t chunk = 16 - ((off + done) & 0x0F);      /* granica strony 16 B */
        if (chunk > n - done) chunk = n - done;
        for (uint16_t i = 0; i < chunk; i++) Wire.write(buf[done + i]);
        if (Wire.endTransmission() != 0) return false;
        delay(6);                                          /* cykl zapisu */
        done += chunk;
    }
    return true;
}

static void handleEepromRead() {
    int dev = (int)strtoul(server.arg("dev").c_str(), NULL, 16);
    if (!dev) dev = 0x50;
    uint16_t off = (uint16_t)strtoul(server.arg("off").c_str(), NULL, 0);
    int len = server.arg("len").toInt();
    if (len < 1) len = 16;
    if (len > 256) len = 256;
    uint8_t abits = server.arg("abits") == "8" ? 8 : 16;

    String j = "{\"ok\":false}";
    if (dev < 1 || dev > 127) {
        server.send(200, "application/json", j);
        return;
    }
    uint8_t buf[256];
    if (!eeprom24Read((uint8_t)dev, off, buf, (uint16_t)len, abits)) {
        server.send(200, "application/json", j);
        return;
    }
    String hex;
    for (int i = 0; i < len; i++) {
        char b[3];
        snprintf(b, sizeof(b), "%02X", buf[i]);
        hex += b;
    }
    j = "{\"ok\":true,\"hex\":\"";
    jsonEscape(j, hex);
    j += "\"}";
    server.send(200, "application/json", j);
}

static void handleEepromWrite() {
    int dev = (int)strtoul(server.arg("dev").c_str(), NULL, 16);
    if (!dev) dev = 0x50;
    uint16_t off = (uint16_t)strtoul(server.arg("off").c_str(), NULL, 0);
    String hex = server.arg("hex");
    uint8_t abits = server.arg("abits") == "8" ? 8 : 16;

    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (dev < 1 || dev > 127) {
        g_out->print("-ERR zly adres I2C\r\n");
    } else if (!hex.length() || (hex.length() & 1)) {
        g_out->print("-ERR hex: parzysta liczba znakow\r\n");
    } else {
        uint8_t buf[256];
        uint16_t n = 0;
        for (int i = 0; i + 1 < (int)hex.length() && n < 256; i += 2) {
            buf[n++] = (uint8_t)((hexNib(hex[i]) << 4) | hexNib(hex[i + 1]));
        }
        if (eeprom24Write((uint8_t)dev, off, buf, n, abits)) {
            g_out->print("+OK zapisano ");
            g_out->print(n);
            g_out->print(" B do EEPROM 0x");
            g_out->print(dev, HEX);
            g_out->print(" pod offset 0x");
            g_out->print(off, HEX);
            g_out->print("\r\n");
        } else {
            g_out->print("-ERR zapis EEPROM nieudany\r\n");
        }
    }
    g_out = saved;
    sendCaptured(200, cap);
}

/* ------------------------------------------------------------------------- */
/* GPIO */
static void handleGpioSet() {
    int pin = server.arg("pin").toInt();
    String mode = server.arg("mode");
    int value = server.arg("value").toInt() ? 1 : 0;

    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;

    if (!gpioInList(pin)) {
        g_out->print("-ERR pin ");
        g_out->print(pin);
        g_out->print(" niedozwolony (wolne: 1,2,7,36,37,38,39,40,47,48)\r\n");
    } else if (mode == "out") {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, value);
        g_out->print("+OK GPIO");
        g_out->print(pin);
        g_out->print(" = wyjscie ");
        g_out->print(value ? "HIGH" : "LOW");
        g_out->print("\r\n");
    } else if (mode == "in") {
        pinMode(pin, INPUT);
        g_out->print("+OK GPIO");
        g_out->print(pin);
        g_out->print(" = wejscie\r\n");
    } else if (mode == "in_pullup") {
        pinMode(pin, INPUT_PULLUP);
        g_out->print("+OK GPIO");
        g_out->print(pin);
        g_out->print(" = wejscie PULLUP\r\n");
    } else if (mode == "in_pulldown") {
        pinMode(pin, INPUT_PULLDOWN);
        g_out->print("+OK GPIO");
        g_out->print(pin);
        g_out->print(" = wejscie PULLDOWN\r\n");
    } else {
        g_out->print("-ERR mode: out|in|in_pullup|in_pulldown\r\n");
    }
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleGpioGet() {
    String j = "{\"pins\":[";
    for (int i = 0; i < GPIO_NUM; i++) {
        if (i) j += ',';
        int p = gpioPins[i];
        int v = digitalRead(p);
        j += "{\"pin\":" + String(p) + ",\"val\":" + String(v) + "}";
    }
    j += "]}";
    server.send(200, "application/json", j);
}

static void handleGpioScan() {
    server.send(200, "application/json", "{\"pins\":[1,2,7,36,37,38,39,40,47,48]}");
}

/* ------------------------------------------------------------------------- */
/* ESP bridge (esptool) */
static void handleEspStatus() {
    String j = "{\"bridged\":" + String(espBridged ? "true" : "false") +
        ",\"port\":" + String(ESP_BRIDGE_PORT) +
        ",\"io0\":" + String(ESP_IO0_PIN) +
        ",\"en\":" + String(ESP_EN_PIN) +
        ",\"uart\":\"" + String(UART_TX_PIN) + "/" + String(UART_RX_PIN) + "\"}";
    server.send(200, "application/json", j);
}

static void handleEspBoot() {
    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;
    espEnterBoot();
    g_out->print("+OK cel w trybie bootloadera (IO0=");
    g_out->print(ESP_IO0_PIN);
    g_out->print(", EN=");
    g_out->print(ESP_EN_PIN);
    g_out->print(")\r\n");
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleEspReset() {
    String cap;
    LogPrint hp(&cap, 2048);
    Print *saved = g_out;
    g_out = &hp;
    pinMode(ESP_IO0_PIN, OUTPUT);
    pinMode(ESP_EN_PIN, OUTPUT);
    digitalWrite(ESP_IO0_PIN, HIGH);
    digitalWrite(ESP_EN_PIN, LOW);
    delay(60);
    digitalWrite(ESP_EN_PIN, HIGH);
    g_out->print("+OK reset celu (normalny boot)\r\n");
    g_out = saved;
    sendCaptured(200, cap);
}

static void handleNotFound() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
}

/* ------------------------------------------------------------------------- */
void webuiSetup() {
    wifiLoad();

    /* Zawsze AP_STA: AP do konfiguracji + opcjonalnie STA do sieci domowej.
     * Dzieki temu skanowanie WiFi dziala, a AP pozostaje dostepne. */
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WEB_AP_SSID, WEB_AP_PASSWORD);
    if (staSsid.length() > 0) {
        WiFi.begin(staSsid.c_str(), staPass.c_str());
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/api/status", HTTP_GET, handleStatus);
    server.on("/api/run", HTTP_POST, handleRun);
    server.on("/api/verify", HTTP_POST, handleVerify);
    server.on("/api/flash", HTTP_POST, handleFlash);
    server.on("/api/spidetect", HTTP_GET, handleSpiDetect);
    server.on("/api/spiblank", HTTP_POST, handleSpiBlank);
    server.on("/api/areg", HTTP_GET, handleAreg);
    server.on("/api/awr", HTTP_POST, handleAwrite);
    server.on("/api/adump", HTTP_GET, handleAdump);
    server.on("/api/spiread", HTTP_GET, handleSpiRead);
    server.on("/api/spiverify", HTTP_POST, handleSpiVerify);
    server.on("/api/spiwrite", HTTP_POST, handleSpiWrite);
    server.on("/api/spierase", HTTP_POST, handleSpiErase);
    server.on("/api/spierasechip", HTTP_POST, handleSpiEraseChip);
    server.on("/api/log", HTTP_GET, handleLog);
    server.on("/api/sdinfo", HTTP_GET, handleSdInfo);
    server.on("/api/sdls", HTTP_GET, handleSdLs);
    server.on("/api/sddownload", HTTP_GET, handleSdDownload);
    server.on("/api/sdupload", HTTP_POST, handleSdUpload);
    server.on("/api/sddelete", HTTP_POST, handleSdDelete);
    server.on("/api/sdformat", HTTP_POST, handleSdFormat);
    server.on("/api/wifiscan", HTTP_GET, handleWifiScan);
    server.on("/api/wifistatus", HTTP_GET, handleWifiStatus);
    server.on("/api/wifisave", HTTP_POST, handleWifiSave);
    server.on("/api/uart", HTTP_POST, handleUart);
    server.on("/api/uart/send", HTTP_POST, handleUartSend);
    server.on("/api/uart/autobaud", HTTP_POST, handleUartAutobaud);
    server.on("/api/uart/sdlog", HTTP_POST, handleUartSdlog);
    server.on("/api/uartrx", HTTP_GET, handleUartRx);
    server.on("/api/rs485", HTTP_POST, handleRs485);
    server.on("/api/rs485/send", HTTP_POST, handleRs485Send);
    server.on("/api/rs485rx", HTTP_GET, handleRs485Rx);
    server.on("/api/owscan", HTTP_GET, handleOwScan);
    server.on("/api/owtemp", HTTP_GET, handleOwTemp);
    server.on("/api/i2cscan", HTTP_GET, handleI2cScan);
    server.on("/api/i2cread", HTTP_GET, handleI2cRead);
    server.on("/api/eeprom/read", HTTP_GET, handleEepromRead);
    server.on("/api/eeprom/write", HTTP_POST, handleEepromWrite);
    server.on("/api/gpio/set", HTTP_POST, handleGpioSet);
    server.on("/api/gpio/get", HTTP_GET, handleGpioGet);
    server.on("/api/gpio/scan", HTTP_GET, handleGpioScan);
    server.on("/api/esp/status", HTTP_GET, handleEspStatus);
    server.on("/api/esp/boot", HTTP_POST, handleEspBoot);
    server.on("/api/esp/reset", HTTP_POST, handleEspReset);
    server.onFileUpload(onUpload);
    server.onNotFound(handleNotFound);
    server.begin();
    espServer.begin();

    if (MDNS.begin("swsprog")) {
        MDNS.addService("http", "tcp", 80);
        g_out->print("WWW: mDNS -> http://swsprog.local\r\n");
    } else {
        g_out->print("WWW: mDNS niedostepny\r\n");
    }

    ArduinoOTA.setHostname("swsprog");
    ArduinoOTA.setPassword("swsprog");
    ArduinoOTA.onStart([]() {
        uartStop();
        g_out->print("OTA: aktualizacja firmware...\r\n");
    });
    ArduinoOTA.onEnd([]() {
        g_out->print("OTA: gotowe, restart...\r\n");
    });
    ArduinoOTA.onError([](ota_error_t e) {
        g_out->print("OTA: blad ");
        g_out->print((int)e);
        g_out->print("\r\n");
    });
    ArduinoOTA.begin();
    g_out->print("WWW: OTA -> http://swsprog.local (haslo: swsprog)\r\n");

    g_out->print("WWW: AP \"");
    g_out->print(WEB_AP_SSID);
    g_out->print("\" -> http://");
    g_out->print(WiFi.softAPIP().toString());
    g_out->print("\r\n");
    if (staSsid.length() > 0) {
        g_out->print("WWW: laczenie z siecia \"");
        g_out->print(staSsid);
        g_out->print("\"...\r\n");
    }
}

void webuiLoop() {
    ArduinoOTA.handle();
    static bool staAnnounced = false;
    static uint32_t nextBgScan = 0;
    if (staSsid.length() > 0 && !staAnnounced && WiFi.status() == WL_CONNECTED) {
        staAnnounced = true;
        g_out->print("WWW: IP w sieci domowej -> http://");
        g_out->print(WiFi.localIP().toString());
        g_out->print("\r\n");
    }

    if (!espBridged) uartPoll();   // podczas mostka ESP bajty ida wprost do klienta esptool
    rsPoll();
    espBridgeHandle();

    /* Skan w tle: pierwszy raz 2 s po starcie, potem co 30 s. /api/wifiscan
     * oddaje wtedy gotowe wyniki z cache, a telefon nie traci AP w momencie
     * kliknięcia "Skanuj". Podczas mostka ESP (TCP esptool) skan pomijamy,
     * bo zmiana trybu WiFi rozwalilaby połączenie TCP. */
    uint32_t now = millis();
    if (!espBridged) {
        if (nextBgScan == 0) {
            nextBgScan = now + 2000;
        }
        if ((int32_t)(now - nextBgScan) >= 0) {
            nextBgScan = now + WIFI_SCAN_INTERVAL_MS;
            wifiScanNow();
        }
    }

    server.handleClient();
}
