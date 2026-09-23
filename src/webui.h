#pragma once

#include <Arduino.h>

/* Bieżące wyjście tekstowe: domyślnie Serial (konsola COM11).
 * Warstwa WWW podmienia je na bufor HTTP na czas wykonania komendy. */
extern Print *g_out;

/* Dołączenie bajtu/tekstu do pierścieniowego bufora logów WWW (żywy podgląd).
 * Wywoływane z main.cpp (domyślne wyjście TeePrint) oraz z webui.cpp. */
void webLogWrite(const uint8_t *p, size_t n);

/* Domyślne wyjście konsoli: Serial + bufor logów WWW jednocześnie. */
class TeePrint : public Print {
public:
    size_t write(uint8_t c) override {
        Serial.write(c);
        webLogWrite(&c, 1);
        return 1;
    }
    size_t write(const uint8_t *p, size_t n) override {
        Serial.write(p, n);
        webLogWrite(p, n);
        return n;
    }
};

/* Wejście do konsoli (zawsze Serial). */
void dispatch(char *line);

/* Aktywacja celu + test SWS (zapis/odczyt dzielnika [0x00b2]).
 * Deklaracja z wartością domyślną parametru verbose. */
bool swsLinkTest(uint8_t div, bool verbose = true);

/* Skan sieci Wi-Fi (niezawodny — radio przełączane na czas skanu w tryb STA).
 * Zwraca liczbę znalezionych sieci; wyniki odczytuje się przez wifiScanGet(). */
int wifiScanNow();

/* Odczyt wyniku skanu o indeksie i (0..n-1). Zwraca false dla złego indeksu. */
bool wifiScanGet(int i, String &ssid, int &rssi, bool &openNet);

/* Ścieżka ostatniego skanu: 0 = skan przy podniesionym AP, 1 = fallback STA. */
uint8_t wifiScanPathGet();

/* Konfiguracja STA z konsoli (fallback, gdy przeglądarka nie ma dostępu do AP). */
String wifiStaGetSsid();
bool   wifiStaConnected();
String wifiStaGetIp();
void   wifiJoin(const String &ssid, const String &pass);
void   wifiForget();

/* Start Wi-Fi + serwera HTTP (wywołać raz w setup(), po init konsoli/SWS). */
void webuiSetup();

/* Obsługa serwera HTTP (wywoływać na początku loop()). */
void webuiLoop();
