#pragma once

/* =============================================================================
 * Konfiguracja pinów programatora SWS (Telink TLSR825x).
 *
 * Piny można zmieniać tutaj - nie ma żadnej innej konfiguracji.
 *
 *  ┌──────────────┐                     Twoja płytka (Tuya, złącze P2)
 *  │   ESP32-S3   │
 *  │              │
 *  │ 3V3      ────┼──── P2 pin 1  VCC   <-- 3,3 V, NIGDY 5 V
 *  │ GND      ────┼──── P2 pin 2  GND
 *  │ SWS_PIN  ────┼──[470R]── P2 pin 3  SWS   (GPIO_PA7 układu)
 *  │ RST_PIN  ────┼──[470R]── P2 pin 4  RST
 *  │ UART_TX  ────┼──── P2 pin 5  (TX układu)   [opcjonalne, patrz niżej]
 *  │ UART_RX  ────┼──── P2 pin 6  (RX układu)   [opcjonalne, patrz niżej]
 *  └──────────────┘
 *
 *  P2 (potwierdzone pomiarem: minus baterii = pin 2, plus baterii = pin 1):
 *      1=VCC  2=GND  3=SWS  4=RST
 *
 *  ⚠️ Tabelka z pvvx (1=RST 2=SWS 3=Vdd 4=GND) NIE pasuje do tego egzemplarza.
 *  Nie sugeruj się nią przy podłączaniu - trzymaj się powyższej.
 *
 *  Rezystory 470 Ω na SWS i RST są OBOWIĄZKOWE przy niepewnym okablowaniu:
 *  ograniczają prąd do ~7 mA, więc nawet podanie linii na Vdd nie robi zwarcia.
 *
 *  UART_TX / UART_RX: NIE są potrzebne do programowania - cały dostęp do
 *  pamięci flash idzie po SWS (przez rejestry MSPI układu). Zostawione dla
 *  trybu diagnostycznego / przyszłych zastosowań.
 * =============================================================================
 */

// SWS (jednoprzewodowy interfejs programowania) - linia danych.
#define SWS_PIN 42

// Reset modułu TLSR8258. Sterowany jako otwarty dren (tylko LOW / high-Z).
#define RST_PIN 41

// UART celu (czujnik Tuya / dowolne urządzenie 3,3 V TTL).
// UART_TX = wyjście ESP32 -> RX układu, UART_RX = wejście ESP32 -> TX układu.
// Piny STAŁE - nie dzielone z niczym innym.
#define UART_TX_PIN 17
#define UART_RX_PIN 18

/* =============================================================================
 * RS485 (półdupleks, przez transceiver MAX485/SN75176 - NIE bezpośrednio!).
 *
 * Osobne, stałe piny - niezależne od UART powyżej. Podłączenie:
 *   ESP32:   RS485_TX -> DI,  RS485_RX -> RO,  RS485_DE -> DE+RE (zwarte)
 *   magistrala RS485: A / B (+ GND wspólny dla wszystkich urządzeń)
 *
 * DE/RE = kierunek: HIGH = nadawanie, LOW = odbiór.
 * =============================================================================
 */
#define RS485_TX_PIN 33
#define RS485_RX_PIN 34
#define RS485_DE_PIN 35

/* =============================================================================
 * Karta microSD (modul Catalex, interfejs SPI) - OSOBNY, drugi kontroler SPI
 * (HSPI). Nie dzieli pinow ze SPI flash (FSPI, piny 12/13/11/10) ani z niczym
 * innym. Podlaczenie modulu Catalex:
 *
 *      ESP32-S3                  Modul Catalex microSD
 *      SD_SCK_PIN  (CLK)  ------> SCK
 *      SD_MISO_PIN (MISO) <------ DO
 *      SD_MOSI_PIN (MOSI) ------> DI
 *      SD_CS_PIN   (CS)   ------> CS
 *      5V                  ------> VCC (modul ma AMS1117 + bufor 74ABT125,
 *                                    mozna tez zasilic z 3V3)
 *      GND                 ------> GND
 * =============================================================================
 */
#define SD_SCK_PIN   14
#define SD_MISO_PIN  15
#define SD_MOSI_PIN  16
#define SD_CS_PIN    21

/* =============================================================================
 * SPI (zewnętrzna kość flash 25xx / klips SOIC-8).
 *
 * Piny to domyślny port FSPI ESP32-S3 (SPI = FSPI). Podłączasz je wprost do
 * kości SPI NOR (wylutowanej albo przez klips SOIC-8) - bez adaptera.
 *
 *   ESP32-S3:   SCK=12  MISO=13  MOSI=11  CS=10
 *   SOIC-8:     6 CLK   2 DO     5 DI     1 CS
 *   + 3V3 -> pin 8 VCC,  GND -> pin 4 GND
 *
 * Prędkość 2 MHz jest bezpieczna przy dłuższych przewodach od klipsa.
 * =============================================================================
 */
#define SPI_SCK_PIN   12
#define SPI_MISO_PIN  13
#define SPI_MOSI_PIN  11
#define SPI_CS_PIN    10
#define SPI_SPEED_HZ  2000000

/* =============================================================================
 * WiFi / interfejs WWW.
 *
 * Po starcie programator tworzy sieć Wi-Fi (SoftAP) - połącz się telefonem lub
 * komputerem i otwórz http://192.168.4.1 w przeglądarce.
 *
 * Opcjonalnie można podłączyć programator do domowej sieci (WEB_STA_SSID różne
 * od "") - wtedy ESP dołączy do niej i wypisze adres IP na konsoli COM11.
 * =============================================================================
 */
#define WEB_AP_SSID     "SWS-Programmer"
#define WEB_AP_PASSWORD "12345678"

#define WEB_STA_SSID    ""
#define WEB_STA_PASSWORD ""

/* Górny limit rozmiaru obrazu .bin wgrywanego przez WWW (w bajtach).
 * To tylko górny limit - rzeczywista wartość jest wyliczana w locie na
 * podstawie rozmiaru PSRAM (webui.cpp), więc N8R2 (2 MB PSRAM) i N16R8
 * (8 MB PSRAM) automatycznie dostają właściwe limity. Firmware TLSR825x
 * mieści się w 1 MB flasha docelowego. */
#define WEB_MAX_FW      (1024u * 1024u)

/* Górny limit pojedynczego wgrania SPI przez WWW (bajty). Bufor jest
 * alokowany w PSRAM; rzeczywisty limit = min(ten limit, wolne PSRAM -
 * margines na SD / przechwytywanie / oscyloskop). */
#define SPI_WEB_MAX     (8u * 1024u * 1024u)

/* =============================================================================
 * Porty szeregowe:
 *  - CONSOLE: Serial  (UART0, GPIO43/44) -> konwerter CH343 -> COM11
 *    (nie ustawiamy USB CDC on boot, więc Serial = UART0)
 *  - TARGET:  Serial1 (UART1) - UART docelowego układu (TX=17, RX=18).
 *  - RS485:   Serial2 (UART2) - TX=33 (DI), RX=34 (RO), DE=35 (kierunek).
 * =============================================================================
 */

/* =============================================================================
 * One-Wire (1-Wire) — np. czujniki DS18B20. Jeden pin, rezystor pull-up 4,7k
 * między linią danych a 3V3 (tak jak w gotowych sondach DS18B20).
 * =============================================================================
 */
#define ONE_WIRE_PIN 4

/* =============================================================================
 * I²C — skan magistrali / odczyt rejestrów obcych układów (EEPROM, czujniki,
 * RTC itd.). Piny wolne (8/9 to domyślny I²C ESP32-S3). Pull-up 4,7k na SDA/SCL.
 * =============================================================================
 */
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

/* =============================================================================
 * Programowanie obcych ESP (ESP8266 / ESP32 / ESP32-S3 / ESP32-C3...) przez
 * UART + esptool — tzw. "wireless flashing". Używamy tego samego UART celu
 * (TX=17 -> RX układu, RX=18 -> TX układu) plus dwóch linii sterujących:
 *
 *   ESP_IO0_PIN -> GPIO0 układu docelowego (BOOT: LOW = tryb download)
 *   ESP_EN_PIN  -> EN / RESET układu docelowego (impuls LOW->HIGH = reset)
 *
 * ESP32-S3 otwiera serwer TCP na porcie ESP_BRIDGE_PORT; z komputera łączysz
 * się:  esptool.py --port socket://<ip>:<port> flash_id / write_flash ...
 * Po podłączeniu klienta programator sam wciska układ w tryb bootloadera.
 * =============================================================================
 */
#define ESP_IO0_PIN 5
#define ESP_EN_PIN  6
#define ESP_BRIDGE_PORT 3232

/* =============================================================================
 * Wolne GPIO do ręcznego sterowania (panel GPIO w WWW). Nie pokrywają się
 * z żadną z powyższych funkcji. GPIO 45/46/0/3 pominięte (piny strapping).
 * =============================================================================
 */
