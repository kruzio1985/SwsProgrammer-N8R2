/* =============================================================================
 * sws_master.h - bit-bangowany master Telink SWire (SWS) dla ESP32-S3.
 *
 * Nie uzywamy floadera ani UART-u ukladu docelowego. Master sam generuje
 * przebieg SWS (open-drain) i sam go probkuje, wiec wystarcza 4 kable:
 * Vdd, GND, SWS, RST.
 *
 * Format bajtu na drucie (10 bitow SWS, wg Telink SWire spec + enkodera pvvx):
 *      [cmd][d7][d6][d5][d4][d3][d2][d1][d0][0]
 *   bit SWS 0 = 1 jednostka LOW + 4 jednostki HIGH
 *   bit SWS 1 = 4 jednostki LOW + 1 jednostka HIGH
 *   cmd = 1 tylko dla bajtu START (0x5A) i END (0xFF)
 *
 * Ramka zapisu : 5A a2 a1 a0 00 <dane...> FF
 * Ramka odczytu: 5A a2 a1 a0 80   (bit7 RW_ID = 1 -> odczyt)
 * =============================================================================
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

/* Inicjalizacja pinow SWS/RST (open-drain + pull-up, RST w stanie wysokim). */
void swsMasterInit();

/* Czas jednej "jednostki" SWS w mikrosekundach (domyslnie 4 us).
 * Ustawia takze dzielnik ukladu docelowego, zeby oba konce mialy to samo
 * tempo - patrz swsUnitToDiv. */
void swsSetUnitUs(uint32_t us);
uint32_t swsGetUnitUs();

/* Dzielnik predkosci SWS ukladu docelowego: jednostka = div / 32 MHz. */
bool swsSetTargetDiv(uint8_t div);

/* Dzielnik [0x00b2] odpowiadajacy jednostce czasu mastera (us * 32, 1..127).
 * Rejestr jest 7-bitowy - 0x80 zamaskowaloby sie do zera i zabilo lacze. */
uint8_t swsUnitToDiv(uint32_t us);

/* Punkt probki decyzyjnej bitu odczytu, w cwiartkach jednostki (domyslnie 10
 * czyli 2,5 jednostki). Slabe podciaganie opoznia zbocze, wiec bywa trzeba
 * przesunac probke pozniej. */
void swsSetSampleQuarter(uint32_t q);
uint32_t swsGetSampleQuarter();

/* Diagnostyka odczytu: szerokosci stanu LOW ostatniego bajtu, w cwiartkach
 * jednostki, mierzone OD PUNKTU PROBKI do zbocza narastajacego. Pelna
 * szerokosc = swsLastSampleQuarter() + szerokosc. Indeks 0 = bit 7. */
const uint32_t *swsLastBitWidths();
uint32_t swsLastSampleQuarter();

/* Surowy zrzut poziomu linii SWS po komendzie odczytu: jeden wpis na kazda
 * cwiartke jednostki (0 = LOW, 1 = HIGH). Bez zalozen o kodowaniu bitow. */
bool swsReadWave(uint32_t addr, uint8_t *levels, uint32_t nq);

/* Zapis/odczyt rejestrow ukladu przez SWS (adres 24-bitowy). */
bool swsWriteReg(uint32_t addr, const uint8_t *data, size_t n);
bool swsWriteReg8(uint32_t addr, uint8_t v);
bool swsReadReg(uint32_t addr, uint8_t *out, size_t n);

/* Dostep do rejestrow analogowych ukladu (TLSR8258) przez rejestry SWS:
 *   [0x00b8] = adres analogowy, [0x00b9] = dane, [0x00ba] = kontrolne.
 * Protokol wg pvvx/TLSRPGM (UART2SWire/Source/main.c): FLD_ANA_START=0x40,
 * na koncu dostepu zapis 0x00 do [0x00ba]. Odczyt nie modyfikuje zawartosci. */
bool swsAnalogRead(uint32_t addr, uint8_t *out);
bool swsAnalogWrite(uint32_t addr, uint8_t val);

/* Pojedynczy bajt END (0xFF) - zamyka sesje SWS. */
void swsEndFrame();

/* Sekwencja aktywacji z TlsrComProg.py: zatrzymanie CPU (0x0602 = 0x05),
 * reset, a nastepnie ustawienie dzielnika SWS. */
bool swsActivate(uint32_t holdMs);

/* Reset ukladu docelowego linia RST. */
void swsResetTargetPulse(uint32_t lowMs);

/* Diagnostyka: ostatni blad odczytu ("brak zbocza w oknie czasu"). */
uint32_t swsLastReadFailAddr();
