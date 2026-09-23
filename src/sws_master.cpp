/* =============================================================================
 * sws_master.cpp - bit-bangowany master Telink SWire (SWS) dla ESP32-S3.
 *
 * Idea: pvvx robi to sprzetowym SWM w TLSR8253 i "sztuczka UART" na CH340.
 * My nie mamy ani jednego, ani drugiego, wiec generujemy SWS programowo,
 * bezposrednio na rejestrach GPIO (open-drain + wewnetrzny pull-up).
 *
 * Zrodla (patrz README.md - atrybucja):
 *   - TelinkSWire/README.md      : format bitu, bajtu, ramki i protokol odczytu
 *   - TlsrComProg.py             : sekwencja aktywacji + dzielnik 0x00b2
 *   - UART2SWire/Source/main.c   : dostep do flash przez rejestry MSPI 0x0c/0x0d
 * =============================================================================
 */

#include "sws_master.h"

#include <Arduino.h>
#include <xtensa/hal.h>

#include "config.h"
#include "soc/soc.h"

#ifndef SWS_PIN
#error "SWS_PIN nie zdefiniowany w config.h"
#endif
#ifndef RST_PIN
#error "RST_PIN nie zdefiniowany w config.h"
#endif

/* Rejestry GPIO ESP32-S3 (bezposrednie, bez narzutu digitalWrite).
 * UWAGA: ESP32-S3 ma OSOBNE rejestry dla GPIO0..31 i GPIO32..48. */
#define S3_GPIO_OUT_W1TS  0x60004008u
#define S3_GPIO_OUT_W1TC  0x6000400Cu
#define S3_GPIO_ENA_W1TS  0x60004024u
#define S3_GPIO_ENA_W1TC  0x60004028u
#define S3_GPIO_IN        0x6000403Cu
#define S3_GPIO_OUT1_W1TS 0x60004014u
#define S3_GPIO_OUT1_W1TC 0x60004018u
#define S3_GPIO_ENA1_W1TS 0x60004030u
#define S3_GPIO_ENA1_W1TC 0x60004034u
#define S3_GPIO_IN1       0x60004040u

#define SWS_BIT   (SWS_PIN & 31)
#define SWS_MASK  (1u << SWS_BIT)
#define SWS_ENA_SET  (SWS_PIN >= 32 ? S3_GPIO_ENA1_W1TS : S3_GPIO_ENA_W1TS)
#define SWS_ENA_CLR  (SWS_PIN >= 32 ? S3_GPIO_ENA1_W1TC : S3_GPIO_ENA_W1TC)
#define SWS_OUT_CLR  (SWS_PIN >= 32 ? S3_GPIO_OUT1_W1TC : S3_GPIO_OUT_W1TC)
#define SWS_IN       (SWS_PIN >= 32 ? S3_GPIO_IN1 : S3_GPIO_IN)

static uint32_t s_unitCyc   = 960;  /* 4 us @ 240 MHz */
static uint32_t s_unitUs    = 4;
static uint32_t s_failAddr  = 0xFFFFFFFFu;

/* Punkt probki decyzyjnej w cwiartkach jednostki (10 = 2,5 jednostki).
 * Przy slabym podciaganiu zbocze narasta wolno, wiec optimum bywa pozniej niz
 * polowa bitu - strojone komenda PHASE. */
static uint32_t s_sampleQ   = 10;

/* Szerokosci stanu LOW ostatnio odczytanego bajtu, mierzone od punktu probki
 * do zbocza narastajacego (indeks 0 = bit 7). Diagnostyka komenda WIDTHS. */
static uint32_t s_widths[8];
static uint32_t s_sampleQUsed;

/* --- czas ----------------------------------------------------------------- */
static inline uint32_t IRAM_ATTR cyc(void) { return (uint32_t)xthal_get_ccount(); }

static inline void IRAM_ATTR spin(uint32_t c) {
    const uint32_t t0 = cyc();
    while ((uint32_t)(cyc() - t0) < c) {
    }
}

/* --- linia SWS (open-drain: latch=0, przelaczamy tylko output-enable) ------ */
static inline void IRAM_ATTR swsLow(void)   { REG_WRITE(SWS_ENA_SET, SWS_MASK); }
static inline void IRAM_ATTR swsRelease(void) { REG_WRITE(SWS_ENA_CLR, SWS_MASK); }
static inline int  IRAM_ATTR swsLevel(void) { return (int)((REG_READ(SWS_IN) >> SWS_BIT) & 1u); }

/* --- czekanie na zbocze (z limitem czasu) --------------------------------- */
static inline bool IRAM_ATTR waitLow(uint32_t maxCyc) {
    const uint32_t t0 = cyc();
    while (swsLevel()) {
        if ((uint32_t)(cyc() - t0) > maxCyc) return false;
    }
    return true;
}

static inline bool IRAM_ATTR waitHigh(uint32_t maxCyc) {
    const uint32_t t0 = cyc();
    while (!swsLevel()) {
        if ((uint32_t)(cyc() - t0) > maxCyc) return false;
    }
    return true;
}

/* --- pojedynczy bit SWS ---------------------------------------------------- */
static inline void IRAM_ATTR swsBit(int b) {
    if (b) {
        swsLow();     spin(s_unitCyc * 4);
        swsRelease(); spin(s_unitCyc);
    } else {
        swsLow();     spin(s_unitCyc);
        swsRelease(); spin(s_unitCyc * 4);
    }
}

/* 10 bitow SWS = [cmd][d7..d0][0] */
static void IRAM_ATTR swsByte(uint8_t v, int cmd) {
    swsBit(cmd);
    for (int i = 7; i >= 0; --i) swsBit((v >> i) & 1);
    swsBit(0);
}

/* Pojedyncza jednostka LOW + zwolnienie - tym master "zadaje pytanie"
 * przy odczycie (patrz punkt 5 protokolu czytania w TelinkSWire/README.md). */
static inline void IRAM_ATTR swsReadTrigger(void) {
    swsLow(); spin(s_unitCyc); swsRelease(); spin(s_unitCyc);
}

/* --- naglowek ramki: START + adres 24-bit + RW_ID ------------------------- */
static void IRAM_ATTR swsHeader(uint32_t addr, uint8_t rwId) {
    swsByte(0x5A, 1);                       /* START: cmd = 1 */
    swsByte((uint8_t)(addr >> 16), 0);
    swsByte((uint8_t)(addr >> 8), 0);
    swsByte((uint8_t)addr, 0);
    swsByte(rwId, 0);
}

/* --- inicjalizacja --------------------------------------------------------- */
void swsMasterInit() {
    pinMode(SWS_PIN, INPUT_PULLUP);          /* idle = HIGH (pull-up) */
    /* RST zostaje w wysokiej impedancji dopoki nie wydamy komendy ktora go
       celowo steruje. Tak jest bezpiecznie nawet przy pomylonym okablowaniu -
       nie podajemy napiecia na nieznany pin czujnika. */
    pinMode(RST_PIN, INPUT);

    REG_WRITE(SWS_OUT_CLR, SWS_MASK);        /* latch = 0 -> "0" znaczy LOW */
    REG_WRITE(SWS_ENA_CLR, SWS_MASK);        /* wysoka impedancja */

    swsSetUnitUs(s_unitUs);
}

void swsSetUnitUs(uint32_t us) {
    if (!us) us = 1;
    s_unitUs  = us;
    s_unitCyc = getCpuFrequencyMhz() * us;   /* us * MHz = cykle */
}

uint32_t swsGetUnitUs() { return s_unitUs; }

void swsSetSampleQuarter(uint32_t q) {
    if (q < 1) q = 1;
    if (q > 60) q = 60;
    s_sampleQ = q;
}
uint32_t swsGetSampleQuarter() { return s_sampleQ; }

const uint32_t *swsLastBitWidths() { return s_widths; }
uint32_t swsLastSampleQuarter() { return s_sampleQUsed; }
uint32_t swsLastReadFailAddr() { return s_failAddr; }

/* --- zapis ----------------------------------------------------------------- */
bool swsWriteReg(uint32_t addr, const uint8_t *data, size_t n) {
    swsHeader(addr, 0x00);                   /* RW_ID bit7 = 0 -> zapis */
    for (size_t i = 0; i < n; i++) swsByte(data[i], 0);
    swsEndFrame();
    return true;
}

bool swsWriteReg8(uint32_t addr, uint8_t v) { return swsWriteReg(addr, &v, 1); }

void swsEndFrame() { swsByte(0xFF, 1); }     /* END: cmd = 1 */

/* --- odczyt ----------------------------------------------------------------
 * Slave odpowiada 8 bitami + jedna jednostka LOW po kazdym "pytaniu".
 * Probkujemy w polowie bitu: LOW w polowie bitu => bit = 1, HIGH => bit = 0.
 * Pomiedzy bitami czekamy na zbocza, wiec dryf zegara sie nie kumuluje.
 */
static bool IRAM_ATTR swsReadByte(uint8_t *out, bool edgeVisible) {
    if (!edgeVisible && !waitLow(s_unitCyc * 200)) return false;

    uint8_t v = 0;
    for (int i = 7; i >= 0; --i) {
        /* Probka decyzyjna: LOW w punkcie s_sampleQ/4 jednostki => bit = 1.
         * Slave trzyma LOW krotko (bit 0) albo dlugo (bit 1), a zbocze narasta
         * wolno przez slabe podciaganie, wiec punkt probki jest strojony. */
        spin((s_unitCyc * s_sampleQ) / 4);
        v = (uint8_t)((v << 1) | (swsLevel() ? 0 : 1));

        /* Do zbocza narastajacego - razem z punktem probki daje pelna
         * szerokosc LOW (diagnostyka, w cwiartkach jednostki). */
        const uint32_t t0 = cyc();
        if (!waitHigh(s_unitCyc * 12)) return false;
        s_widths[i]   = (uint32_t)(((cyc() - t0) * 4u) / s_unitCyc);
        s_sampleQUsed = s_sampleQ;

        if (i && !waitLow(s_unitCyc * 40)) return false;  /* poczatek kolejnego */
    }
    *out = v;
    return true;
}

bool swsReadReg(uint32_t addr, uint8_t *out, size_t n) {
    s_failAddr = 0xFFFFFFFFu;

    swsHeader(addr, 0x80);                   /* RW_ID bit7 = 1 -> odczyt */

    for (size_t i = 0; i < n; i++) {
        bool edgeVisible = false;

        if (i == 0) {
            /* Ostatni bit bajtu RW_ID to juz 1 jednostka LOW + 4 HIGH, czyli
             * spelnia warunek startu transmisji slave'a. Sprawdzamy, czy slave
             * zaczal nadawac; jesli nie - wysylamy jawne "pytanie". */
            if (!waitLow(s_unitCyc * 60)) {
                swsReadTrigger();
            } else {
                edgeVisible = true;
            }
        } else {
            swsReadTrigger();
        }

        if (!swsReadByte(&out[i], edgeVisible)) {
            s_failAddr = addr + (uint32_t)i;
            swsEndFrame();
            return false;
        }

        /* Terminator bajtu: jedna jednostka LOW, potem zwolnienie szyny. */
        waitHigh(s_unitCyc * 12);
        if (waitLow(s_unitCyc * 20)) waitHigh(s_unitCyc * 20);
    }

    swsEndFrame();
    return true;
}

/* --- rejestry analogowe (TLSR8258) ----------------------------------------- */
bool swsAnalogRead(uint32_t addr, uint8_t *out) {
    if (!out) return false;
    uint8_t w[3];
    w[0] = (uint8_t)addr;                  /* [0xb8] adres analogowy  */
    w[1] = 0x00;                           /* [0xb9] dane (nieistotne) */
    w[2] = 0x40;                           /* [0xba] FLD_ANA_START     */
    if (!swsWriteReg(0x00b8, w, 3)) return false;

    uint8_t r[2];
    if (!swsReadReg(0x00b9, r, 2)) return false;

    uint8_t end = 0x00;                    /* zakonczenie dostepu      */
    swsWriteReg(0x00ba, &end, 1);

    *out = r[0];                           /* [0xb9] = dane            */
    return true;
}

bool swsAnalogWrite(uint32_t addr, uint8_t val) {
    uint8_t w[3];
    w[0] = (uint8_t)addr;                  /* [0xb8] adres analogowy   */
    w[1] = val;                            /* [0xb9] dane              */
    w[2] = 0x40;                           /* [0xba] FLD_ANA_START     */
    if (!swsWriteReg(0x00b8, w, 3)) return false;

    uint8_t end = 0x00;                    /* zakonczenie dostepu      */
    swsWriteReg(0x00ba, &end, 1);
    return true;
}

/* Surowy zrzut poziomu linii po komendzie odczytu: jeden bajt na kazda
 * cwiartke jednostki. Nie zaklada nic o kodowaniu bitow - sluzy do zmierzenia
 * rzeczywistego ksztaltu odpowiedzi slave'a (szerokosci impulsow i okres).
 * Probkuje z korekcja dryfu, wiec pozycja w tablicy = dokladny czas. */
bool swsReadWave(uint32_t addr, uint8_t *levels, uint32_t nq) {
    swsHeader(addr, 0x80);                   /* RW_ID bit7 = 1 -> odczyt */

    if (!waitLow(s_unitCyc * 60)) swsReadTrigger();   /* slave startuje sam */

    uint32_t next = cyc() + s_unitCyc / 4;
    for (uint32_t k = 0; k < nq; k++) {
        levels[k] = swsLevel() ? 1 : 0;
        while ((int32_t)(cyc() - next) < 0) {
        }
        next += s_unitCyc / 4;
    }

    swsEndFrame();
    return true;
}

/* Dzielnik SWS ukladu docelowego odpowiadajacy jednostce czasu mastera.
 * 0x00b2 = liczba taktow 32 MHz na jedna jednostke, wiec div = us * 32.
 * Rejestr jest 7-bitowy (pvvx w TlsrComProg.py obcina do 127), a 0x80
 * zamaskowaloby sie do zera i calkowicie zabilo lacze - dlatego gorna granica
 * to 127, nie 255. Master i uklad docelowy MUSZA miec to samo tempo. */
uint8_t swsUnitToDiv(uint32_t us) {
    uint32_t d = us * 32u;
    if (d < 1) d = 1;
    if (d > 127) d = 127;
    return (uint8_t)d;
}

/* 0x00b2 = liczba taktow 32 MHz na jedna jednostke SWS (pvvx: 32e6/baud).
 */
bool swsSetTargetDiv(uint8_t div) { return swsWriteReg8(0x00B2, div); }

/* --- reset / aktywacja -----------------------------------------------------
 * RST dziala jak linia otwartego drenu: albo sciagamy ja do zera, albo
 * puszczamy (wysoka impedancja). Nigdy nie podajemy 3,3 V na pin czujnika.
 * Funkcjonalnie to to samo (modul ma wlasny rezystor podciagajacy na RESET),
 * a przy pomylonym okablowaniu nie zwiera wyjscia GPIO do masy. */
static inline void rstLow() {
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, LOW);
}

static inline void rstRelease() { pinMode(RST_PIN, INPUT); }

void swsResetTargetPulse(uint32_t lowMs) {
    rstLow();
    delay(lowMs);
    rstRelease();
}

bool swsActivate(uint32_t holdMs) {
    rstRelease();

    /* 1) END + zatrzymanie CPU: [0x0602] = 0x05 */
    swsEndFrame();
    delay(2);
    swsWriteReg8(0x0602, 0x05);

    /* 2) Reset w trakcie powtarzanego blokowania CPU (jak TlsrComProg.py). */
    rstLow();
    delay(50);
    for (int i = 0; i < 5; i++) swsWriteReg8(0x0602, 0x05);
    rstRelease();
    for (int i = 0; i < 5; i++) swsWriteReg8(0x0602, 0x05);

    /* 3) Podtrzymanie zatrzymania CPU przez holdMs. */
    const uint32_t deadline = millis() + holdMs;
    while ((int32_t)(millis() - deadline) < 0) swsWriteReg8(0x0602, 0x05);

    /* 4) Dzielnik predkosci SWS slave'a dobrany do tempa mastera. */
    return swsSetTargetDiv(swsUnitToDiv(s_unitUs));
}
