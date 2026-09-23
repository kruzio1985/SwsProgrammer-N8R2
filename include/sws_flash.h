/* =============================================================================
 * sws_flash.h - dostep do flash ukladu docelowego po SWS (przez rejestry MSPI).
 *
 * To funkcjonalny odpowiednik dongle pvvx (TlsrPgm.py + uart2swire.bin), ale
 * zaimplementowany programowo na ESP32-S3. Nie potrzeba floadera w RAM ukladu
 * ani UART-u ukladu docelowego - wystarcza SWS + RST + zasilanie.
 *
 * Rejestry ukladu docelowego (TLSR825x):
 *   [0x000c] reg_mspi_data        - dane MSPI (auto-read po ustawieniu trybu)
 *   [0x000d] reg_master_spi_ctrl  - CS=bit0, SDO=bit1, CONT=bit2, RD=bit3
 *   [0x00b3] tryb FIFO/auto-inkrementacji adresu slave'a (bit7)
 * =============================================================================
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

#define SWS_FLASH_JEDEC_ID_CMD 0x9F
#define SWS_FLASH_READ_CMD     0x03
#define SWS_FLASH_WRITE_CMD    0x02
#define SWS_FLASH_WREN_CMD     0x06
#define SWS_FLASH_STATUS_CMD   0x05
#define SWS_FLASH_STATUS2_CMD  0x35
#define SWS_FLASH_STATUS3_CMD  0x15
#define SWS_FLASH_WRSR_CMD     0x01
#define SWS_FLASH_ERASE4K_CMD  0x20
#define SWS_FLASH_ERASE_ALL_CMD 0xC7
#define SWS_FLASH_PAGE         256u
#define SWS_FLASH_SECTOR       4096u
/* Maksymalny blok w jednej ramce SWS (FIFO slave'a obsluguje do 1024 B). */
#define SWS_SWS_BURST          512u

/* Bity rejestru kontrolnego MSPI [0x000d] (register.h B85). */
#define SWS_MSPI_CS            0x01u
#define SWS_MSPI_SDO           0x02u
#define SWS_MSPI_CONT          0x04u
#define SWS_MSPI_RD            0x08u
#define SWS_MSPI_BUSY          0x10u
/* Rejestr trybu auto-read: RD|SDO = 0x0a (dokladnie jak SDK Telink). */
#define SWS_MSPI_AUTO_READ     (SWS_MSPI_RD | SWS_MSPI_SDO)

/* Rozmiar flash ukladu TS0201 (TLSR825x 1M). */
#define SWS_FLASH_SIZE         (1024u * 1024u)

/* ID flash (np. c8 60 13 dla 1 MB). */
bool swsFlashJedecId(uint32_t *id);

/* Bit0 = WIP (zapis/erase w toku). */
bool swsFlashStatus(uint8_t *status);
bool swsFlashIsBusy(bool *busy);

/* Odczyt dowolnego obszaru flash (auto-read z rejestru MSPI). */
bool swsFlashRead(uint32_t addr, uint8_t *buf, uint32_t n);

/* Kasowanie sektora 4 kB. */
bool swsFlashEraseSector(uint32_t addr);

/* Zapis strony (max 256 B), adres musi byc wyrownany do strony. */
bool swsFlashWritePage(uint32_t addr, const uint8_t *buf, uint32_t n);

/* Zapis dowolnego obszaru z automatycznym kasowaniem sektorow.
 * eraseFirst=false zaklada, ze sektory sa juz skasowane. */
bool swsFlashWriteRange(uint32_t addr, const uint8_t *buf, uint32_t n, bool eraseFirst);

/* Kasowanie calego ukladu (dlugo trwa). */
bool swsFlashEraseAll();

/* =============================================================================
 * Diagnostyka niskopoziomowa MSPI.
 *
 * SDK Telink (bsp/b85/flash.c + b85/spi_i.h) kazda komende do kosci wysyla tak:
 *     mspi_high(); sleep_us(1); mspi_low(); mspi_write(cmd); mspi_wait();
 * gdzie mspi_wait() czeka na skasowanie FLD_MSPI_BUSY (bit4 rejestru [0x0d]).
 * Zwykla sciezka (swsFlashRead/...) nie czeka na BUSY, bo jedna ramka SWS jest
 * ~1000x dluzsza od bajtu MSPI. Funkcje ponizej wystawiaja oba mechanizmy,
 * zeby dalo sie sprawdzic, czy sprzet przyjmuje zapisy [0x0d] i czy komenda
 * kasowania/zapisu naprawde dochodzi do kosci (bit WEL i WIP).
 * =============================================================================
 */

/* Odczyt [0x000d] (rejestr kontrolny MSPI). */
bool swsMspiCtrlRead(uint8_t *v);

/* Zapis [0x000d] z natychmiastowym odczytem kontrolnym (readback != NULL).
 * Readback != zapisana wartosc = sprzet odrzucil zapis. */
bool swsMspiCtrlWrite(uint8_t v, uint8_t *readback);

/* Czeka, az BUSY (bit4 [0x0d]) sie skasuje. false = przekroczony timeout. */
bool swsMspiWaitBusy(uint32_t timeoutMs, uint32_t *waitedMs);

/* Czy przed zapisem bajtu do [0x0c] czekac na BUSY (jak mspi_wait w SDK). */
void swsSetBusyPoll(bool on);
bool swsGetBusyPoll();
uint32_t swsBusyTimeoutCount();

/* Surowa transakcja: CS w gore, ~2 us pauzy, CS w dol, kolejne bajty do
 * [0x0c], CS w gore (to zbocze wykonuje komende WREN/ERASE/PROGRAM).
 * ctrlLo/ctrlHi (moga byc NULL) dostaja odczyt [0x0d] po kazdym zboczu. */
bool swsFlashRawTx(const uint8_t *bytes, uint32_t n, uint8_t *ctrlLo, uint8_t *ctrlHi);

/* Surowa transakcja + odczyt no bajtow w trybie auto-read (0x0a). */
bool swsFlashRawTxRead(const uint8_t *bytes, uint32_t n, uint8_t *out, uint32_t no);

/* Odczyt n bajtow dowolnego rejestru statusu (0x05, 0x35, 0x15).
 * sdkStyle=true uzywa metody z SDK (bajt 0x00 = zegar, odczyt [0x0c] na bajt). */
bool swsFlashStatusCmd(uint8_t cmd, uint8_t *out, uint32_t n, bool sdkStyle);

/* WREN (0x06). */
bool swsFlashWren();

/* Zapis rejestru statusu: WREN + [0x01] + wartosc (zdjecie blokad BP/SRP). */
bool swsFlashWriteStatusReg(uint8_t val);
