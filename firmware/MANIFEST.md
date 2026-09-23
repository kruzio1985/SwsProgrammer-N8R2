# Firmware — N8R2 build

Prebuilt, frozen image for the `ESP32-S3-DevKitC-1-N8R2`
(**8 MB (Quad SPI)** flash, **2 MB (Quad SPI)** PSRAM).

| File | Size | SHA-256 |
|---|---|---|
| `sws_programmer_esp32s3_n8r2_v1.2.bin` | 969616 B | `2D78AF8CD63C51F4A494570E2660714300923A867FE7F01D1634A750B0F6C176` |

- **target:** `ESP32-S3-DevKitC-1-N8R2` (PlatformIO env `esp32s3`)
- **pins (all functions):** SWS=42, RST=41, UART TX/RX=17/18, RS485=33/34/35,
  SD SCK/MISO/MOSI/CS=14/15/16/21, SPI flash SCK/MISO/MOSI/CS=12/13/11/10,
  I²C=8/9, 1-Wire=4, ESP bridge IO0/EN=5/6, console UART0 GPIO43/44
- **build:** `pio run -e esp32s3`
- **flash:** `pio run -e esp32s3 -t upload --upload-port COMx`
- **web UI:** SoftAP `SWS-Programmer` / `12345678` → `http://192.168.4.1`

## Verify

```powershell
Get-FileHash firmware\sws_programmer_esp32s3_n8r2_v1.2.bin -Algorithm SHA256
```