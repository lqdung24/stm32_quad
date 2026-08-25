# ESP32-S3 + MFRC522 card reader/copy tool

Standalone ESP-IDF firmware for an ESP32-S3-WROOM-1-N16R8. It reads the UID
and accessible MIFARE Classic data blocks, keeps one snapshot in RAM, and can
write/verify those data blocks on another card through commands received on the
ESP32-S3 native USB Serial/JTAG port.

## Wiring

| RC522 pin | ESP32-S3 | Notes |
|---|---:|---|
| 3.3V | 3V3 | Never connect the RC522 to 5 V |
| GND | GND | Common ground |
| SCK | GPIO10 | SPI2 clock |
| MOSI | GPIO11 | SPI2 controller to RC522 |
| SDA / SS | GPIO12 | SPI chip select; this is not I2C SDA |
| MISO | GPIO13 | RC522 to SPI2 controller |
| RST | GPIO14 | Reset |
| IRQ | not connected | Polling is used |

GPIO19/20 are deliberately left free for native USB D-/D+. GPIO0, GPIO3,
GPIO45, and GPIO46 are avoided because they are strapping pins.

## Build, flash, and connect

Use ESP-IDF v5.5 or newer:

```sh
idf.py -C rfid_s3 build
idf.py -C rfid_s3 -p /dev/ttyACM0 flash
idf.py -C rfid_s3 -p /dev/ttyACM0 monitor
```

Do not run a second serial monitor while issuing commands. Any serial terminal
can use the same native USB Serial/JTAG port. The baud-rate setting is ignored
by native USB.

## Commands

- `scan` reads UID, SAK, and ATQA.
- `copy` reads all accessible data blocks into RAM.
- `dump` prints the snapshot.
- `write` writes every captured data block to a same-size target card, reads it
  back, and reports the verification result.
- `key A all FFFFFFFFFFFF` sets an authentication key for every sector.
- `key B 7 A0A1A2A3A4A5` sets Key B for sector 7.
- `clear` erases the snapshot and resets all keys to `FFFFFFFFFFFF`.

Only MIFARE Classic Mini, 1K, and 4K geometry is supported. The firmware does
not guess keys. Block 0 and every sector trailer are intentionally excluded
from writes so a bad dump cannot overwrite access conditions or brick a card.
UIDs on normal MIFARE Classic cards are factory-programmed and cannot be
changed; `write` reports whether the target UID differs and still copies data.
The `copy` and `write` commands wait up to 10 seconds for a card.

Use this tool only with cards and systems you own or are authorized to test.
