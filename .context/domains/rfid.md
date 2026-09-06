# Domain: RFID utility

`esp_controller/rfid_s3` is a standalone ESP32-S3 + MFRC522 USB CLI. It is not part of the flight-control link.

## Interfaces

- USB commands: `scan`, `read <block>`, `write <block> <32 hex>`, `key <A|B> <all|0..39> <12 hex>`, `clear`.
- RC522 transport: SPI2 with pins defined in `main/main.c`.
- Card family: operations require MIFARE Classic compatibility.
- Keys: per-sector Key A/B values are held only in RAM; default/reset is Key A `FFFFFFFFFFFF`.

## Concurrency contract

- `s_picc_lock` protects the active library-owned PICC pointer.
- `s_card_seen` is a binary arrival notification with stale notifications drained on removal.
- `s_rc522_io_lock` is shared with the RC522 polling task. CLI authentication/read/write must hold it so polling cannot touch the device concurrently.

## Read/write flow

Parse arguments → wait up to 10 s for a card → acquire hardware mutex up to 2 s → recheck card → resolve sector → authenticate → optional write → read back → verify bytes → deauthenticate → release mutex.

## Important risk

The intended checks rejecting manufacturer block 0 and sector trailers in `data_block()` are currently commented out. Therefore the implementation can reach sensitive blocks despite comments saying only ordinary data blocks are allowed. Treat writes as potentially destructive until those checks are restored and tested. Never test writes on an irreplaceable card.

See `esp_controller/rfid_s3/function-flow.md` for per-function behavior.
