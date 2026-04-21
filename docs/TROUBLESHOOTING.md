# Troubleshooting

## PN532 not detected

Check these first:

- PN532 module selector is set to `SPI`
- `VCC` is `3V3`
- `SCK`, `MISO`, `MOSI`, and `SS` wiring matches configured pins
- `SS` is wired to the configured GPIO and not left floating

## MQTT not working

Check:

- Wi-Fi is connected
- broker address and credentials are correct
- web log shows `MQTT connected`

If the web log shows `MQTT publish skipped (not connected)`, the reader is working but MQTT is offline.

## OTA update fails

Use the OTA file only:

- `ota/rfid-esp32s3-latest.bin`

If OTA activation still fails on a device with old flash history, use one full flash with:

- `web-installer/firmware/merged.bin`
