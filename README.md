# ESP32-S3 PN532 Access Controller

! AI-assisted, human-tested !

Firmware for an `ESP32-S3-WROOM-1` with a `PN532 NFC RFID V3` module in `SPI` mode.

It reads NFC tags, publishes events to MQTT, integrates with Home Assistant through MQTT discovery, and provides a built-in web UI for setup, diagnostics, tag management, backup/restore, and OTA updates.

## Features

- PN532 `SPI` reader support
- Tag learning and tag management
- Tag states: `allowed`, `blocked`, `disabled`
- Per-tag actions: `none`, `pulse`, `toggle`
- Output pin control with pulse limits and lockout protection
- MQTT integration with Home Assistant discovery
- Web UI for setup, status, diagnostics, logs, backup/restore, and OTA
- Recent event ring buffer for short MQTT outages
- Captive portal fallback AP and mDNS
- Browser-based flashing with `esp-web-tools`

## Hardware

- ESP32-S3 board using the `esp32-s3-devkitc-1` PlatformIO profile
- PN532 NFC RFID V3 module
- USB data cable
- Optional relay or output driver for door control

## PN532 Wiring

Default pin mapping:

- `GPIO12` -> `PN532 SCK`
- `GPIO13` -> `PN532 MISO`
- `GPIO11` -> `PN532 MOSI`
- `GPIO10` -> `PN532 SS / NSS / SDA`
- `3V3` -> `PN532 VCC`
- `GND` -> `PN532 GND`

`RST / RSTPDN` is optional and disabled by default in this release.

## Build

```bash
./tools/bootstrap.sh
./tools/pio.sh run -e esp32s3
```

USB serial upload:

```bash
./tools/pio.sh run -e esp32s3 -t upload
```

## Browser Flashing

Serve this folder locally:

```bash
python3 -m http.server 8000
```

Open:

- `http://localhost:8000/web-installer/`

WebSerial requires `localhost` or HTTPS on the same machine that has the ESP connected by USB.

## OTA Files

- `ota/rfid-esp32s3-latest.bin`
- `ota/rfid-esp32s3-0.5.0-20260421-132735.bin`

Full flash image:

- `web-installer/firmware/merged.bin`

## Layout

- `src/`: firmware sources
- `include/`: local headers
- `tools/`: build helpers
- `web-installer/`: browser flashing page and full-flash binaries
- `ota/`: OTA binaries
- `docs/`: integration and publishing notes

## License

This project is released under the MIT License.
