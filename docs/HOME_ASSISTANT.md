# Home Assistant

Home Assistant integration is based on MQTT discovery.

## Prerequisites

- MQTT broker available on the local network
- MQTT integration enabled in Home Assistant
- Device configured with MQTT host, port, and credentials in the web UI
- `Home Assistant Discovery` enabled in the device config

## Entities

The firmware publishes discovery data for:

- last UID
- last name
- last event
- tag count
- Wi-Fi RSSI
- free heap
- reader state
- door status
- learning switch
- learn-next button
- reboot button
- output switch
- output pulse button

## Base Topic

- `<topic_prefix>/<deviceName>`

Default:

- `rfid/rfid-esp32`

## Commands

Publish JSON to:

- `<base>/cmd`

Examples:

```json
{"action":"learn_next","name":"Alice"}
```

```json
{"action":"set_learning","enabled":true,"next_only":false,"name":""}
```

```json
{"action":"set_tag_state","uid":"A1B2C3D4","state":"blocked"}
```

```json
{"action":"set_tag_action","uid":"A1B2C3D4","tag_action":"pulse"}
```

```json
{"action":"pulse_output"}
```
