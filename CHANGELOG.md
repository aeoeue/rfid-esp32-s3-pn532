# Changelog

## v0.5.0

- Split reader handling into a PN532 reader abstraction
- Added guarded door/output control with pulse limits and lockout rules
- Added recent event buffering with MQTT replay after reconnect
- Expanded diagnostics for reader state, door state, Wi-Fi, heap, reset reason, and OTA partition state
- Cleaned up the web UI with labeled fields, tag action dropdowns, diagnostics, event view, and wiring help
- Extended backup/restore to include metadata validation and recent events
