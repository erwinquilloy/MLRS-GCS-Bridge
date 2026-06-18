# MLRS-GCS-Bridge

Heltec WiFi Kit 32 firmware that bridges an mLRS Nomad to an
**MFD Mini Crossbow** OSD over UART, with live telemetry on the onboard
SSD1306 OLED. Two link-layer variants are provided in this repo:

| Sketch folder | Link to Nomad | When to use |
| ------------- | ------------- | ----------- |
| `mlrs_espnow_gcs_heltec_v2/` | ESP-NOW (raw 802.11) | Longest range, lowest latency, point-to-point |
| `mlrs_udp_gcs_heltec_v2/` | WiFi UDP (joins Nomad SoftAP) | Lets phone/laptop GCSs co-exist on the same AP |

```
   mLRS Nomad ──ESP-NOW or WiFi UDP──▶ Heltec WiFi Kit 32 ──UART──▶ MFD Mini Crossbow
                                              │
                                              └── SSD1306 OLED (telemetry HUD)
```

## Features (common to both variants)

- Bidirectional MAVLink bridge (downlink + uplink).
- Built-in MAVLink v1/v2 parser (no MAVLink library dependency).
- Decoded telemetry on OLED: flight mode, arm state, GPS fix/sats, lat/lon,
  altitude, ground speed, heading, battery V/%, and RSSI %.
- ArduPilot / iNav firmware autodetection from the HEARTBEAT autopilot field
  (mode names are mapped accordingly).
- Link status indicator (`LINK OK` / `NO DATA` / `waiting`) based on packet
  recency.

## ESP-NOW variant specifics

- Channel auto-scan (1 / 6 / 11 / 13) until an mLRS bridge is found.
- Sender MAC latching so only the paired bridge is accepted.
- WiFi country locked to `EU` (channels 1-13), radio forced to `802.11b` for
  maximum range.

## WiFi UDP variant specifics

- Heltec joins the Nomad's SoftAP (default SSID `mLRS AP`, password
  `thisisgreat`).
- Listens on UDP port `14550` and latches the first peer's IP/port for uplink.
- Multiple GCS clients (e.g. a phone running QGC + this bridge) can attach to
  the same Nomad AP simultaneously. Be aware that if more than one client
  sends commands, the uplink streams interleave at the radio - it's a
  coordination problem, not a connectivity one.
- Auto-reconnects if the AP drops.

## Hardware

| Item | Notes |
| ---- | ----- |
| Heltec WiFi Kit 32 | v1 or v2 (micro-USB variants); both share the same OLED pinout used here |
| MFD Mini Crossbow OSD | MAVLink input @ 115200 8N1 |
| mLRS Nomad | Configured as the matching bridge counterpart (see below) |

### Wiring

| Heltec pin | Connects to |
| ---------- | ----------- |
| `GPIO17` (TX2) | Crossbow MAVLink input |
| `GND`          | Crossbow GND |
| micro-USB      | 5 V power |

UART RX is intentionally disabled (`-1`) so that **GPIO16** stays free as the
OLED reset line.

## mLRS Nomad (Tx) setup

The Nomad's WiFi Bridge mode **must match the sketch you flash**:

| Sketch you flash | Nomad WiFi Bridge mode |
| ---------------- | ---------------------- |
| `mlrs_espnow_gcs_heltec_v2` | `ESP-NOW` |
| `mlrs_udp_gcs_heltec_v2`    | `WiFi UDP` |

Quick path via the OLED menu:

```
Setup -> WiFi Bridge -> Mode -> <ESP-NOW | WiFi UDP>
```

After changing the mode, reboot the Nomad so the new radio mode takes effect.

## Build & flash

### Arduino IDE

1. Install the **ESP32 Arduino core >= 3.0.0** via Boards Manager.
2. Install these libraries via Library Manager:
   - Adafruit SSD1306
   - Adafruit GFX Library
3. Board: **ESP32 Arduino -> Heltec WiFi Kit 32**.
4. Open the `.ino` for the variant you want (`mlrs_espnow_gcs_heltec_v2/...`
   or `mlrs_udp_gcs_heltec_v2/...`), then Upload.

### Configurable defaults

Set at the top of each `.ino`:

| Define | Default | Purpose |
| ------ | ------- | ------- |
| `CROSSBOW_BAUD` | `115200` | UART baud to the Crossbow |
| `TX_PIN` | `17` | UART TX pin |
| `OLED_UPDATE_MS` | `200` | OLED refresh interval |
| `OLED_SDA` / `OLED_SCL` / `OLED_RST` | `4` / `15` / `16` | OLED I2C pins (Heltec defaults) |
| `OLED_ADDR` | `0x3C` | SSD1306 I2C address |

UDP variant adds:

| Define | Default | Purpose |
| ------ | ------- | ------- |
| `WIFI_SSID` | `"mLRS AP"` | Nomad SoftAP SSID |
| `WIFI_PASS` | `"thisisgreat"` | Nomad SoftAP password |
| `UDP_PORT` | `14550` | MAVLink UDP port |

## Supported MAVLink messages

| Msg ID | Name | Fields used |
| ------ | ---- | ----------- |
| 0  | `HEARTBEAT`           | `type`, `autopilot`, `base_mode`, `custom_mode` |
| 1  | `SYS_STATUS`          | `voltage_battery`, `battery_remaining` |
| 24 | `GPS_RAW_INT`         | `fix_type`, `satellites_visible` |
| 33 | `GLOBAL_POSITION_INT` | `lat`, `lon`, `alt`, `vx`, `vy`, `hdg` |
| 109 | `RADIO_STATUS`       | `rssi` (treated as 0-100% per mLRS, not 0-254 SiK) |

All other messages are still forwarded over UART -- only these are decoded for
the on-board OLED.

## License

GPL v3.
