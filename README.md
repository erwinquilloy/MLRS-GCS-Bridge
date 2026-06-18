# MLRS-GCS-Bridge

Heltec WiFi Kit 32 firmware that bridges an mLRS Nomad to an
**MFD Mini Crossbow** OSD over UART, with live telemetry on the onboard
SSD1306 OLED. Three sketches are provided:

| Sketch folder | Link to Nomad | When to use |
| ------------- | ------------- | ----------- |
| `mlrs_gcs_heltec_v2/` (recommended) | **Either** - selectable at runtime via the PRG button | Day-to-day; flash once and toggle without re-flashing |
| `mlrs_espnow_gcs_heltec_v2/` | ESP-NOW (raw 802.11) | Reference sketch for the ESP-NOW link layer alone |
| `mlrs_udp_gcs_heltec_v2/` | WiFi UDP (joins Nomad SoftAP) | Reference sketch for the WiFi-UDP link layer alone |
| `mlrs_espnow_gcs_esp32_v2/` | ESP-NOW (raw 802.11) | Generic ESP32 (no OLED); reference sketch for the ESP-NOW link layer alone |
| `mlrs_udp_gcs_esp32_v2/`    | WiFi UDP (joins Nomad SoftAP) | Generic ESP32 (no OLED); reference sketch for the WiFi-UDP link layer alone |

> **Note:** Only the Heltec sketches have been tested end-to-end on real
> hardware. The two `*_esp32_v2` ports compile and mirror the Heltec
> single-mode logic line-for-line (minus OLED), but they are **not fully
> tested yet** - treat them as a starting point for bare-ESP32 boards.
> There is intentionally no combined ESP32 sketch: the PRG-toggle UX
> depends on the OLED countdown, so on bare ESP32 you just flash whichever
> single-mode sketch you need.

The combined sketch defaults to **WiFi UDP** on a fresh flash. To toggle:
power-cycle and **hold the onboard PRG button (GPIO0) for the entire boot
splash (~2.5 s)** - the OLED tells you what mode it's currently in and
counts down. Release before the splash ends to keep the current mode.
The choice is persisted to NVS via `Preferences`, so it survives reboots.

```
   mLRS Nomad ──ESP-NOW or WiFi UDP──▶ Heltec WiFi Kit 32 ──UART──▶ MFD Mini Crossbow
                                              │
                                              └── SSD1306 OLED (telemetry HUD)
```

## Features (common to both variants)

- Bidirectional MAVLink bridge (downlink + uplink).
- Built-in MAVLink v1/v2 parser (no MAVLink library dependency).
- Decoded telemetry on OLED: flight mode, arm state, GPS fix/sats, lat/lon,
  altitude, heading, ground speed (plus airspeed on ArduPlane/VTOL),
  battery V/%, and RSSI %.
- ArduPilot / iNav firmware autodetection from the HEARTBEAT autopilot field
  (mode names are mapped accordingly).
- Link status indicator (`LINK OK` / `NO DATA` / `waiting`) based on packet
  recency.

## OLED layout

Once the link is up, the 128x64 SSD1306 shows seven lines of telemetry,
refreshed every 200 ms:

```
MANUAL  ARM S:12 ARDU      <- flight mode | arm state | sats | autopilot
Lat: 14.123456             <- latitude  (or "Fix:Nd No GPS" if no fix)
Lon:121.654321             <- longitude (or blank             if no fix)
Alt:  42m Hdg: 87          <- altitude (m) and heading (deg)
AS: 14 GS: 18              <- airspeed + ground speed (ArduPlane/VTOL)
                              -- OR --
GS: 18                     <- ground speed only (iNav, copter, unknown)
Bat:11.8V  87%             <- battery voltage and remaining %
RSSI: 73% LINK OK          <- mLRS RSSI (0-100%) and link state
```

| Field | Source MAVLink message | Notes |
| ----- | ---------------------- | ----- |
| Flight mode  | `HEARTBEAT.custom_mode`     | Mapped to ArduPilot or iNav name table depending on autopilot autodetect |
| Arm state    | `HEARTBEAT.base_mode` bit 7 | `ARM` if armed, `dis` otherwise |
| Sats         | `GPS_RAW_INT.satellites_visible` | Satellite count |
| Autopilot    | `HEARTBEAT.autopilot`       | `ARDU` (ArduPilot=3) / `iNAV` (iNav=12) / `?` until first HEARTBEAT |
| Lat / Lon    | `GLOBAL_POSITION_INT.lat/lon` | Degrees, 6 dp; falls back to "Fix:Nd No GPS" using `GPS_RAW_INT.fix_type` when no GPS lock |
| Alt          | `GLOBAL_POSITION_INT.alt`   | Metres above sea level (mm in MAVLink, converted) |
| Hdg          | `GLOBAL_POSITION_INT.hdg`   | Compass heading in degrees |
| AS           | `VFR_HUD.airspeed`          | Airspeed (m/s). **Only shown for ArduPlane fixed-wing or VTOL** (`HEARTBEAT.type` 1 or 19 + autopilot ArduPilot) - on those the EKF synthesises an airspeed from the wind estimator even without a pitot. On iNav and copters AS is hidden because the field is just a ground-speed mirror or unused |
| GS           | `GLOBAL_POSITION_INT.vx/vy` | EKF-fused horizontal ground speed (not raw GPS) |
| Bat V        | `SYS_STATUS.voltage_battery` | Volts |
| Bat %        | `SYS_STATUS.battery_remaining` | 0-100 %, hidden if -1 (unknown) |
| RSSI         | `RADIO_STATUS.rssi`          | mLRS reports 0-100% directly (not 0-254 SiK); `---` if unknown |
| Link state   | derived                      | `LINK OK` if a packet arrived in the last 2 s, else `NO DATA` (after first data) or `waiting` (before any data) |

Before any telemetry arrives, the same area shows status messages
instead - e.g. `Scanning for mLRS bridge...`, `Connecting...`,
`UDP:14550 waiting`, or the boot splash with the current persisted
mode and PRG hold prompt.

## ESP-NOW variant specifics

- Channel auto-scan (1 / 6 / 11 / 13) until an mLRS bridge is found.
- Sender MAC latching so only the paired bridge is accepted.
- WiFi country locked to `EU` (channels 1-13), radio forced to `802.11b` for
  maximum range.

## WiFi UDP variant specifics

- Heltec joins the Nomad's SoftAP. The Nomad generates an SSID from its MAC
  in the form `mLRS-xxxx AP UDP` - check your Tx OLED/CLI for the exact name
  and put it in `WIFI_SSID` at the top of the sketch.
- **UDP mode has no password by default** per the
  [mLRS wireless-bridge docs](https://github.com/olliw42/mLRS-docu/blob/main/docs/WIRELESS_BRIDGE.md),
  so leave `WIFI_PASS` empty unless you've explicitly set one on the Nomad.
- Listens on UDP port `14550` and latches the first peer's IP/port for uplink.
- **Co-exists with a phone/laptop GCS.** The mLRS bridge only sends downlink
  to UDP clients it has seen send a packet first (otherwise the Heltec would
  silently fall off the unicast list the moment a phone running QGC
  registered itself). The sketch sends a MAVLink HEARTBEAT to the Nomad once
  a second so the bridge keeps the Heltec in its `clients[]` table. If DHCP
  doesn't advertise a gateway, the heartbeat target falls back to the `.1`
  host of our `/24` subnet (the typical mLRS SoftAP address).
- Auto-reconnects if the AP drops, with a 3 s grace window so a brief
  disassoc (e.g. another client joining) doesn't tear down our UDP state.

## Bare-ESP32 variants (`mlrs_*_esp32_v2`)

> **Status:** not fully tested yet. The Heltec sketches are the ones
> validated on real hardware; the ESP32 ports compile and mirror the
> Heltec single-mode logic minus OLED, but should be treated as a
> starting point.

Two ESP32 sketches matching the two single-mode Heltec sketches:

| Sketch | Equivalent Heltec sketch |
| ------ | ------------------------ |
| `mlrs_espnow_gcs_esp32_v2` | `mlrs_espnow_gcs_heltec_v2` (ESP-NOW only) |
| `mlrs_udp_gcs_esp32_v2`    | `mlrs_udp_gcs_heltec_v2` (WiFi UDP only) |

Functionally identical to their Heltec counterparts, but with all
SSD1306 code removed so they build and run on any generic ESP32 dev
board.

- No Adafruit_GFX / Adafruit_SSD1306 dependency.
- Status and telemetry print to the USB serial monitor (115200 baud)
  instead of the OLED.
- Pins freed compared to the Heltec builds: GPIO4 / 15 / 16 are no
  longer used. UART RX is still left disabled (`-1`) for the Crossbow
  path; if you have a back-channel from the Crossbow you can wire a
  real RX pin.
- `mlrs_udp_gcs_esp32_v2` mirrors the Heltec single-mode UDP sketch,
  which uses the older 1-byte presence ping; the MAVLink-HEARTBEAT
  presence ping + gateway fallback are only in the combined Heltec
  sketch.
- There is intentionally no combined ESP32 sketch. The PRG-toggle UX
  on the combined Heltec sketch depends on the OLED countdown during
  boot, which doesn't translate cleanly to a bare ESP32 - flash
  whichever single-mode sketch you need instead.

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
| 5 V in         | USB or a dedicated UBEC (see Power below) |

UART RX is intentionally disabled (`-1`) so that **GPIO16** stays free as the
OLED reset line.

### Power

The sketches all run TX at the full 19.5 dBm (~100 mW), which has a peak
current draw that **the MFD Mini Crossbow's 5 V output cannot reliably
supply** - powering the Heltec from the Crossbow's rail will reset the
ESP32 with a `BROWNOUT` reset reason during WiFi TX bursts (ESP-NOW is
gentler than UDP, but neither is safe).

Use **USB** during bench testing and a **dedicated 1 A UBEC** off the
flight battery for field use. Tie the UBEC's ground to the Crossbow's
ground so the UART signal levels stay correct.

## mLRS Nomad (Tx) setup

The Nomad's WiFi Bridge mode **must match the sketch you flash**:

| Sketch you flash | Nomad WiFi Bridge mode |
| ---------------- | ---------------------- |
| `mlrs_gcs_heltec_v2` (combined) | match the **active mode** the Heltec is currently in (`ESP-NOW` or `WiFi UDP`) - toggle one to match the other |
| `mlrs_espnow_gcs_heltec_v2` | `ESP-NOW` |
| `mlrs_udp_gcs_heltec_v2`    | `WiFi UDP` |
| `mlrs_espnow_gcs_esp32_v2` | `ESP-NOW` |
| `mlrs_udp_gcs_esp32_v2`    | `WiFi UDP` |

Quick path via the OLED menu:

```
Setup -> WiFi Bridge -> Mode -> <ESP-NOW | WiFi UDP>
```

After changing the mode, reboot the Nomad so the new radio mode takes effect.

## Build & flash

### Arduino IDE

1. Install the **ESP32 Arduino core >= 3.0.0** via Boards Manager.
2. Install these libraries via Library Manager (**Heltec sketches only**;
   the `*_esp32_v2` sketches have no library dependencies):
   - Adafruit SSD1306
   - Adafruit GFX Library
3. Board:
   - Heltec sketches: **ESP32 Arduino -> Heltec WiFi Kit 32**
   - `*_esp32_v2` sketches: **ESP32 Arduino -> ESP32 Dev Module** (or
     whichever generic ESP32 board you're using)
4. Open the `.ino` for the sketch you want - normally
   `mlrs_gcs_heltec_v2/mlrs_gcs_heltec_v2.ino` (combined, runtime-toggleable)
   or one of the single-mode sketches - then Upload.

### Configurable defaults

Set at the top of each `.ino`:

| Define | Default | Purpose |
| ------ | ------- | ------- |
| `CROSSBOW_BAUD` | `115200` | UART baud to the Crossbow |
| `TX_PIN` | `17` | UART TX pin |
| `OLED_UPDATE_MS` | `200` | OLED refresh interval |
| `OLED_SDA` / `OLED_SCL` / `OLED_RST` | `4` / `15` / `16` | OLED I2C pins (Heltec defaults; absent in `*_esp32_v2`) |
| `OLED_ADDR` | `0x3C` | SSD1306 I2C address (absent in `*_esp32_v2`) |
| `STATUS_UPDATE_MS` | `2000` | (`*_esp32_v2` sketches only) serial status print cadence |

UDP variant adds:

| Define | Default | Purpose |
| ------ | ------- | ------- |
| `WIFI_SSID` | _(empty - set to your Nomad's `mLRS-xxxx AP UDP` SSID)_ | Nomad SoftAP SSID |
| `WIFI_PASS` | _(empty - UDP mode is open by default; leave blank unless you set one)_ | Nomad SoftAP password |
| `UDP_PORT` | `14550` | MAVLink UDP port |

## Supported MAVLink messages

| Msg ID | Name | Fields used |
| ------ | ---- | ----------- |
| 0  | `HEARTBEAT`           | `type`, `autopilot`, `base_mode`, `custom_mode` |
| 1  | `SYS_STATUS`          | `voltage_battery`, `battery_remaining` |
| 24 | `GPS_RAW_INT`         | `fix_type`, `satellites_visible` |
| 33 | `GLOBAL_POSITION_INT` | `lat`, `lon`, `alt`, `vx`, `vy`, `hdg` |
| 74 | `VFR_HUD`             | `airspeed` (only displayed on ArduPlane fixed-wing / VTOL) |
| 109 | `RADIO_STATUS`       | `rssi` (treated as 0-100% per mLRS, not 0-254 SiK) |

All other messages are still forwarded over UART -- only these are decoded for
the on-board OLED.

## License

GPL v3.
