//*******************************************************
// mLRS GCS Bridge - Heltec WiFi Kit 32 v1/v2/v3 (combined)
// Switches between ESP-NOW and WiFi UDP link layers at
// runtime via the onboard PRG button (GPIO0). Mode is
// persisted to NVS so it survives reboots.
//
// On boot a splash screen shows the current mode. Hold
// PRG for the entire splash window (~2.5 s) to toggle
// to the other mode and reboot.
//
// Default mode on fresh flash: WiFi UDP.
//
// Forwards MAVLink to MFD Mini Crossbow via Serial2 and
// shows decoded telemetry on the onboard SSD1306 OLED.
//
// Libraries required (Library Manager):
//   - Adafruit SSD1306
//   - Adafruit GFX Library
//
// Board (v1/v2): Tools -> Board -> ESP32 Arduino -> Heltec WiFi Kit 32
// Board (v3/v4): Tools -> Board -> ESP32 Arduino -> Heltec WiFi Kit 32(V3)
// Pin map is chosen automatically from the selected board's MCU target
// (ESP32 for v1/v2, ESP32-S3 for v3/v4).
// ESP32 Arduino core >= 3.0.0
//
// License: GPL v3
//*******************************************************

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Forward declaration so Arduino's auto-generated function prototypes -
// inserted between the #includes and the first function definition - can
// reference MavPacket before its full definition appears further down.
struct MavPacket;

//-------------------------------------------------------
// USER CONFIGURATION
//-------------------------------------------------------

// UDP mode only: set to match your Nomad's WiFi Bridge.
// SSID is generated from the Tx MAC as "mLRS-xxxx AP UDP".
// UDP mode has no password by default; leave blank unless set.
#define WIFI_SSID       ""
#define WIFI_PASS       ""
#define UDP_PORT        14550

#define CROSSBOW_BAUD   115200

#define OLED_UPDATE_MS  200

#define PRG_PIN         0      // Onboard PRG button (active LOW, both board generations)
#define SPLASH_MS       2500   // Hold PRG this long during splash to toggle

// Board-specific pin map. The ESP32-S3 branch covers Heltec WiFi Kit 32 v3
// (and v4, which keeps the same OLED layout). The original ESP32 branch
// covers v1/v2. On v3 GPIO17 is the OLED SDA, so the Crossbow UART has to
// move; GPIO33 is broken out on the V3 header and is neither a strapping
// pin nor a USB D+/- line.
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define TX_PIN        33     // Crossbow MAVLink input (Heltec V3 free GPIO)
  #define OLED_SDA      17
  #define OLED_SCL      18
  #define OLED_RST      21
  #define VEXT_PIN      36     // Drive LOW to power the OLED rail (Vext)
#else
  #define TX_PIN        17     // GPIO17 -> Crossbow MAVLink input
  #define OLED_SDA      4
  #define OLED_SCL      15
  #define OLED_RST      16
#endif
#define OLED_ADDR       0x3C

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);
WiFiUDP   udp;
Preferences prefs;


//-------------------------------------------------------
// Mode selection
//-------------------------------------------------------

enum Mode : uint8_t { MODE_UDP = 0, MODE_ESPNOW = 1 };
Mode current_mode = MODE_UDP;

const char* mode_str(Mode m) {
    return (m == MODE_UDP) ? "WiFi UDP" : "ESP-NOW";
}


//-------------------------------------------------------
// Boot reason helpers (diagnostic)
//-------------------------------------------------------

const char* reset_reason_str(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT (RST pin)";
        case ESP_RST_SW:        return "SW (esp_restart)";
        case ESP_RST_PANIC:     return "PANIC (Guru Meditation)";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT (other)";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}


//-------------------------------------------------------
// Minimal MAVLink parser (no library needed)
//-------------------------------------------------------

#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_VFR_HUD              74
#define MAVLINK_MSG_ID_RADIO_STATUS         109
#define MAVLINK_MSG_ID_HOME_POSITION        242

struct MavPacket {
    bool     valid;
    uint8_t  version;
    uint32_t msg_id;
    uint8_t  payload[280];
    uint8_t  payload_len;
};

// Explicit prototypes so Arduino's auto-prototype generator (which inserts
// prototypes near the top of the .ino, sometimes above the MavPacket
// forward declaration) doesn't try to synthesise its own broken ones for
// these functions.
bool mav_parse_byte(uint8_t c, MavPacket& pkt);
void process_mavlink_packet(MavPacket& pkt);

// MAVLink CRC-16 (X.25), used to build our presence-ping HEARTBEAT.
inline uint16_t mav_crc_accumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
    tmp ^= (tmp << 4);
    return ((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
}

// Build a MAVLink v1 HEARTBEAT from a GCS endpoint. Returns packet length.
// sys_id=255, comp_id=190 is the conventional GCS identity (Mission Planner).
int build_gcs_heartbeat(uint8_t* out, uint8_t seq) {
    out[0] = 0xFE;        // STX (MAVLink v1)
    out[1] = 9;           // payload length
    out[2] = seq;         // sequence
    out[3] = 255;         // sys_id (GCS)
    out[4] = 190;         // comp_id (MISSION_PLANNER)
    out[5] = 0;           // msg_id = HEARTBEAT
    // payload: u32 custom_mode, u8 type, u8 autopilot, u8 base_mode,
    //          u8 system_status, u8 mavlink_version
    out[6]  = 0; out[7] = 0; out[8] = 0; out[9] = 0;  // custom_mode = 0
    out[10] = 6;          // type = MAV_TYPE_GCS
    out[11] = 8;          // autopilot = MAV_AUTOPILOT_INVALID
    out[12] = 0;          // base_mode
    out[13] = 4;          // system_status = MAV_STATE_ACTIVE
    out[14] = 3;          // mavlink_version
    // CRC over [LEN..end of payload] + HEARTBEAT crc_extra (50)
    uint16_t crc = 0xFFFF;
    for (int i = 1; i < 15; i++) crc = mav_crc_accumulate(out[i], crc);
    crc = mav_crc_accumulate(50, crc);   // HEARTBEAT crc_extra
    out[15] = (uint8_t)(crc & 0xff);
    out[16] = (uint8_t)(crc >> 8);
    return 17;
}

enum MavParseState { WAIT_STX, IN_HEADER, IN_PAYLOAD, IN_CRC };

static MavParseState parse_state = WAIT_STX;
static uint8_t  parse_buf[300];
static int      parse_idx  = 0;
static int      parse_len  = 0;
static int      header_len = 0;
static uint8_t  parse_ver  = 0;

int32_t  mav_get_i32(const uint8_t* p, int o) {
    return (int32_t)(p[o] | (p[o+1]<<8) | (p[o+2]<<16) | (p[o+3]<<24));
}
uint32_t mav_get_u32(const uint8_t* p, int o) {
    return (uint32_t)(p[o] | (p[o+1]<<8) | (p[o+2]<<16) | (p[o+3]<<24));
}
uint16_t mav_get_u16(const uint8_t* p, int o) {
    return (uint16_t)(p[o] | (p[o+1]<<8));
}

bool mav_parse_byte(uint8_t c, MavPacket& pkt) {
    switch (parse_state) {
        case WAIT_STX:
            if (c == 0xFE) {
                parse_ver = 1; header_len = 6;
                parse_buf[0] = c; parse_idx = 1;
                parse_state = IN_HEADER;
            } else if (c == 0xFD) {
                parse_ver = 2; header_len = 10;
                parse_buf[0] = c; parse_idx = 1;
                parse_state = IN_HEADER;
            }
            break;
        case IN_HEADER:
            parse_buf[parse_idx++] = c;
            if (parse_idx >= header_len) {
                parse_len = parse_buf[1];
                parse_state = IN_PAYLOAD;
            }
            break;
        case IN_PAYLOAD:
            parse_buf[parse_idx++] = c;
            if (parse_idx >= header_len + parse_len)
                parse_state = IN_CRC;
            break;
        case IN_CRC:
            parse_buf[parse_idx++] = c;
            if (parse_idx >= header_len + parse_len + 2) {
                pkt.valid       = true;
                pkt.version     = parse_ver;
                pkt.payload_len = parse_len;
                if (parse_ver == 1) {
                    pkt.msg_id = parse_buf[5];
                    memcpy(pkt.payload, &parse_buf[6], parse_len);
                } else {
                    pkt.msg_id = parse_buf[7] | (parse_buf[8]<<8) | (parse_buf[9]<<16);
                    memcpy(pkt.payload, &parse_buf[10], parse_len);
                }
                parse_state = WAIT_STX;
                parse_idx   = 0;
                return true;
            }
            break;
        default:
            parse_state = WAIT_STX;
            parse_idx   = 0;
            break;
    }
    if (parse_idx >= (int)sizeof(parse_buf)) {
        parse_state = WAIT_STX;
        parse_idx   = 0;
    }
    return false;
}


//-------------------------------------------------------
// Telemetry data + decoding
//-------------------------------------------------------

float     telem_lat         = 0.0f;
float     telem_lon         = 0.0f;
float     telem_alt_m       = 0.0f;
float     telem_spd_ms      = 0.0f;
float     telem_airspeed_ms = 0.0f;
float     home_lat          = 0.0f;
float     home_lon          = 0.0f;
bool      home_set          = false;
int       telem_satellites  = 0;
uint8_t   telem_fix         = 0;
float     telem_bat_v       = 0.0f;
int       telem_bat_pct     = -1;
int       telem_rssi_pct    = -1;
uint32_t  telem_flight_mode = 0;
uint8_t   telem_vehicle_type = 0;
bool      telem_armed       = false;
bool      telem_gps_ok      = false;
bool      telem_ever_data   = false;
unsigned long telem_last_ms = 0;

enum FwType { FW_UNKNOWN, FW_ARDU, FW_INAV };
FwType    fw_type           = FW_UNKNOWN;
const char* fw_label        = "?";

const char* get_mode_str_ardu(uint32_t mode) {
    switch (mode) {
        case 0:  return "MANUAL";
        case 1:  return "CIRCLE";
        case 2:  return "STAB";
        case 5:  return "FBWA";
        case 6:  return "FBWB";
        case 7:  return "CRUISE";
        case 10: return "AUTO";
        case 11: return "RTL";
        case 12: return "LOITER";
        case 15: return "GUIDED";
        case 26: return "CRSH";
        default: return "---";
    }
}

const char* get_mode_str_inav(uint32_t mode) {
    switch (mode) {
        case 0:  return "ANGLE";
        case 1:  return "HORIZON";
        case 2:  return "ACRO";
        case 3:  return "MANUAL";
        case 4:  return "ALTHOLD";
        case 5:  return "POSHOLD";
        case 6:  return "HEADFRE";
        case 7:  return "NAVRTH";
        case 8:  return "NAVWP";
        case 9:  return "NAVCRSH";
        case 10: return "NAVPH";
        case 11: return "COURSE";
        case 12: return "AUTOTRN";
        case 17: return "LAUNCH";
        case 19: return "HOMING";
        default: return "---";
    }
}

const char* get_mode_str(uint32_t mode) {
    if (fw_type == FW_INAV) return get_mode_str_inav(mode);
    return get_mode_str_ardu(mode);
}

void process_mavlink_packet(MavPacket& pkt) {
    const uint8_t* p = pkt.payload;
    switch (pkt.msg_id) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if (pkt.payload_len >= 28) {
                telem_lat   = (float)mav_get_i32(p, 4)  / 1e7f;
                telem_lon   = (float)mav_get_i32(p, 8)  / 1e7f;
                telem_alt_m = (float)mav_get_i32(p, 16) / 1000.0f;
                int16_t vx  = (int16_t)mav_get_u16(p, 20);
                int16_t vy  = (int16_t)mav_get_u16(p, 22);
                telem_spd_ms = sqrtf((float)(vx*vx + vy*vy)) / 100.0f;
                telem_gps_ok = true;
            }
            break;
        case MAVLINK_MSG_ID_HOME_POSITION:
            // HOME_POSITION: int32 latitude @0 (deg*1e7), int32 longitude @4
            // (deg*1e7), int32 altitude @8 (mm). ArduPilot streams this on
            // arming and on any home update. Match Mission Planner / Yaapu by
            // anchoring distance-to-home to the autopilot's official home.
            if (pkt.payload_len >= 8) {
                home_lat = (float)mav_get_i32(p, 0) / 1e7f;
                home_lon = (float)mav_get_i32(p, 4) / 1e7f;
                home_set = true;
            }
            break;
        case MAVLINK_MSG_ID_GPS_RAW_INT:
            if (pkt.payload_len >= 30) {
                telem_fix        = p[28];
                telem_satellites = p[29];
            }
            break;
        case MAVLINK_MSG_ID_SYS_STATUS:
            if (pkt.payload_len >= 31) {
                telem_bat_v   = (float)mav_get_u16(p, 14) / 1000.0f;
                telem_bat_pct = (int8_t)p[30];
            }
            break;
        case MAVLINK_MSG_ID_RADIO_STATUS:
            if (pkt.payload_len >= 5) {
                uint8_t raw = p[4];
                telem_rssi_pct = (raw == 255) ? -1 : (int)constrain(raw, 0, 100);
            }
            break;
        case MAVLINK_MSG_ID_VFR_HUD:
            // VFR_HUD layout: float airspeed @0, float groundspeed @4,
            // int16 heading @8, uint16 throttle @10, float alt @12, float climb @16.
            if (pkt.payload_len >= 4) {
                float as;
                memcpy(&as, &p[0], sizeof(float));
                telem_airspeed_ms = as;
            }
            break;
        case MAVLINK_MSG_ID_HEARTBEAT:
            if (pkt.payload_len >= 7) {
                uint8_t mav_type   = p[4];
                uint8_t autopilot  = p[5];
                uint8_t base_mode  = p[6];
                if (fw_type == FW_UNKNOWN) {
                    if (autopilot == 12)      { fw_type = FW_INAV; fw_label = "iNAV"; }
                    else if (autopilot == 3)  { fw_type = FW_ARDU; fw_label = "ARDU"; }
                }
                if (mav_type == 1 || mav_type == 2 || mav_type == 3 ||
                    mav_type == 13 || mav_type == 19) {
                    telem_vehicle_type = mav_type;
                    telem_flight_mode = mav_get_u32(p, 0);
                    telem_armed = (base_mode & 0x80) != 0;
                }
            }
            break;
    }
    telem_ever_data = true;
    telem_last_ms   = millis();
}


//-------------------------------------------------------
// Distance-to-home (equirectangular approximation, good to ~10 km)
//-------------------------------------------------------

float distance_to_home_m() {
    if (!home_set || !telem_gps_ok) return -1.0f;
    float home_lat_rad = home_lat * (PI / 180.0f);
    float dlat_m = (telem_lat - home_lat) * 111320.0f;
    float dlon_m = (telem_lon - home_lon) * 111320.0f * cosf(home_lat_rad);
    return sqrtf(dlat_m * dlat_m + dlon_m * dlon_m);
}

void format_dist_to_home(char* buf, size_t n) {
    float d = distance_to_home_m();
    if (d < 0)            snprintf(buf, n, "----");
    else if (d < 1000.0f) snprintf(buf, n, "%3.0fm", d);
    else                  snprintf(buf, n, "%.2fkm", d / 1000.0f);
}

void format_alt(char* buf, size_t n) {
    if (telem_alt_m < 1000.0f) snprintf(buf, n, "%4.0fm", telem_alt_m);
    else                       snprintf(buf, n, "%.2fkm", telem_alt_m / 1000.0f);
}


//-------------------------------------------------------
// OLED
//-------------------------------------------------------

void oled_init() {
#if defined(VEXT_PIN)
    // Heltec V3/V4: OLED rail is gated by Vext. Drive LOW to power it
    // before talking to the SSD1306.
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, LOW);
    delay(50);
#endif
    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("SSD1306 init failed!");
        for (;;) delay(1000);
    }
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.display();
}

void oled_status(const char* l1, const char* l2 = "") {
    display.clearDisplay();
    display.setCursor(0, 0);  display.printf("mLRS GCS [%s]", mode_str(current_mode));
    display.setCursor(0, 16); display.println(l1);
    display.setCursor(0, 28); display.println(l2);
    display.display();
}

void oled_telem() {
    bool data_active = (millis() - telem_last_ms < 2000);

    display.clearDisplay();
    display.setCursor(0, 0);

    display.printf("%-6s %s S:%-2d %s\n",
        get_mode_str(telem_flight_mode),
        telem_armed ? "ARM" : "dis",
        telem_satellites,
        fw_label);

    if (telem_gps_ok) {
        display.printf("Lat:%10.6f\n", telem_lat);
        display.printf("Lon:%10.6f\n", telem_lon);
    } else {
        display.printf("Fix:%dD  No GPS\n", telem_fix);
        display.printf("\n");
    }

    char abuf[8], hbuf[8];
    format_alt(abuf, sizeof(abuf));
    format_dist_to_home(hbuf, sizeof(hbuf));
    display.printf("Alt:%s Home:%s\n", abuf, hbuf);

    // ArduPlane (fixed-wing or VTOL) has a meaningful airspeed even without
    // a pitot, via the EKF wind estimator. iNav without a pitot just mirrors
    // ground speed, and copters have no meaningful airspeed - show GS only
    // in those cases.
    bool show_as = (fw_type == FW_ARDU) &&
                   (telem_vehicle_type == 1 || telem_vehicle_type == 19);
    if (show_as) {
        display.printf("AS:%3.0f GS:%3.0f m/s\n", telem_airspeed_ms, telem_spd_ms);
    } else {
        display.printf("GS:%3.0f m/s\n", telem_spd_ms);
    }

    if (telem_bat_pct >= 0)
        display.printf("Bat:%4.1fV %3d%%\n", telem_bat_v, telem_bat_pct);
    else
        display.printf("Bat:%4.1fV\n", telem_bat_v);

    if (telem_rssi_pct >= 0)
        display.printf("RSSI:%3d%% %s\n",
            telem_rssi_pct,
            data_active ? "LINK OK" : (telem_ever_data ? "NO DATA" : "waiting"));
    else
        display.printf("RSSI:--- %s\n",
            data_active ? "LINK OK" : (telem_ever_data ? "NO DATA" : "waiting"));

    display.display();
}


//-------------------------------------------------------
// ESP-NOW state
//-------------------------------------------------------

#define ESPNOW_RXBUF_SIZE  2048
uint8_t espnow_rxbuf[ESPNOW_RXBUF_SIZE];
volatile uint16_t espnow_rxbuf_head = 0;
volatile uint16_t espnow_rxbuf_tail = 0;

volatile bool espnow_latched_mac_available = false;
uint8_t espnow_latched_mac[6];
bool    latched_peer_added = false;

uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t scan_channels[] = { 1, 6, 11, 13 };

void espnow_rxbuf_push(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        uint16_t next = (espnow_rxbuf_head + 1) & (ESPNOW_RXBUF_SIZE - 1);
        if (next == espnow_rxbuf_tail) break;
        espnow_rxbuf[espnow_rxbuf_head] = data[i];
        espnow_rxbuf_head = next;
    }
}

int espnow_rxbuf_pop(uint8_t* dst, int maxlen) {
    int cnt = 0;
    while (espnow_rxbuf_tail != espnow_rxbuf_head && cnt < maxlen) {
        dst[cnt++] = espnow_rxbuf[espnow_rxbuf_tail];
        espnow_rxbuf_tail = (espnow_rxbuf_tail + 1) & (ESPNOW_RXBUF_SIZE - 1);
    }
    return cnt;
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void espnow_recv_cb(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = info->src_addr;
#else
void espnow_recv_cb(const uint8_t* mac, const uint8_t* data, int len)
{
    const uint8_t* sender_mac = mac;
#endif
    if (!espnow_latched_mac_available) {
        memcpy(espnow_latched_mac, sender_mac, 6);
        espnow_latched_mac_available = true;
    } else if (memcmp(sender_mac, espnow_latched_mac, 6) != 0) {
        return;
    }
    espnow_rxbuf_push(data, len);
}


//-------------------------------------------------------
// UDP state
//-------------------------------------------------------

bool      udp_peer_latched = false;
IPAddress udp_peer_ip;
uint16_t  udp_peer_port    = 0;


//-------------------------------------------------------
// Shared link state
//-------------------------------------------------------

bool          is_connected          = false;
unsigned long is_connected_tlast_ms = 0;
bool          link_initialized      = false;
uint8_t       buf[1024];


//-------------------------------------------------------
// ESP-NOW bring-up + channel scan
//-------------------------------------------------------

void scan_for_bridge(void) {
    if (espnow_latched_mac_available) {
        esp_now_del_peer(espnow_latched_mac);
        espnow_latched_mac_available = false;
        latched_peer_added = false;
    }

    oled_status("Scanning for", "mLRS bridge...");

    while (true) {
        for (int i = 0; i < (int)sizeof(scan_channels); i++) {
            esp_wifi_set_channel(scan_channels[i], WIFI_SECOND_CHAN_NONE);

            char ch_str[20];
            snprintf(ch_str, sizeof(ch_str), "Channel %d...", scan_channels[i]);
            oled_status("Scanning...", ch_str);

            unsigned long t = millis();
            while (millis() - t < 500) {
                while (Serial2.available()) Serial2.read();
                if (espnow_rxbuf_head != espnow_rxbuf_tail) {
                    char found_str[20];
                    snprintf(found_str, sizeof(found_str), "Ch:%d found!", scan_channels[i]);
                    oled_status("Bridge found!", found_str);
                    delay(800);
                    return;
                }
                delay(1);
            }
        }
    }
}

void setup_espnow(void) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    wifi_country_t country = {
        .cc = "EU",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_MANUAL
    };
    esp_wifi_set_country(&country);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

    // ESP-NOW didn't show brownouts at default; keep full power for range.
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);

    scan_for_bridge();
}


//-------------------------------------------------------
// WiFi event logging (UDP mode)
//-------------------------------------------------------

void wifi_event_handler(WiFiEvent_t ev, WiFiEventInfo_t info) {
    switch (ev) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            Serial.println("[wifi] STA_CONNECTED");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("[wifi] GOT_IP ");
            Serial.println(WiFi.localIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.print("[wifi] DISCONNECTED reason=");
            Serial.println(info.wifi_sta_disconnected.reason);
            break;
        default:
            break;
    }
}


//-------------------------------------------------------
// UDP bring-up
//-------------------------------------------------------

void setup_udp(void) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(50);

    WiFi.persistent(true);
    WiFi.setAutoReconnect(true);

    // Force 802.11b (flatter TX envelope, matches the ESP-NOW path) and
    // use the maximum 19.5 dBm TX power. The earlier brownout we saw on
    // the Crossbow 5 V rail can return at this power level - use USB or a
    // dedicated UBEC for the Heltec if that's an issue.
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    char l2[24];
    snprintf(l2, sizeof(l2), "SSID:%s", WIFI_SSID);
    oled_status("Connecting...", l2);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long t0 = millis();
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        char prog[24];
        snprintf(prog, sizeof(prog), "Connecting%.*s", (dots % 4) + 1, "....");
        oled_status(prog, l2);
        dots++;

        if (millis() - t0 > 15000) {
            oled_status("WiFi timeout", "retrying...");
            delay(800);
            WiFi.disconnect();
            delay(200);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            t0 = millis();
        }
    }

    int udp_ok = udp.begin(UDP_PORT);
    Serial.print("[udp] begin(");
    Serial.print(UDP_PORT);
    Serial.print(") -> ");
    Serial.println(udp_ok);

    IPAddress ip = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    Serial.print("[udp] me=");      Serial.print(ip);
    Serial.print(" gateway=");      Serial.print(gw);
    Serial.print(" mask=");         Serial.println(WiFi.subnetMask());

    char l1[24], l2b[24];
    snprintf(l1, sizeof(l1), "IP:%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    snprintf(l2b, sizeof(l2b), "UDP:%d waiting", UDP_PORT);
    oled_status(l1, l2b);
}


//-------------------------------------------------------
// Boot splash + mode toggle
//-------------------------------------------------------

void splash_and_maybe_toggle() {
    pinMode(PRG_PIN, INPUT_PULLUP);

    display.clearDisplay();
    display.setCursor(0, 0);  display.println("mLRS GCS Bridge");
    display.setCursor(0, 16); display.printf("Mode: %s\n", mode_str(current_mode));
    display.setCursor(0, 32); display.println("Hold PRG to swap");
    display.setCursor(0, 48); display.println("");
    display.display();

    unsigned long t0 = millis();
    bool held_all_along = true;
    bool any_press_seen = false;

    while (millis() - t0 < SPLASH_MS) {
        bool pressed = (digitalRead(PRG_PIN) == LOW);
        if (pressed) any_press_seen = true;
        else         held_all_along = false;

        unsigned long remaining = SPLASH_MS - (millis() - t0);
        display.fillRect(0, 48, 128, 16, SSD1306_BLACK);
        display.setCursor(0, 48);
        display.printf("Boot in %lus", (remaining + 999) / 1000);
        display.display();
        delay(50);
    }

    if (any_press_seen && held_all_along) {
        Mode next = (current_mode == MODE_UDP) ? MODE_ESPNOW : MODE_UDP;
        prefs.begin("mlrs", false);
        prefs.putUChar("mode", (uint8_t)next);
        prefs.end();

        display.clearDisplay();
        display.setCursor(0, 0);  display.println("Mode toggled.");
        display.setCursor(0, 16); display.printf("Next: %s", mode_str(next));
        display.setCursor(0, 32); display.println("Rebooting...");
        display.display();
        delay(1200);
        ESP.restart();
    }
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.print("[boot] reset_reason=");
    Serial.print((int)esp_reset_reason());
    Serial.print(" (");
    Serial.print(reset_reason_str(esp_reset_reason()));
    Serial.println(")");
    Serial.print("[boot] free_heap=");
    Serial.println(ESP.getFreeHeap());

    // Load persisted mode (default UDP on fresh flash).
    prefs.begin("mlrs", true);
    current_mode = (Mode)prefs.getUChar("mode", (uint8_t)MODE_UDP);
    prefs.end();
    Serial.print("[boot] mode=");
    Serial.println(mode_str(current_mode));

    WiFi.onEvent(wifi_event_handler);

    oled_init();
    splash_and_maybe_toggle();

    // TX only - RX disabled so GPIO16 stays free for OLED reset.
    Serial2.begin(CROSSBOW_BAUD, SERIAL_8N1, -1, TX_PIN);

    // Common reset
    espnow_rxbuf_head = 0;
    espnow_rxbuf_tail = 0;
    espnow_latched_mac_available = false;
    latched_peer_added    = false;
    udp_peer_latched      = false;
    is_connected          = false;
    is_connected_tlast_ms = 0;
    link_initialized      = false;
}


// ----------------- ESP-NOW loop tick -----------------

void loop_espnow(unsigned long tnow_ms) {
    // Connection timeout
    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
        telem_gps_ok = false;
        scan_for_bridge();
    }

    // Drain ESP-NOW buffer
    int len = espnow_rxbuf_pop(buf, sizeof(buf));
    if (len > 0) {
        Serial2.write(buf, len);

        MavPacket pkt;
        for (int i = 0; i < len; i++) {
            pkt.valid = false;
            if (mav_parse_byte(buf[i], pkt)) {
                process_mavlink_packet(pkt);
            }
        }

        is_connected = true;
        is_connected_tlast_ms = tnow_ms;
    }

    // Register latched peer for uplink
    if (espnow_latched_mac_available && !latched_peer_added) {
        esp_now_peer_info_t lpeer = {};
        memcpy(lpeer.peer_addr, espnow_latched_mac, 6);
        esp_now_add_peer(&lpeer);
        latched_peer_added = true;
    }

    // Uplink: Crossbow -> ESP-NOW -> vehicle
    int rlen = 0;
    while (Serial2.available() && rlen < (int)sizeof(buf)) {
        buf[rlen++] = Serial2.read();
    }
    if (rlen > 0) {
        uint8_t* dest = espnow_latched_mac_available ?
                        espnow_latched_mac : broadcast_mac;
        esp_now_send(dest, buf, rlen);
    }
}


// ----------------- UDP loop tick -----------------

void loop_udp(unsigned long tnow_ms) {
    static unsigned long wifi_down_t0 = 0;
    if (WiFi.status() != WL_CONNECTED) {
        if (wifi_down_t0 == 0) wifi_down_t0 = tnow_ms;
        if (tnow_ms - wifi_down_t0 > 3000) {
            udp.stop();
            udp_peer_latched = false;
            is_connected    = false;
            telem_gps_ok    = false;
            link_initialized = false;
            wifi_down_t0    = 0;
        }
        return;
    }
    wifi_down_t0 = 0;

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
        telem_gps_ok = false;
        oled_status("UDP idle", "waiting...");
    }

    // Downlink
    while (true) {
        int psize = udp.parsePacket();
        if (psize <= 0) break;

        IPAddress src_ip = udp.remoteIP();
        uint16_t  src_pt = udp.remotePort();

        if (!udp_peer_latched) {
            udp_peer_ip      = src_ip;
            udp_peer_port    = src_pt;
            udp_peer_latched = true;
        }

        int len = udp.read(buf, sizeof(buf));
        if (len <= 0) break;

        Serial2.write(buf, len);

        MavPacket pkt;
        for (int i = 0; i < len; i++) {
            pkt.valid = false;
            if (mav_parse_byte(buf[i], pkt)) {
                process_mavlink_packet(pkt);
            }
        }

        is_connected          = true;
        is_connected_tlast_ms = tnow_ms;
    }

    // Uplink
    int rlen = 0;
    while (Serial2.available() && rlen < (int)sizeof(buf)) {
        buf[rlen++] = Serial2.read();
    }
    if (rlen > 0 && udp_peer_latched) {
        udp.beginPacket(udp_peer_ip, udp_peer_port);
        udp.write(buf, rlen);
        udp.endPacket();
    }

    // Presence ping: keep us in the mLRS bridge's UDP client list so it
    // unicasts downlink to us even after another GCS registers.
    //
    // Destination preference:
    //   1. latched peer IP (after we've received anything once),
    //   2. gateway IP from DHCP,
    //   3. .1 host on our /24 subnet - mLRS doesn't always advertise a
    //      gateway, and without it WiFi.gatewayIP() returns 0.0.0.0
    //      which would silently skip the ping.
    static unsigned long ping_t = 0;
    static bool ping_logged = false;
    if (tnow_ms - ping_t >= 1000) {
        ping_t = tnow_ms;
        IPAddress dest;
        if (udp_peer_latched) {
            dest = udp_peer_ip;
        } else {
            dest = WiFi.gatewayIP();
            if (dest[0] == 0) {
                IPAddress me = WiFi.localIP();
                dest = IPAddress(me[0], me[1], me[2], 1);
            }
        }
        if (dest[0] != 0) {
            if (!ping_logged) {
                Serial.print("[udp] presence_ping -> ");
                Serial.print(dest);
                Serial.println(" (MAVLink HEARTBEAT)");
                ping_logged = true;
            }
            static uint8_t hb_seq = 0;
            uint8_t hb[17];
            int hblen = build_gcs_heartbeat(hb, hb_seq++);
            udp.beginPacket(dest, UDP_PORT);
            udp.write(hb, hblen);
            udp.endPacket();
        }
    }
}


// ----------------- main loop -----------------

void loop() {
    if (!link_initialized) {
        link_initialized = true;
        if (current_mode == MODE_UDP) setup_udp();
        else                          setup_espnow();
        return;
    }

    unsigned long tnow_ms = millis();

    if (current_mode == MODE_UDP) loop_udp(tnow_ms);
    else                          loop_espnow(tnow_ms);

    // Heap / WiFi snapshot every 5 s
    static unsigned long heap_t = 0;
    if (tnow_ms - heap_t >= 5000) {
        heap_t = tnow_ms;
        Serial.print("[heap] free=");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" min=");
        Serial.print(ESP.getMinFreeHeap());
        Serial.print(" mode=");
        Serial.print(mode_str(current_mode));
        if (current_mode == MODE_UDP) {
            Serial.print(" wifi=");
            Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "DOWN");
            Serial.print(" rssi=");
            Serial.println(WiFi.RSSI());
        } else {
            Serial.println();
        }
    }

    // OLED refresh
    static unsigned long oled_t = 0;
    if (tnow_ms - oled_t >= OLED_UPDATE_MS) {
        oled_t = tnow_ms;
        if (is_connected) {
            oled_telem();
        } else {
            if (current_mode == MODE_UDP) {
                char l1[24], l2[24];
                IPAddress ip = WiFi.localIP();
                snprintf(l1, sizeof(l1), "IP:%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
                snprintf(l2, sizeof(l2), "UDP:%d waiting", UDP_PORT);
                oled_status(l1, l2);
            } else {
                oled_status("Scanning for", "mLRS bridge...");
            }
        }
    }

    delay(2);
}
