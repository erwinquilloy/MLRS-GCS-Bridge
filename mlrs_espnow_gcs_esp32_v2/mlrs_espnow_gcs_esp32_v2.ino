//*******************************************************
// mLRS ESP-NOW GCS Bridge - generic ESP32 (no OLED)
// Receives MAVLink from mLRS Nomad via ESP-NOW and
// forwards to MFD Mini Crossbow via UART (Serial2).
// Status and telemetry print to the USB serial monitor.
//
// OLED-less port of mlrs_espnow_gcs_heltec_v2.
// No external libraries required - core only.
//
// Board: Tools -> Board -> ESP32 Arduino -> ESP32 Dev Module
// ESP32 Arduino core >= 3.0.0
//
// Wiring to MFD Mini Crossbow:
//   ESP32 GPIO17 (TX2)   -->  Crossbow MAVLink input pin
//   ESP32 GND            -->  Crossbow GND
//   Power ESP32 via USB or a dedicated UBEC
//
// License: GPL v3
//*******************************************************

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

//-------------------------------------------------------
// USER CONFIGURATION
//-------------------------------------------------------

#define CROSSBOW_BAUD   115200
#define TX_PIN          17    // GPIO17 -> Crossbow MAVLink input
// RX disabled (-1); if your Crossbow has a back-channel wire a real pin.

#define STATUS_UPDATE_MS  2000  // Serial telemetry/status print cadence


//-------------------------------------------------------
// Minimal MAVLink parser (no library needed)
//-------------------------------------------------------

#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
#define MAVLINK_MSG_ID_VFR_HUD              74
#define MAVLINK_MSG_ID_RADIO_STATUS         109

struct MavPacket {
    bool     valid;
    uint8_t  version;
    uint32_t msg_id;
    uint8_t  payload[280];
    uint8_t  payload_len;
};

enum MavParseState {
    WAIT_STX, IN_HEADER, IN_PAYLOAD, IN_CRC
};

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
// Telemetry data
//-------------------------------------------------------

float     telem_lat         = 0.0f;
float     telem_lon         = 0.0f;
float     telem_alt_m       = 0.0f;
float     telem_spd_ms      = 0.0f;
float     telem_airspeed_ms = 0.0f;
float     telem_hdg_deg     = 0.0f;
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
                telem_lat    = (float)mav_get_i32(p, 4)  / 1e7f;
                telem_lon    = (float)mav_get_i32(p, 8)  / 1e7f;
                telem_alt_m  = (float)mav_get_i32(p, 16) / 1000.0f;
                int16_t vx   = (int16_t)mav_get_u16(p, 20);
                int16_t vy   = (int16_t)mav_get_u16(p, 22);
                telem_spd_ms  = sqrtf((float)(vx*vx + vy*vy)) / 100.0f;
                uint16_t h   = mav_get_u16(p, 26);
                if (h != 65535) telem_hdg_deg = h / 100.0f;
                telem_gps_ok  = true;
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
            // VFR_HUD: float airspeed @0, float groundspeed @4, int16 heading @8,
            // uint16 throttle @10, float alt @12, float climb @16.
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
                    if (autopilot == 12) {
                        fw_type  = FW_INAV;
                        fw_label = "iNAV";
                    } else if (autopilot == 3) {
                        fw_type  = FW_ARDU;
                        fw_label = "ARDU";
                    }
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
// Serial status / telem print (replaces OLED HUD)
//-------------------------------------------------------

void serial_status(const char* l1, const char* l2 = "") {
    Serial.printf("[mLRS ESP-NOW] %s", l1);
    if (l2 && l2[0]) Serial.printf(" | %s", l2);
    Serial.println();
}

void serial_telem() {
    bool data_active = (millis() - telem_last_ms < 2000);
    const char* link =
        data_active ? "LINK OK" : (telem_ever_data ? "NO DATA" : "waiting");

    Serial.printf("[telem] %-6s %s sats=%d fw=%s ",
        get_mode_str(telem_flight_mode),
        telem_armed ? "ARM" : "dis",
        telem_satellites,
        fw_label);

    if (telem_gps_ok) {
        Serial.printf("lat=%.6f lon=%.6f ", telem_lat, telem_lon);
    } else {
        Serial.printf("fix=%dD ", telem_fix);
    }

    bool show_as = (fw_type == FW_ARDU) &&
                   (telem_vehicle_type == 1 || telem_vehicle_type == 19);
    if (show_as) {
        Serial.printf("alt=%.0fm as=%.0f gs=%.0f hdg=%d ",
            telem_alt_m, telem_airspeed_ms, telem_spd_ms, (int)telem_hdg_deg);
    } else {
        Serial.printf("alt=%.0fm gs=%.0f hdg=%d ",
            telem_alt_m, telem_spd_ms, (int)telem_hdg_deg);
    }

    if (telem_bat_pct >= 0)
        Serial.printf("bat=%.1fV %d%% ", telem_bat_v, telem_bat_pct);
    else
        Serial.printf("bat=%.1fV ", telem_bat_v);

    if (telem_rssi_pct >= 0)
        Serial.printf("rssi=%d%% %s\n", telem_rssi_pct, link);
    else
        Serial.printf("rssi=--- %s\n", link);
}


//-------------------------------------------------------
// ESP-NOW internals
//-------------------------------------------------------

#define ESPNOW_RXBUF_SIZE  2048
uint8_t espnow_rxbuf[ESPNOW_RXBUF_SIZE];
volatile uint16_t espnow_rxbuf_head = 0;
volatile uint16_t espnow_rxbuf_tail = 0;

void espnow_rxbuf_push(const uint8_t* data, int len) {
    for (int i = 0; i < len; i++) {
        uint16_t next = (espnow_rxbuf_head + 1) & (ESPNOW_RXBUF_SIZE - 1);
        if (next == espnow_rxbuf_tail) break;
        espnow_rxbuf[espnow_rxbuf_head] = data[i];
        espnow_rxbuf_head = next;
    }
}

int espnow_rxbuf_pop(uint8_t* buf, int maxlen) {
    int cnt = 0;
    while (espnow_rxbuf_tail != espnow_rxbuf_head && cnt < maxlen) {
        buf[cnt++] = espnow_rxbuf[espnow_rxbuf_tail];
        espnow_rxbuf_tail = (espnow_rxbuf_tail + 1) & (ESPNOW_RXBUF_SIZE - 1);
    }
    return cnt;
}

volatile bool espnow_latched_mac_available = false;
uint8_t espnow_latched_mac[6];
bool latched_peer_added = false;

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

uint8_t broadcast_mac[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint8_t scan_channels[] = { 1, 6, 11, 13 };

bool is_connected = false;
unsigned long is_connected_tlast_ms = 0;
bool wifi_initialized = false;
uint8_t buf[250];


//-------------------------------------------------------
// Channel scan
//-------------------------------------------------------

void scan_for_bridge(void) {
    if (espnow_latched_mac_available) {
        esp_now_del_peer(espnow_latched_mac);
        espnow_latched_mac_available = false;
        latched_peer_added = false;
    }

    serial_status("Scanning for", "mLRS bridge...");

    while (true) {
        for (int i = 0; i < (int)sizeof(scan_channels); i++) {
            esp_wifi_set_channel(scan_channels[i], WIFI_SECOND_CHAN_NONE);

            char ch_str[20];
            snprintf(ch_str, sizeof(ch_str), "Channel %d...", scan_channels[i]);
            serial_status("Scanning...", ch_str);

            unsigned long t = millis();
            while (millis() - t < 500) {
                while (Serial2.available()) Serial2.read();
                if (espnow_rxbuf_head != espnow_rxbuf_tail) {
                    char found_str[20];
                    snprintf(found_str, sizeof(found_str), "Ch:%d found!", scan_channels[i]);
                    serial_status("Bridge found!", found_str);
                    delay(800);
                    return;
                }
                delay(1);
            }
        }
    }
}


//-------------------------------------------------------
// WiFi / ESP-NOW init
//-------------------------------------------------------

void setup_wifi(void) {
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

    // Set TX power explicitly so this sketch is self-documenting and not
    // dependent on whatever the WiFi NVS happens to hold from a previous
    // flash (e.g. the UDP variant persists its 11 dBm cap to NVS).
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recv_cb);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, broadcast_mac, 6);
    esp_now_add_peer(&peer);

    scan_for_bridge();
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup() {
    Serial.begin(115200);

    Serial.println();
    Serial.println("---- mLRS ESP-NOW GCS Bridge (ESP32) ----");
    serial_status("Initializing...", "mLRS ESP-NOW GCS");
    delay(1500);

    // TX only - RX disabled. Wire a real RX pin if your Crossbow has
    // a back-channel and you want uplink to flow.
    Serial2.begin(CROSSBOW_BAUD, SERIAL_8N1, -1, TX_PIN);

    espnow_rxbuf_head = 0;
    espnow_rxbuf_tail = 0;
    espnow_latched_mac_available = false;
    latched_peer_added = false;
    is_connected = false;
    is_connected_tlast_ms = 0;
    wifi_initialized = false;
}


void loop() {
    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi();
        return;
    }

    unsigned long tnow_ms = millis();

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

    // Serial status refresh
    static unsigned long status_t = 0;
    if (tnow_ms - status_t >= STATUS_UPDATE_MS) {
        status_t = tnow_ms;
        if (is_connected) {
            serial_telem();
        } else {
            serial_status("Scanning for", "mLRS bridge...");
        }
    }

    delay(2);
}
