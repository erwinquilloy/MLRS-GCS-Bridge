//*******************************************************
// mLRS WiFi-UDP GCS Bridge - Heltec WiFi Kit 32 v1/v2
// Receives MAVLink from mLRS Nomad via WiFi UDP and
// forwards to MFD Mini Crossbow via UART (Serial2)
// Also displays telemetry on built-in OLED display
//
// Sibling of the ESP-NOW variant. The Heltec joins the
// Nomad's SoftAP and listens on UDP for MAVLink. Multiple
// GCS clients can attach to the same AP simultaneously
// (e.g. phone running QGC + this bridge).
//
// Uses Adafruit SSD1306 + Adafruit GFX (not heltec.h)
// Compatible with Heltec WiFi Kit 32 v1/v2 (micro-USB)
//
// Libraries required (Library Manager):
//   - Adafruit SSD1306
//   - Adafruit GFX Library
//
// Board: Tools -> Board -> ESP32 Arduino -> Heltec WiFi Kit 32
// ESP32 Arduino core >= 3.0.0
//
// Wiring to MFD Mini Crossbow:
//   Heltec GPIO17 (TX2)  -->  Crossbow MAVLink input pin
//   Heltec GND           -->  Crossbow GND
//   Power Heltec via micro-USB
//
// License: GPL v3
//*******************************************************

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//-------------------------------------------------------
// USER CONFIGURATION
//-------------------------------------------------------

// Set WIFI_SSID to your Nomad's SoftAP name. mLRS generates it from the
// device MAC, so it looks like "mLRS-xxxx AP UDP" - check your Tx OLED or
// CLI for the exact value. UDP mode has no password by default, so leave
// WIFI_PASS empty unless you set one on the Nomad.
#define WIFI_SSID       ""
#define WIFI_PASS       ""
#define UDP_PORT        14550

#define CROSSBOW_BAUD   115200
#define TX_PIN          17    // GPIO17 -> Crossbow MAVLink input
// RX disabled (-1) so GPIO16 stays free for OLED reset

#define OLED_UPDATE_MS  200

// OLED pins - Heltec WiFi Kit 32 v1/v2
#define OLED_SDA        4
#define OLED_SCL        15
#define OLED_RST        16
#define OLED_ADDR       0x3C

Adafruit_SSD1306 display(128, 64, &Wire, OLED_RST);

WiFiUDP udp;


//-------------------------------------------------------
// Minimal MAVLink parser (no library needed)
//-------------------------------------------------------

#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_SYS_STATUS           1
#define MAVLINK_MSG_ID_GPS_RAW_INT          24
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33
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
float     telem_hdg_deg     = 0.0f;
int       telem_satellites  = 0;
uint8_t   telem_fix         = 0;
float     telem_bat_v       = 0.0f;
int       telem_bat_pct     = -1;
int       telem_rssi_pct    = -1;
uint32_t  telem_flight_mode = 0;
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
// OLED display - Adafruit SSD1306
//-------------------------------------------------------

void oled_init() {
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
    display.setCursor(0, 0);  display.println("mLRS UDP GCS");
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

    display.printf("Alt:%4.0fm GS:%3.0f\n", telem_alt_m, telem_spd_ms);
    display.printf("Hdg:%3d deg\n", (int)telem_hdg_deg);

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
// WiFi UDP link state
//-------------------------------------------------------

bool          udp_peer_latched      = false;
IPAddress     udp_peer_ip;
uint16_t      udp_peer_port         = 0;

bool          is_connected          = false;
unsigned long is_connected_tlast_ms = 0;
bool          wifi_initialized      = false;

uint8_t       buf[1024];


//-------------------------------------------------------
// WiFi join + UDP bring-up
//-------------------------------------------------------

void setup_wifi_udp(void) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(50);

    char l2[24];
    snprintf(l2, sizeof(l2), "SSID:%s", WIFI_SSID);
    oled_status("Connecting...", l2);

    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long t0 = millis();
    int dots = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        char prog[24];
        snprintf(prog, sizeof(prog), "Connecting%.*s",
                 (dots % 4) + 1, "....");
        oled_status(prog, l2);
        dots++;

        // After ~15 s with no link, blink a hint and keep trying.
        if (millis() - t0 > 15000) {
            oled_status("WiFi timeout", "retrying...");
            delay(800);
            WiFi.disconnect();
            delay(200);
            WiFi.begin(WIFI_SSID, WIFI_PASS);
            t0 = millis();
        }
    }

    udp.begin(UDP_PORT);

    IPAddress ip = WiFi.localIP();
    char l1[24], l2b[24];
    snprintf(l1, sizeof(l1), "IP:%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    snprintf(l2b, sizeof(l2b), "UDP:%d waiting", UDP_PORT);
    oled_status(l1, l2b);
}


//-------------------------------------------------------
// setup() and loop()
//-------------------------------------------------------

void setup() {
    Serial.begin(115200);

    oled_init();
    oled_status("Initializing...", "mLRS UDP GCS");
    delay(1500);

    // TX only - RX disabled so GPIO16 stays free for OLED reset
    Serial2.begin(CROSSBOW_BAUD, SERIAL_8N1, -1, TX_PIN);

    udp_peer_latched      = false;
    is_connected          = false;
    is_connected_tlast_ms = 0;
    wifi_initialized      = false;
}


void loop() {
    if (!wifi_initialized) {
        wifi_initialized = true;
        setup_wifi_udp();
        return;
    }

    // If the AP drops, reconnect.
    if (WiFi.status() != WL_CONNECTED) {
        udp.stop();
        udp_peer_latched = false;
        is_connected     = false;
        telem_gps_ok     = false;
        wifi_initialized = false;
        return;
    }

    unsigned long tnow_ms = millis();

    // Connection timeout (no packets for 2 s)
    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) {
        is_connected = false;
        telem_gps_ok = false;
        oled_status("UDP idle", "waiting...");
    }

    // Downlink: drain all pending UDP packets this tick.
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

    // Uplink: Crossbow -> UDP -> vehicle. Only send once we know
    // where to send it.
    int rlen = 0;
    while (Serial2.available() && rlen < (int)sizeof(buf)) {
        buf[rlen++] = Serial2.read();
    }
    if (rlen > 0 && udp_peer_latched) {
        udp.beginPacket(udp_peer_ip, udp_peer_port);
        udp.write(buf, rlen);
        udp.endPacket();
    }

    // OLED refresh
    static unsigned long oled_t = 0;
    if (tnow_ms - oled_t >= OLED_UPDATE_MS) {
        oled_t = tnow_ms;
        if (is_connected) {
            oled_telem();
        } else {
            char l1[24], l2[24];
            IPAddress ip = WiFi.localIP();
            snprintf(l1, sizeof(l1), "IP:%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            snprintf(l2, sizeof(l2), "UDP:%d waiting", UDP_PORT);
            oled_status(l1, l2);
        }
    }

    delay(2);
}
