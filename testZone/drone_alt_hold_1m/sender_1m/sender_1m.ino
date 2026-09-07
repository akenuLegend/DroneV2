/*
 * SENDER_1M - tram mat dat toi gian cho DRONE_ALT_HOLD_1M.
 *
 * Khoi dong CHI gui heartbeat, tuyet doi khong tu ARM.
 * Lenh Serial 115200: HELP, ARM, TAKEOFF, LAND, DISARM, KILL YES,
 *                     HOVER <1050..1750>, STATUS.
 * Moi action duoc gui lai 1.2 s voi cung sequence; drone xu ly dung mot lan.
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ctype.h>

#define WIFI_CHANNEL 1
#define CMD_MAGIC 0xC3
#define CMD_VER 1
#define TELEM_MAGIC 0xD1
#define TELEM_VER 2
#define HEARTBEAT_MS 200UL
#define ACTION_REPEAT_MS 1200UL
#define LINK_TIMEOUT_MS 3000UL
#define SEND_TIMEOUT_MS 100UL

static uint8_t peerMac[6] = {0x3C, 0x8A, 0x1F, 0xA7, 0x8D, 0x40};
// Phai khop tung byte voi flight controller; doi ca hai ban truoc khi dung that.
static const uint8_t espnowPmk[ESP_NOW_KEY_LEN] = {
  0x44,0x52,0x4F,0x4E,0x45,0x2D,0x31,0x4D,0x2D,0x50,0x4D,0x4B,0x30,0x30,0x31,0x21
};
static const uint8_t espnowLmk[ESP_NOW_KEY_LEN] = {
  0x41,0x4C,0x54,0x2D,0x48,0x4F,0x4C,0x44,0x2D,0x4C,0x4D,0x4B,0x30,0x30,0x31,0x21
};

enum CommandCode : uint8_t {
  CMD_HEARTBEAT = 0, CMD_ARM, CMD_TAKEOFF, CMD_LAND,
  CMD_DISARM, CMD_KILL, CMD_SET_HOVER
};

struct __attribute__((packed)) ControlPacket {
  uint8_t magic, ver, command, reserved;
  uint16_t sequence, targetCm, hoverUs, crc;
};
static_assert(sizeof(ControlPacket) == 12, "ControlPacket phai la 12 byte");

struct __attribute__((packed)) TelemPacket {
  uint8_t magic, ver, mode, flags;
  uint32_t ms;
  uint16_t loopHz, seq;
  float pitch, roll, yaw;
  float pitchRate, rollRate, yawRate;
  float outP, outR, outY;
  float iPitch, iRoll, iYaw;
  float biasX, biasY, biasZ;
  float calBiasX, calBiasY, calBiasZ;
  float pitchOffset, rollOffset;
  float vibX, vibY, vibZ;
  float accW;
  int16_t m1, m2, m3, m4;
  int16_t throttle, hover, hoverTarget;
  uint16_t clip, rx, bad, lost;
  char reason[20];
  float altitudeM, verticalSpeedMps, altitudeSetpointM;
  float rangeAltitudeM, baroAltitudeM;
  int16_t altitudeCorrectionUs;
  uint16_t rangeAgeMs, baroAgeMs;
  uint8_t altitudeSource, rangeStrength, lastCommand, reserved;
};
static_assert(sizeof(TelemPacket) == 180, "TelemPacket khong khop drone");

struct RxMessage { uint8_t len; uint8_t data[200]; };
static QueueHandle_t rxQueue = NULL;
static volatile bool sendPending = false;
static volatile uint32_t txOk = 0, txFail = 0, rxDrop = 0;
static uint32_t lastSendMs = 0, lastTelemMs = 0, lastPrintMs = 0;
static uint32_t sendPendingSinceMs = 0;
static uint32_t lastNoTelemReportMs = 0;
static uint32_t lastLostReportMs = 0;
static uint16_t nextSequence = 1;
static uint16_t hoverUs = 1470;
static uint8_t repeatCommand = CMD_HEARTBEAT;
static uint16_t repeatSequence = 0;
static uint32_t repeatUntilMs = 0;
static TelemPacket latest = {};
static bool haveTelem = false;
static char inputLine[48] = {};
static uint8_t inputLength = 0;

static uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static const char *modeName(uint8_t mode) {
  switch (mode) {
    case 1: return "ARMED_IDLE";
    case 2: return "TAKEOFF";
    case 3: return "ALT_HOLD";
    case 4: return "LANDING";
    case 5: return "FAILSAFE";
    case 6: return "TRIPPED";
    default: return "DISARMED";
  }
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 1, 0)
void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
#else
void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  (void)mac;
#endif
  if (status == ESP_NOW_SEND_SUCCESS) txOk = txOk + 1; else txFail = txFail + 1;
  sendPending = false;
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (memcmp(mac, peerMac, 6) != 0 || len <= 0 || !rxQueue) return;
  RxMessage m;
  m.len = (uint8_t)min(len, (int)sizeof(m.data));
  memcpy(m.data, data, m.len);
  if (xQueueSend(rxQueue, &m, 0) != pdTRUE) rxDrop = rxDrop + 1;
}

static bool sendControl(uint8_t command, uint16_t sequence) {
  if (sendPending && (uint32_t)(millis() - sendPendingSinceMs) <= SEND_TIMEOUT_MS) return false;
  if (sendPending) {
    sendPending = false;
    Serial.println("[LINK] callback gui timeout; thu lai.");
  }
  ControlPacket p = {};
  p.magic = CMD_MAGIC; p.ver = CMD_VER; p.command = command;
  p.sequence = sequence; p.targetCm = 100; p.hoverUs = hoverUs;
  p.crc = crc16Ccitt((const uint8_t *)&p, sizeof(p) - sizeof(p.crc));
  const esp_err_t result = esp_now_send(peerMac, (const uint8_t *)&p, sizeof(p));
  lastSendMs = millis();
  if (result != ESP_OK) { txFail = txFail + 1; return false; }
  sendPending = true; sendPendingSinceMs = millis();
  return true;
}

static void queueAction(uint8_t command) {
  repeatCommand = command;
  repeatSequence = nextSequence++;
  repeatUntilMs = millis() + ACTION_REPEAT_MS;
  sendControl(repeatCommand, repeatSequence);
}

static void printHelp() {
  Serial.println("\nLenh an toan:");
  Serial.println("  ARM          kiem tra pre-arm 2s, motor chuyen ARMED_IDLE");
  Serial.println("  TAKEOFF      cat canh va giu 1.00m");
  Serial.println("  LAND         ha canh co dieu khien");
  Serial.println("  DISARM       chi duoc chap nhan khi ARMED_IDLE/da o dat");
  Serial.println("  KILL YES     tat khan cap; drone se khoa den khi reset");
  Serial.println("  HOVER n      dat base throttle khi DISARMED (1050..1750us)");
  Serial.println("  STATUS       in telemetry moi nhat");
}

static void printStatus() {
  if (!haveTelem) { Serial.println("[STATUS] Chua co telemetry tu drone."); return; }
  if ((uint32_t)(millis() - lastTelemMs) > LINK_TIMEOUT_MS) {
    Serial.printf("[STATUS] TELEMETRY STALE %.1fs - khong dung cac so lieu cu de ra lenh.\n",
                  (millis() - lastTelemMs) * 0.001f);
    return;
  }
  const char *source = latest.altitudeSource == 1 ? "MTF" :
                       latest.altitudeSource == 2 ? "BMP" : "NONE";
  Serial.printf("[%s] z=%+.3fm sp=%.3fm vz=%+.3fm/s src=%s corr=%+dus | "
                "P=%+.1f R=%+.1f T=%d H=%d M=%d/%d/%d/%d | rangeAge=%ums "
                "baroAge=%ums str=%u | %s\n",
                modeName(latest.mode), latest.altitudeM, latest.altitudeSetpointM,
                latest.verticalSpeedMps, source, latest.altitudeCorrectionUs,
                latest.pitch, latest.roll, latest.throttle, latest.hover,
                latest.m1, latest.m2, latest.m3, latest.m4,
                latest.rangeAgeMs, latest.baroAgeMs, latest.rangeStrength,
                latest.reason);
}

static void uppercase(char *s) {
  for (; *s; s++) *s = (char)toupper((unsigned char)*s);
}

static bool telemetryFresh() {
  return haveTelem && (uint32_t)(millis() - lastTelemMs) <= LINK_TIMEOUT_MS;
}

static void processLine() {
  inputLine[inputLength] = '\0'; inputLength = 0;
  while (inputLine[0] == ' ') memmove(inputLine, inputLine + 1, strlen(inputLine));
  uppercase(inputLine);

  if (!strcmp(inputLine, "HELP") || !strcmp(inputLine, "H") || !strcmp(inputLine, "?"))
    printHelp();
  else if (!strcmp(inputLine, "STATUS") || !strcmp(inputLine, "S")) printStatus();
  else if (!strcmp(inputLine, "ARM")) {
    if (!telemetryFresh() || latest.mode != 0 || latest.altitudeSource != 1 ||
        latest.rangeAgeMs > 250 || latest.baroAgeMs > 250)
      Serial.println("[TX] ARM bi chan: can telemetry moi, DISARMED, MTF+BMP san sang.");
    else { Serial.println("[TX] ARM (drone van phai dat pre-arm checks)"); queueAction(CMD_ARM); }
  } else if (!strcmp(inputLine, "TAKEOFF")) {
    if (!telemetryFresh() || latest.mode != 1 || latest.altitudeSource != 1 ||
        latest.rangeAgeMs > 250 || latest.baroAgeMs > 250)
      Serial.println("[TX] TAKEOFF bi chan: can telemetry moi + ARMED_IDLE + MTF/BMP.");
    else { Serial.println("[TX] TAKEOFF -> 1.00m"); queueAction(CMD_TAKEOFF); }
  } else if (!strcmp(inputLine, "LAND")) {
    Serial.println("[TX] LAND"); queueAction(CMD_LAND);
  } else if (!strcmp(inputLine, "DISARM")) {
    Serial.println("[TX] DISARM"); queueAction(CMD_DISARM);
  } else if (!strcmp(inputLine, "KILL YES")) {
    Serial.println("[TX] !!! KILL KHAN CAP !!!"); queueAction(CMD_KILL);
  } else if (!strncmp(inputLine, "HOVER ", 6)) {
    char *end = NULL; const long value = strtol(inputLine + 6, &end, 10);
    if (*end || value < 1050 || value > 1750) Serial.println("[TX] HOVER khong hop le.");
    else if (!telemetryFresh() || latest.mode != 0)
      Serial.println("[TX] HOVER bi chan: can telemetry moi va drone DISARMED.");
    else { hoverUs = (uint16_t)value; Serial.printf("[TX] HOVER %u\n", hoverUs); queueAction(CMD_SET_HOVER); }
  } else if (inputLine[0]) Serial.println("[CMD] Khong hieu. Go HELP.");
}

static void serviceSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') { if (inputLength) processLine(); }
    else if (inputLength < sizeof(inputLine) - 1) inputLine[inputLength++] = c;
  }
}

static void serviceTelemetry() {
  RxMessage m;
  while (xQueueReceive(rxQueue, &m, 0) == pdTRUE) {
    if (m.len != sizeof(TelemPacket)) continue;
    TelemPacket p; memcpy(&p, m.data, sizeof(p));
    if (p.magic != TELEM_MAGIC || p.ver != TELEM_VER) continue;
    p.reason[sizeof(p.reason) - 1] = '\0'; latest = p;
    haveTelem = true; lastTelemMs = millis();
  }
}

void setup() {
  Serial.begin(115200); delay(300);
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("\n=== SENDER_1M - safe ground station ===");
  Serial.printf("MAC sender: %s\nDrone: %02X:%02X:%02X:%02X:%02X:%02X\n",
                WiFi.macAddress().c_str(), peerMac[0], peerMac[1], peerMac[2],
                peerMac[3], peerMac[4], peerMac[5]);
  rxQueue = xQueueCreate(6, sizeof(RxMessage));
  if (!rxQueue || esp_now_init() != ESP_OK) {
    Serial.println("[FATAL] ESP-NOW/queue init that bai."); while (true) delay(500);
  }
  if (esp_now_set_pmk(espnowPmk) != ESP_OK) {
    Serial.println("[FATAL] Khong dat duoc ESP-NOW PMK."); while (true) delay(500);
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, peerMac, 6); peer.channel = WIFI_CHANNEL;
  memcpy(peer.lmk, espnowLmk, ESP_NOW_KEY_LEN); peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[FATAL] Khong add duoc drone peer."); while (true) delay(500);
  }
  Serial.println("Khoi dong an toan: chi HEARTBEAT, khong ARM. Go HELP.");
}

void loop() {
  serviceSerial();
  serviceTelemetry();
  const uint32_t now = millis();
  if ((int32_t)(repeatUntilMs - now) > 0) {
    if ((uint32_t)(now - lastSendMs) >= HEARTBEAT_MS) sendControl(repeatCommand, repeatSequence);
  } else {
    repeatCommand = CMD_HEARTBEAT;
    if ((uint32_t)(now - lastSendMs) >= HEARTBEAT_MS) sendControl(CMD_HEARTBEAT, nextSequence++);
  }
  if (haveTelem && (uint32_t)(now - lastPrintMs) >= 500UL) {
    lastPrintMs = now; printStatus();
  }
  if (haveTelem && (uint32_t)(now - lastTelemMs) > LINK_TIMEOUT_MS &&
      (uint32_t)(now - lastLostReportMs) > LINK_TIMEOUT_MS) {
    Serial.println("[LINK] MAT TELEMETRY >3s; drone se tu vao failsafe neu dang bay.");
    lastLostReportMs = now;
  }
  if (!haveTelem && now > LINK_TIMEOUT_MS &&
      (uint32_t)(now - lastNoTelemReportMs) > LINK_TIMEOUT_MS) {
    lastNoTelemReportMs = now;
    Serial.println("[LINK] CHUA CO TELEMETRY; kiem tra MAC, key, channel va nguon drone.");
  }
}
