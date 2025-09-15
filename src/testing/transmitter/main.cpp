// Transmitter for XIAO ESP32S3 (Arduino-ESP32 v3.x / IDF v5)
// - Type commands over Serial to send test motions to the car over ESP-NOW.
// - Prints incoming telemetry (yaw, gz) from the receiver.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>   // esp_wifi_get_mac, esp_wifi_set_channel

#define ESPNOW_CHANNEL 1      // Try channel 1 instead of 6
#define SERIAL_BAUD   115200

// ------------ Message formats ------------
#pragma pack(push, 1)
struct CmdMsg {
  uint32_t seq;         // sequence id
  float uL;             // left command [-1..1]
  float uR;             // right command [-1..1]
  uint32_t duration_ms; // 0 = continuous until new cmd
  uint8_t reset_yaw;    // 1 = reset yaw to 0
  uint8_t recal_gyro;   // 1 = recalibrate gyro bias
};

struct TelemetryMsg {
  uint32_t seq_rx;      // receiver's seq counter
  uint32_t millis_rx;   // receiver milliseconds
  float yaw_deg;        // integrated yaw (deg)
  float gz_dps;         // instantaneous gyro z (deg/s)
  float uL_applied;     // applied on receiver
  float uR_applied;
};
#pragma pack(pop)

// ------------ Globals ------------
uint8_t peerMac[6] = {0x8C,0xBF,0xEA,0x8E,0x59,0x98}; // Default to receiver MAC
volatile uint32_t txSeq = 1;

// ------------ Helpers ------------
static void printMac(const uint8_t mac[6]) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
static void getStaMac(uint8_t out[6]) {
  esp_wifi_get_mac(WIFI_IF_STA, out);
}
static void printMyMac() {
  uint8_t mac[6]; getStaMac(mac);
  Serial.print("My MAC: "); printMac(mac); Serial.println();
}

static void printWiFiChannel() {
  uint8_t primary;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  Serial.printf("Current WiFi Channel: %d\n", primary);
}

// ------------ ESP-NOW callbacks ------------
static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("[TX] Send success");
  } else {
    Serial.printf("[TX] Send failed: %d\n", status);
  }
}

static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == (int)sizeof(TelemetryMsg)) {
    TelemetryMsg t;
    memcpy(&t, data, sizeof(t));
    Serial.printf("[TEL] rxSeq=%lu  t=%lums  yaw=%.2f°  gz=%.2f dps  uL=%.2f  uR=%.2f\n",
                  (unsigned long)t.seq_rx, (unsigned long)t.millis_rx,
                  t.yaw_deg, t.gz_dps, t.uL_applied, t.uR_applied);
  } else {
    Serial.printf("[TEL] %d bytes\n", len);
  }
}

static bool addPeer() {
  // First verify our own channel
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  
  if (current_channel != ESPNOW_CHANNEL) {
    Serial.printf("[ERROR] WiFi channel mismatch! Current: %d, Expected: %d\n", current_channel, ESPNOW_CHANNEL);
    Serial.println("Attempting to fix channel...");
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(10);
    esp_wifi_get_channel(&current_channel, &second);
    if (current_channel != ESPNOW_CHANNEL) {
      Serial.printf("[ERROR] Channel fix failed! Still on channel %d\n", current_channel);
      return false;
    }
  }

  esp_now_peer_info_t p;
  memset(&p, 0, sizeof(p));
  memcpy(p.peer_addr, peerMac, 6);
  p.channel = ESPNOW_CHANNEL;
  p.encrypt = false;

  Serial.printf("[DEBUG] Current WiFi channel: %d, Peer channel: %d\n", current_channel, p.channel);

  // Re-add in case it already exists with different settings
  esp_now_del_peer(peerMac);
  esp_err_t result = esp_now_add_peer(&p);
  if (result == ESP_OK) {
    Serial.print("Peer added: "); printMac(peerMac); Serial.println();
    return true;
  }
  Serial.printf("Failed to add peer. Error: %d (%s)\n", result, esp_err_to_name(result));
  return false;
}

// ------------ TX ------------
static void sendCmd(float uL, float uR, uint32_t dur_ms, uint8_t reset_yaw = 0, uint8_t recal_gyro = 0) {
  CmdMsg m;
  memset(&m, 0, sizeof(m));
  m.seq = txSeq++;
  m.uL = constrain(uL, -1.0f, 1.0f);
  m.uR = constrain(uR, -1.0f, 1.0f);
  m.duration_ms = dur_ms;
  m.reset_yaw = reset_yaw;
  m.recal_gyro = recal_gyro;
  
  esp_err_t result = esp_now_send(peerMac, (uint8_t*)&m, sizeof(m));
  if (result != ESP_OK) {
    Serial.printf("[TX ERROR] esp_now_send failed: %d (%s)\n", result, esp_err_to_name(result));
  }
  
  Serial.printf("[TX] seq=%lu uL=%.2f uR=%.2f dur=%lums reset_yaw=%u recal_gyro=%u\n",
                (unsigned long)m.seq, m.uL, m.uR, (unsigned long)m.duration_ms, m.reset_yaw, m.recal_gyro);
}

// ------------ Setup/Loop ------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println("\n=== ESP-NOW TRANSMITTER ===");

  // More robust WiFi initialization sequence for ESP32-C6
  WiFi.mode(WIFI_MODE_NULL);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(500); // Longer delay for ESP32-C6
  
  // Ensure WiFi is started
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  delay(100);

  // Print my MAC (IDF v5 way)
  printMyMac();

  // Force Wi-Fi channel BEFORE ESP-NOW init (critical for ESP32-C6)
  esp_err_t channel_result = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Channel set result: %d (%s)\n", channel_result, esp_err_to_name(channel_result));
  delay(100); // Give time for channel to settle
  
  // Verify channel was set correctly
  uint8_t current_channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&current_channel, &second);
  Serial.printf("WiFi channel set to: %d (target: %d)\n", current_channel, ESPNOW_CHANNEL);
  
  if (current_channel != ESPNOW_CHANNEL) {
    Serial.printf("WARNING: Channel mismatch! Expected %d, got %d\n", ESPNOW_CHANNEL, current_channel);
    // Try setting again
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);
    esp_wifi_get_channel(&current_channel, &second);
    Serial.printf("After retry - WiFi channel: %d\n", current_channel);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed! Rebooting...");
    delay(1000); ESP.restart();
  }
  
  // Verify channel after ESP-NOW init
  esp_wifi_get_channel(&current_channel, &second);
  Serial.printf("WiFi channel after ESP-NOW init: %d\n", current_channel);
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Automatically add the default peer
  addPeer();

  Serial.println("Type HELP for commands.");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c=='\r') continue;
    if (c=='\n') {
      line.trim();
      if (line.length()) {
        // Commands:
        // HELP
        // MAC           -> print my MAC
        // PEER?         -> print current peer MAC (if set)
        // PEER xx:xx:xx:xx:xx:xx
        // S
        // F <u> <ms>
        // T <u> <ms>    -> turn in place; left if u>0, right if u<0
        // A <uL> <uR> <ms>
        // R <ms>
        // Y             -> reset yaw to 0
        // C             -> recalibrate gyro

        if (line.equalsIgnoreCase("HELP")) {
          Serial.println("Commands:");
          Serial.println("  MAC      -> print my MAC address");
          Serial.println("  CHANNEL  -> print current WiFi channel");
          Serial.println("  PEER?    -> print current peer MAC");
          Serial.println("  PEER xx:xx:xx:xx:xx:xx -> set peer MAC");
          Serial.println("  S        -> stop");
          Serial.println("  F <u 0..1> <ms>       -> forward");
          Serial.println("  T <u -1..1> <ms>      -> turn");
          Serial.println("  A <uL -1..1> <uR -1..1> <ms> -> arbitrary");
          Serial.println("  R <ms>   -> rest/ping");
          Serial.println("  Y        -> reset yaw to 0 degrees");
          Serial.println("  C        -> recalibrate gyro bias");
        } else if (line.equalsIgnoreCase("MAC")) {
          printMyMac();
        } else if (line.equalsIgnoreCase("CHANNEL")) {
          printWiFiChannel();
        } else if (line.equalsIgnoreCase("PEER?")) {
          Serial.print("Peer: ");
          if (peerMac[0]==0xFF && peerMac[1]==0xFF && peerMac[2]==0xFF &&
              peerMac[3]==0xFF && peerMac[4]==0xFF && peerMac[5]==0xFF) {
            Serial.println("(not set)");
          } else { printMac(peerMac); Serial.println(); }
        } else if (line.startsWith("PEER")) {
          int sp = line.indexOf(' ');
          if (sp>0) {
            String macs = line.substring(sp+1);
            unsigned int parts[6];
            int cnt = sscanf(macs.c_str(), "%x:%x:%x:%x:%x:%x",
                             &parts[0],&parts[1],&parts[2],&parts[3],&parts[4],&parts[5]);
            if (cnt==6) {
              for (int i=0;i<6;i++) peerMac[i]=(uint8_t)parts[i];
              Serial.print("Peer set to "); printMac(peerMac); Serial.println();
            } else {
              Serial.println("Bad MAC format. Use PEER xx:xx:xx:xx:xx:xx");
            }
          } else {
            Serial.println("Usage: PEER xx:xx:xx:xx:xx:xx");
          }
        } else if (line.equalsIgnoreCase("S")) {
          sendCmd(0,0,0);
        } else if (line.startsWith("F ")) {
          float u; uint32_t ms;
          if (sscanf(line.c_str(),"F %f %lu",&u,&ms)==2) sendCmd(u,u,ms);
          else Serial.println("Usage: F <u 0..1> <ms>");
        } else if (line.startsWith("T ")) {
          float u; uint32_t ms;
          if (sscanf(line.c_str(),"T %f %lu",&u,&ms)==2) sendCmd(-u,+u,ms);
          else Serial.println("Usage: T <u -1..1> <ms>");
        } else if (line.startsWith("A ")) {
          float uL,uR; uint32_t ms;
          if (sscanf(line.c_str(),"A %f %f %lu",&uL,&uR,&ms)==3) sendCmd(uL,uR,ms);
          else Serial.println("Usage: A <uL -1..1> <uR -1..1> <ms>");
        } else if (line.startsWith("R ")) {
          uint32_t ms;
          if (sscanf(line.c_str(),"R %lu",&ms)==1) sendCmd(0,0,ms); // ping with duration payload
          else Serial.println("Usage: R <ms>");
        } else if (line.equalsIgnoreCase("Y")) {
          sendCmd(0,0,0,1); // reset yaw command
          Serial.println("Yaw reset command sent");
        } else if (line.equalsIgnoreCase("C")) {
          sendCmd(0,0,0,0,1); // recalibrate gyro command
          Serial.println("Gyro recalibration command sent");
        } else {
          Serial.println("Unknown. Type HELP.");
        }
      }
      line = "";
    } else {
      line += c;
    }
  }
}
