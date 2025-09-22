// Self-Driving Car Telemetry Receiver
// - Receives telemetry from self-driving car via ESP-NOW
// - Forwards telemetry data over serial for robot_control.py

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>   // esp_wifi_get_mac, esp_wifi_set_channel

#define ESPNOW_CHANNEL 6      // Match self-driving car channel
#define SERIAL_BAUD   115200

// ------------ Message formats ------------
#pragma pack(push, 1)
struct TelemetrySample {
  uint32_t millis_sent;
  float sensor_left;      // VL6180X left distance (normalized)
  float sensor_center;    // VL6180X center distance (normalized)
  float sensor_right;     // VL6180X right distance (normalized)
  float nn_output_left;   // Neural network left wheel command
  float nn_output_right;  // Neural network right wheel command
  float yaw_deg;          // Integrated yaw angle
  int16_t accel[3];       // Raw accelerometer: X, Y, Z
  int16_t gyro[3];        // Raw gyroscope: X, Y, Z
};
struct TelemetryMsg {
  uint32_t seq_tx;
  uint8_t sample_count;
  TelemetrySample samples[5];
};
#pragma pack(pop)

// ------------ Globals ------------
uint8_t selfDrivingCarMac[6] = {0x8C, 0xBF, 0xEA, 0x8E, 0x59, 0x98}; // Default to self-driving car MAC

// ------------ Helpers ------------
static void printMac(const uint8_t mac[6]) {
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
}
static void getStaMac(uint8_t out[6]) {
  esp_wifi_get_mac(WIFI_IF_STA, out);
}
static void printMyMac() {
  uint8_t mac[6];
  getStaMac(mac);
  Serial.print("My MAC: ");
  printMac(mac);
  Serial.println();
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

    // Print each sample in the batch as individual [TEL] messages to match robot_control.py format
    for (uint8_t i = 0; i < t.sample_count && i < 5; i++) {
      Serial.printf("[TEL] rxSeq=%lu  t=%lums  sl=%.3f sc=%.3f sr=%.3f  nl=%.3f nr=%.3f  yaw=%.2f°  ax=%d ay=%d az=%d  gx=%d gy=%d gz=%d\n",
                    (unsigned long)t.seq_tx, (unsigned long)t.samples[i].millis_sent,
                    t.samples[i].sensor_left, t.samples[i].sensor_center, t.samples[i].sensor_right,
                    t.samples[i].nn_output_left, t.samples[i].nn_output_right,
                    t.samples[i].yaw_deg,
                    t.samples[i].accel[0], t.samples[i].accel[1], t.samples[i].accel[2],
                    t.samples[i].gyro[0], t.samples[i].gyro[1], t.samples[i].gyro[2]);
    }
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
  }

  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, selfDrivingCarMac, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  peer_info.ifidx = WIFI_IF_STA;  // Specify interface

  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("Failed to add peer");
    return false;
  }
  return true;
}

// ------------ Setup ------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println("=== SELF-DRIVING CAR TELEMETRY RECEIVER ===");

  // 1) WiFi/ESP-NOW setup
  WiFi.mode(WIFI_MODE_NULL);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(500);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_start();
  delay(100);

  printMyMac();

  // Set WiFi channel
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  if (esp_now_init()!=ESP_OK) {
    Serial.println("ESP-NOW init FAILED, rebooting...");
    delay(1000); ESP.restart();
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  if (!addPeer()) {
    Serial.println("Failed to add self-driving car as peer");
  }

  Serial.println("Ready to receive telemetry from self-driving car");
}

// ------------ Main loop ------------
void loop() {
  // Optional local commands
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.equalsIgnoreCase("MAC")) {
      printMyMac();
    }
  }
}