// Receiver for Seeed XIAO ESP32S3 (Arduino-ESP32 v3.x, IDF v5)
// - MCPWM (prelude API) @ 20 kHz driving DRV8833 (4 pins: L_FWD/L_REV, R_FWD/R_REV)
// - Control & IMU loop at 200 Hz using esp_timer
// - ESP-NOW commands in, telemetry (yaw, gz, applied uL/uR) out
// - MPU6050 yaw integration with 1.5 s bias calibration
// - Motor gain correction for straight-line driving (r = 1.02699)

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>

#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"

// --------------- CONFIG ----------------
#define SERIAL_BAUD     115200

#define ESPNOW_CHANNEL  6  // Changed from 1 to 6 (less crowded channel)
#define TEL_HZ          50  // Reduced from 50 to 20 Hz for more reliable transmission
#define CTRL_HZ         500

// DRV8833 pins (XIAO ESP32S3)
#define L_MOT_FWD       3
#define L_MOT_REV       4
#define R_MOT_FWD       2
#define R_MOT_REV       1

// PWM
#define PWM_FREQ_HZ     20000       // 20 kHz
#define PWM_RES_HZ      1000000     // 1 MHz MCPWM timer resolution

// MPU6050
#define MPU_ADDR        0x68
static const float GYRO_SENS_DPS = 16.4f; // FS_SEL=3 (±2000 dps)
static const float GYRO_GAIN_CORR = 1.0127f; // Gain correction: 90°/(90°-4.59°)
#define GYRO_CAL_MS     4000

// Motor gain correction to make car go straight
static const float GAIN_RATIO_R = 1.03f; // r = Kr/Kl (calculated from calibration)
static const float GAIN_LEFT = (1.0f + GAIN_RATIO_R) / 2.0f;
static const float GAIN_RIGHT = (1.0f + (1.0f / GAIN_RATIO_R)) / 2.0f;
// --------------------------------------

// ---- Messages ----
#pragma pack(push,1)
struct CmdMsg {
  uint32_t seq;
  float uL;
  float uR;
  uint32_t duration_ms; // 0 = continuous
  uint8_t reset_yaw;    // 1 = reset yaw to 0
  uint8_t recal_gyro;   // 1 = recalibrate gyro bias
};
struct TelemetrySample {
  uint32_t millis_rx;
  float yaw_deg;
  float uL_applied;
  float uR_applied;
  // Full IMU data
  int16_t accel[3];  // Raw accelerometer: X, Y, Z
  int16_t gyro[3];   // Raw gyroscope: X, Y, Z
};
struct TelemetryMsg {
  uint32_t seq_rx;
  uint8_t sample_count;  // Number of batched samples
  TelemetrySample samples[5];  // Batch up to 5 samples (120 bytes total)
};
#pragma pack(pop)

// ---- Globals ----
static uint8_t g_peer_mac[6] = {0x54, 0x32, 0x04, 0x21, 0x80, 0x28};

static volatile float g_uL = 0.0f, g_uR = 0.0f;
static volatile uint32_t g_cmd_expire_ms = 0;

static volatile float g_yaw_deg = 0.0f;
static float g_gz_bias = 0.0f;
static bool  g_imu_ready = false;
static uint32_t g_imu_calib_end_ms = 0;
static bool g_recalibrating = false;

static esp_timer_handle_t g_ctrl_timer = nullptr;
static volatile bool g_ctrl_tick = false;

static uint32_t g_last_tel_ms = 0;

// ---- Telemetry batching ----
#define BATCH_SIZE 5  // Send every 3 samples (every 150ms instead of 50ms)
static TelemetrySample g_telemetry_buffer[BATCH_SIZE];
static uint8_t g_batch_index = 0;
static uint32_t g_batch_seq = 1;

// MCPWM handles
static mcpwm_timer_handle_t   pwm_timer   = nullptr;
static mcpwm_oper_handle_t    op_left     = nullptr;
static mcpwm_oper_handle_t    op_right    = nullptr;
static mcpwm_cmpr_handle_t    cmp_l_fwd   = nullptr;
static mcpwm_cmpr_handle_t    cmp_l_rev   = nullptr;
static mcpwm_cmpr_handle_t    cmp_r_fwd   = nullptr;
static mcpwm_cmpr_handle_t    cmp_r_rev   = nullptr;
static mcpwm_gen_handle_t     gen_l_fwd   = nullptr;
static mcpwm_gen_handle_t     gen_l_rev   = nullptr;
static mcpwm_gen_handle_t     gen_r_fwd   = nullptr;
static mcpwm_gen_handle_t     gen_r_rev   = nullptr;

static uint32_t period_ticks = 0;

// ---- Utils ----
static inline uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi){ return x<lo?lo:(x>hi?hi:x); }
static inline float    clamp_f  (float    x, float    lo, float    hi){ return x<lo?lo:(x>hi?hi:x); }

// ---- IMU low-level ----
static int16_t mpu_read16(uint8_t reg_high) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg_high);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}
static void mpu_read_accel_gyro_burst(int16_t* accel, int16_t* gyro) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true); // 6 accel + 2 temp + 6 gyro bytes
  uint8_t data[14];
  for (int i = 0; i < 14; i++) {
    data[i] = Wire.read();
  }
  // Accel: X, Y, Z (registers 0x3B-0x40)
  accel[0] = (data[0] << 8) | data[1]; // ACCEL_X
  accel[1] = (data[2] << 8) | data[3]; // ACCEL_Y
  accel[2] = (data[4] << 8) | data[5]; // ACCEL_Z
  // Skip temperature (data[6-7])
  // Gyro: X, Y, Z (registers 0x43-0x48)
  gyro[0] = (data[8] << 8) | data[9];   // GYRO_X
  gyro[1] = (data[10] << 8) | data[11]; // GYRO_Y
  gyro[2] = (data[12] << 8) | data[13]; // GYRO_Z
}
static bool mpu_init() {
  Wire.begin();
  Wire.setClock(400000);
  delay(50);
  // Wake up
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00);
  if (Wire.endTransmission()!=0) return false;
  delay(5);
  // Gyro FS = +/-2000 dps (maximum range for extreme turns)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x18);
  Wire.endTransmission();

  g_imu_calib_end_ms = millis() + GYRO_CAL_MS;
  g_gz_bias = 0.0f;
  g_imu_ready = false;
  return true;
}

// Blocking calibration: compute gyro Z bias for the specified duration
static void calibrate_gyro_bias_blocking(uint32_t ms)
{
  Serial.printf("Calibrating gyro bias for %lu ms...\n", (unsigned long)ms);
  const uint32_t t_end = millis() + ms;
  double acc = 0.0;
  uint32_t n = 0;
  uint32_t last_print = millis();
  while ((int32_t)(t_end - millis()) > 0) {
    int16_t gz_raw = mpu_read16(0x47); // GYRO_ZOUT_H
    float gz_dps = -(float)gz_raw / GYRO_SENS_DPS; // sign for turn direction
    acc += (double)gz_dps;
    n++;
    // Light pacing ~1 kHz depending on I2C speed; no delay needed but add a tiny one
    delayMicroseconds(500);
    if (millis() - last_print >= 1000) {
      Serial.print(".");
      last_print = millis();
    }
  }
  g_gz_bias = (n > 0) ? (float)(acc / (double)n) : 0.0f;
  g_imu_ready = true;
  Serial.printf("\nGyro calibration complete. Bias: %.3f dps (n=%lu)\n", g_gz_bias, (unsigned long)n);
}

// ---- MCPWM setup (prelude API) ----
static void mcpwm_setup() {
  mcpwm_timer_config_t tcfg = {};
  tcfg.group_id       = 0;
  tcfg.clk_src        = MCPWM_TIMER_CLK_SRC_DEFAULT;
  tcfg.resolution_hz  = PWM_RES_HZ;                     // 1 MHz
  tcfg.period_ticks   = PWM_RES_HZ / PWM_FREQ_HZ;       // ticks per period
  tcfg.count_mode     = MCPWM_TIMER_COUNT_MODE_UP;
  ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &pwm_timer));
  period_ticks = tcfg.period_ticks;

  mcpwm_operator_config_t ocfg = {};
  ocfg.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &op_left));
  ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &op_right));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op_left,  pwm_timer));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op_right, pwm_timer));

  mcpwm_comparator_config_t ccfg = {};
  ccfg.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(op_left,  &ccfg, &cmp_l_fwd));
  ESP_ERROR_CHECK(mcpwm_new_comparator(op_left,  &ccfg, &cmp_l_rev));
  ESP_ERROR_CHECK(mcpwm_new_comparator(op_right, &ccfg, &cmp_r_fwd));
  ESP_ERROR_CHECK(mcpwm_new_comparator(op_right, &ccfg, &cmp_r_rev));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_l_fwd, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_l_rev, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_r_fwd, 0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_r_rev, 0));

  mcpwm_generator_config_t gcfg = {};
  gcfg.gen_gpio_num = L_MOT_FWD; ESP_ERROR_CHECK(mcpwm_new_generator(op_left,  &gcfg, &gen_l_fwd));
  gcfg.gen_gpio_num = L_MOT_REV; ESP_ERROR_CHECK(mcpwm_new_generator(op_left,  &gcfg, &gen_l_rev));
  gcfg.gen_gpio_num = R_MOT_FWD; ESP_ERROR_CHECK(mcpwm_new_generator(op_right, &gcfg, &gen_r_fwd));
  gcfg.gen_gpio_num = R_MOT_REV; ESP_ERROR_CHECK(mcpwm_new_generator(op_right, &gcfg, &gen_r_rev));

  // On timer empty: set HIGH (start of pulse)
  mcpwm_gen_timer_event_action_t act_empty_high = {};
  act_empty_high.direction = MCPWM_TIMER_DIRECTION_UP;
  act_empty_high.event     = MCPWM_TIMER_EVENT_EMPTY;
  act_empty_high.action    = MCPWM_GEN_ACTION_HIGH;

  // On compare: set LOW (end of pulse)
  mcpwm_gen_compare_event_action_t act_cmp_low = {};
  act_cmp_low.direction = MCPWM_TIMER_DIRECTION_UP;
  act_cmp_low.action    = MCPWM_GEN_ACTION_LOW;

  // Left FWD
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_l_fwd, act_empty_high));
  act_cmp_low.comparator = cmp_l_fwd;
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_l_fwd, act_cmp_low));
  // Left REV
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_l_rev, act_empty_high));
  act_cmp_low.comparator = cmp_l_rev;
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_l_rev, act_cmp_low));
  // Right FWD
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_r_fwd, act_empty_high));
  act_cmp_low.comparator = cmp_r_fwd;
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_r_fwd, act_cmp_low));
  // Right REV
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_r_rev, act_empty_high));
  act_cmp_low.comparator = cmp_r_rev;
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_r_rev, act_cmp_low));

  ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));
}

static inline void set_compare(mcpwm_cmpr_handle_t cmp, uint32_t ticks) {
  ticks = clamp_u32(ticks, 0, period_ticks);
  mcpwm_comparator_set_compare_value(cmp, ticks);
}
static void setWheel(float u, mcpwm_cmpr_handle_t cmp_fwd, mcpwm_cmpr_handle_t cmp_rev) {
  u = clamp_f(u, -1.0f, 1.0f);
  uint32_t duty = (uint32_t)(fabsf(u) * (float)period_ticks);
  if (u >= 0) { set_compare(cmp_fwd, duty); set_compare(cmp_rev, 0); }
  else        { set_compare(cmp_fwd, 0);    set_compare(cmp_rev, duty); }
}
static void applyCommand(float uL, float uR) {
  // Apply gain correction to make car go straight
  float uL_corrected = uL * GAIN_LEFT;
  float uR_corrected = uR * GAIN_RIGHT;
  
  g_uL = clamp_f(uL_corrected, -1.0f, 1.0f);
  g_uR = clamp_f(uR_corrected, -1.0f, 1.0f);
  setWheel(g_uL, cmp_l_fwd, cmp_l_rev);
  setWheel(g_uR, cmp_r_fwd, cmp_r_rev);
}
static void stopMotors() {
  set_compare(cmp_l_fwd, 0);
  set_compare(cmp_l_rev, 0);
  set_compare(cmp_r_fwd, 0);
  set_compare(cmp_r_rev, 0);
  g_uL = 0; g_uR = 0;
}

// ---- ESP-NOW ----
// Add global for transmit timing
static volatile uint32_t g_tx_start_us = 0;
static volatile uint32_t g_tx_time_us = 0;

static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (g_tx_start_us > 0) {
    g_tx_time_us = micros() - g_tx_start_us;
    g_tx_start_us = 0;  // Reset
  }
}

static void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len == (int)sizeof(CmdMsg)) {
    CmdMsg in{};
    memcpy(&in, data, sizeof(in));   // safe local copy

    // element-wise apply
    float uL  = in.uL;
    float uR  = in.uR;
    uint32_t dur = in.duration_ms;
    uint8_t reset_yaw = in.reset_yaw;
    uint8_t recal_gyro = in.recal_gyro;

    // Handle yaw reset first
    if (reset_yaw) {
      g_yaw_deg = 0.0f;
      Serial.println("[YAW RESET] Yaw angle reset to 0 degrees");
    }

    // Handle gyro recalibration
    if (recal_gyro) {
      g_imu_calib_end_ms = millis() + GYRO_CAL_MS;
      g_gz_bias = 0.0f;
      g_imu_ready = false;
      g_recalibrating = true;
      Serial.printf("[GYRO RECAL] Starting gyro recalibration for %d ms\n", GYRO_CAL_MS);
    }

    applyCommand(uL, uR);
    if (dur > 0) g_cmd_expire_ms = millis() + dur;
    else         g_cmd_expire_ms = 0;

    Serial.printf("[RX CMD] seq=%lu  uL=%.2f  uR=%.2f  dur=%lums  reset_yaw=%u  recal_gyro=%u\n",
                  (unsigned long)in.seq, uL, uR, (unsigned long)dur, reset_yaw, recal_gyro);
  }
}
static void sendTelemetry(float yaw_deg, int16_t* accel, int16_t* gyro) {
  // Add sample to batch
  g_telemetry_buffer[g_batch_index].millis_rx = millis();
  g_telemetry_buffer[g_batch_index].yaw_deg = yaw_deg;
  g_telemetry_buffer[g_batch_index].uL_applied = g_uL;
  g_telemetry_buffer[g_batch_index].uR_applied = g_uR;
  // Add full IMU data
  memcpy(g_telemetry_buffer[g_batch_index].accel, accel, sizeof(int16_t) * 3);
  memcpy(g_telemetry_buffer[g_batch_index].gyro, gyro, sizeof(int16_t) * 3);
  g_batch_index++;

  // Send batch when full
  if (g_batch_index >= BATCH_SIZE) {
    TelemetryMsg t{};
    t.seq_rx = g_batch_seq++;
    t.sample_count = BATCH_SIZE;
    memcpy(t.samples, g_telemetry_buffer, sizeof(TelemetrySample) * BATCH_SIZE);
    
    // Record transmit start time
    g_tx_start_us = micros();
    esp_now_send(g_peer_mac, (uint8_t*)&t, sizeof(t));
    
    // Print transmit time when available (will be updated by callback)
    static uint32_t last_print = 0;
    if (g_tx_time_us > 0 && millis() - last_print > 1000) {  // Print every ~1s
      Serial.printf("[TX TIME] ESP-NOW transmit: %lu us (batched %d samples)\n", (unsigned long)g_tx_time_us, BATCH_SIZE);
      g_tx_time_us = 0;  // Reset after printing
      last_print = millis();
    }
    
    g_batch_index = 0;  // Reset batch
  }
}

// ---- Control/IMU periodic ----
static void ctrl_tick_cb(void* arg) { g_ctrl_tick = true; }

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println("=== RECEIVER ===");
  
  // Print motor gain correction values
  Serial.printf("Motor gain correction: r=%.5f, GAIN_LEFT=%.5f, GAIN_RIGHT=%.5f\n", 
                GAIN_RATIO_R, GAIN_LEFT, GAIN_RIGHT);

  mcpwm_setup();
  stopMotors();

  if (!mpu_init()) {
    Serial.println("MPU6050 init FAILED");
  }

  // 1) Bias first (blocking)
  calibrate_gyro_bias_blocking(GYRO_CAL_MS);

  // 2) Then transmission (WiFi/ESP-NOW)
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

  // Print MAC address
  uint8_t mac[6]; esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

  // Set WiFi channel
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  // Optimize WiFi for faster ESP-NOW transmission
  esp_wifi_set_max_tx_power(78);  // Max TX power (78 = 19.5 dBm)
  esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS7_SGI);  // High data rate

  if (esp_now_init()!=ESP_OK) {
    Serial.println("ESP-NOW init FAILED, rebooting...");
    delay(1000); ESP.restart();
  }
  
  // Optimize ESP-NOW for faster transmission
  esp_now_set_pmk((uint8_t*)"pmk1234567890123");  // Set PMK for faster peer addition
  
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add transmitter as peer
  esp_now_peer_info_t peer_info = {};
  memcpy(peer_info.peer_addr, g_peer_mac, 6);
  peer_info.channel = ESPNOW_CHANNEL;
  peer_info.encrypt = false;
  peer_info.ifidx = WIFI_IF_STA;  // Specify interface
  
  if (esp_now_add_peer(&peer_info) != ESP_OK) {
    Serial.println("Failed to add peer");
  }

  // 3) Finally, rest of the code: control timer and runtime
  const esp_timer_create_args_t args = {};
  esp_timer_create_args_t a = args;
  a.callback = &ctrl_tick_cb;
  a.arg = nullptr;
  a.dispatch_method = ESP_TIMER_TASK;
  a.name = "ctrl200Hz";
  ESP_ERROR_CHECK(esp_timer_create(&a, &g_ctrl_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(g_ctrl_timer, 1000000ULL/CTRL_HZ));

  g_last_tel_ms = millis();
  Serial.println("Ready");
}

void loop() {
  if (g_ctrl_tick) {
    g_ctrl_tick = false;

    static uint32_t last_us = micros();
    uint32_t now_us = micros();
    float dt = (now_us - last_us) * 1e-6f;
    if (dt <= 0 || dt > 0.5f) dt = 1.0f/CTRL_HZ;
    last_us = now_us;

    // IMU: read accel and gyro, measure time
    static uint32_t imu_print_count = 0;
    uint32_t imu_read_start = micros();
    // Read accel and gyro in burst
    int16_t accel[3], gyro[3];
    mpu_read_accel_gyro_burst(accel, gyro);
    int16_t ax_raw = accel[0];
    int16_t ay_raw = accel[1];
    int16_t az_raw = accel[2];
    int16_t gx_raw = gyro[0];
    int16_t gy_raw = gyro[1];
    int16_t gz_raw = gyro[2];
    uint32_t imu_read_end = micros();
    if (++imu_print_count % 500 == 0) {
      Serial.printf("IMU read time: %lu us\n", imu_read_end - imu_read_start);
    }
    float gz_dps = -(float)gz_raw / GYRO_SENS_DPS;  // Negate for correct turn direction

    // Handle gyro recalibration if in progress
    if (g_recalibrating) {
      if (millis() < g_imu_calib_end_ms) {
        static double acc=0.0; static uint32_t n=0;
        // Reset accumulator if this is the start of recalibration
        static uint32_t last_recal_start = 0;
        if (g_imu_calib_end_ms - GYRO_CAL_MS != last_recal_start) {
          acc = 0.0; n = 0;
          last_recal_start = g_imu_calib_end_ms - GYRO_CAL_MS;
        }
        acc += gz_dps; n++;
        g_gz_bias = (float)(acc/(double)n);
      } else {
        g_imu_ready = true;
        g_recalibrating = false;
        Serial.printf("[GYRO RECAL] Recalibration complete. New bias: %.3f dps\n", g_gz_bias);
      }
    }

    float gz_corr = gz_dps - g_gz_bias;
    g_yaw_deg += gz_corr * dt * GYRO_GAIN_CORR;  // Apply gain correction
    if (g_yaw_deg >= 180.0f) g_yaw_deg -= 360.0f;
    else if (g_yaw_deg < -180.0f) g_yaw_deg += 360.0f;    // Timed command expiry
    uint32_t ms = millis();
    if (g_cmd_expire_ms && ms >= g_cmd_expire_ms) {
      stopMotors();
      g_cmd_expire_ms = 0;
    }

    // Telemetry
    if (ms - g_last_tel_ms >= (1000 / TEL_HZ)) {
      g_last_tel_ms = ms;
      sendTelemetry(g_yaw_deg, accel, gyro);
    }
  }

  // Optional local stop
  while (Serial.available()) {
    String s = Serial.readStringUntil('\n'); s.trim();
    if (s.equalsIgnoreCase("S")) { stopMotors(); Serial.println("Stopped."); }
  }
}