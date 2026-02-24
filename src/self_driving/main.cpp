// RL Controller for Seeed XIAO ESP32S3
// - MCPWM @ 20 kHz driving DRV8833 
// - VL6180X distance sensors (3x: Left, Center, Right)
// - MPU6050 IMU with yaw integration
// - Neural network inference at 50Hz
// - ESP-NOW telemetry transmission (batched)
// - Dynamic past states support (sensors or wheel outputs)

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_VL6180X.h>
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "neural_network.h"
#include "logging.h"

// --------------- CONFIG ----------------
#define SERIAL_BAUD     115200
#define INFERENCE_HZ    50

#define ESPNOW_CHANNEL  6  // ESP-NOW channel
#define TEL_BATCH_SIZE  5  // Send telemetry in batches of 5

// DRV8833 pins (XIAO ESP32S3) - same as testing receiver
#define L_MOT_FWD       3
#define L_MOT_REV       4
#define R_MOT_FWD       2
#define R_MOT_REV       1

// VL6180X I2C addresses
#define VL6180X_LEFT_ADDR    0x2A
#define VL6180X_CENTER_ADDR  0x2B  
#define VL6180X_RIGHT_ADDR   0x2C

// VL6180X GPIO pins for XSHUT (shutdown)
#define VL_LEFT_XSHUT   D10  // Left
#define VL_CENTER_XSHUT D9   // Mid/Center  
#define VL_RIGHT_XSHUT  D8   // Right

// MPU6050
#define MPU_ADDR        0x68
static const float GYRO_SENS_DPS = 16.4f; // FS_SEL=3 (±2000 dps)
static const float GYRO_GAIN_CORR = 1.0127f; // Gain correction
#define GYRO_CAL_MS     4000

// PWM configuration - same as testing receiver
#define PWM_FREQ_HZ     20000
#define PWM_RES_HZ      1000000

// Sensor configuration
#define MAX_SENSOR_DISTANCE_MM  180.0f  // Normalize to this max distance
#define MIN_SENSOR_DISTANCE_MM  10.0f   // Minimum reliable reading

// Motor scaling: model output [0,1] -> PWM [0, MAX_MOTOR_POWER]
#define MAX_MOTOR_POWER 0.5f
// Turn gain: multiply the differential (left-right) by this factor to change turning authority
// 1.0 = no change, <1.0 reduces turning, >1.0 increases turning
#define TURN_GAIN 0.6f
// --------------------------------------

// ---- Messages ----
#pragma pack(push,1)
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
  TelemetrySample samples[TEL_BATCH_SIZE];
};
#pragma pack(pop)
// --------------------------------------

// ---- Globals ----
static NeuralNetwork g_neural_net;

static volatile float g_sensor_distances[3] = {0.5f, 0.5f, 0.5f};  // Normalized [0,1]

static esp_timer_handle_t g_inference_timer = nullptr;
static volatile bool g_inference_tick = false;

// IMU globals
static volatile float g_yaw_deg = 0.0f;
static float g_gz_bias = 0.0f;
static bool g_imu_ready = false;
static uint32_t g_imu_calib_end_ms = 0;

// Previous NN outputs for telemetry pairing
static float g_prev_nn_left = 0.0f;
static float g_prev_nn_right = 0.0f;

// ESP-NOW globals
static uint8_t g_peer_mac[6] = {0x54, 0x32, 0x04, 0x21, 0x80, 0x28};
static volatile uint32_t g_tx_start_us = 0;
static volatile uint32_t g_tx_time_us = 0;
static uint32_t g_telemetry_seq = 1;

// Telemetry batching
static TelemetrySample g_telemetry_buffer[TEL_BATCH_SIZE];
static uint8_t g_batch_index = 0;

// MCPWM handles - same setup as testing receiver
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
static inline uint32_t clamp_u32(uint32_t x, uint32_t lo, uint32_t hi) { 
    return x < lo ? lo : (x > hi ? hi : x); 
}
static inline float clamp_f(float x, float lo, float hi) { 
    return x < lo ? lo : (x > hi ? hi : x); 
}

// ---- VL6180X ----
static Adafruit_VL6180X vl_left, vl_center, vl_right;

// I2C speed
const uint32_t I2C_HZ = 400000;  // 400 kHz

// I2C register access helpers
uint8_t rd8(uint8_t addr, uint16_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg >> 8); 
    Wire.write(reg & 0xFF);
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    return Wire.read();
}

void wr8(uint8_t addr, uint16_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg >> 8); 
    Wire.write(reg & 0xFF); 
    Wire.write(val);
    Wire.endTransmission();
}

// VL6180X fast continuous config (~50 Hz)
void vl6180x_fastInit(uint8_t addr) {
    wr8(addr, 0x0014, 0x04);  // Interrupt config
    wr8(addr, 0x001C, 12);    // Range measurement period
    wr8(addr, 0x010A, 12);    // Averaging sample period
    wr8(addr, 0x001B, 1);     // Range offset
    wr8(addr, 0x0018, 0x03);  // Start continuous ranging
}

// Fast register-based sensor status and read
inline bool vl6180x_ready(uint8_t addr) { 
    return (rd8(addr, 0x004F) & 0x07) == 0x04; 
}

inline uint8_t vl6180x_read_mm(uint8_t addr) {
    uint8_t mm = rd8(addr, 0x0062);
    wr8(addr, 0x0015, 0x07);  // Clear interrupt
    return mm;
}



// Helper function for sensor initialization
bool initOneTOF_SHDN(int shdnPin, Adafruit_VL6180X &dev, uint8_t newAddr) {
    pinMode(shdnPin, OUTPUT);
    digitalWrite(shdnPin, HIGH);
    delay(5);
    if (!dev.begin(&Wire)) return false;
    if (!dev.setAddress(newAddr)) return false;
    delay(2);
    return true;
}

// ---- MCPWM setup (same as testing receiver) ----
static void mcpwm_setup() {
    mcpwm_timer_config_t tcfg = {};
    tcfg.group_id = 0;
    tcfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = PWM_RES_HZ;
    tcfg.period_ticks = PWM_RES_HZ / PWM_FREQ_HZ;
    tcfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &pwm_timer));
    period_ticks = tcfg.period_ticks;

    mcpwm_operator_config_t ocfg = {};
    ocfg.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &op_left));
    ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &op_right));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op_left, pwm_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(op_right, pwm_timer));

    mcpwm_comparator_config_t ccfg = {};
    ccfg.flags.update_cmp_on_tez = true;
    ESP_ERROR_CHECK(mcpwm_new_comparator(op_left, &ccfg, &cmp_l_fwd));
    ESP_ERROR_CHECK(mcpwm_new_comparator(op_left, &ccfg, &cmp_l_rev));
    ESP_ERROR_CHECK(mcpwm_new_comparator(op_right, &ccfg, &cmp_r_fwd));
    ESP_ERROR_CHECK(mcpwm_new_comparator(op_right, &ccfg, &cmp_r_rev));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_l_fwd, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_l_rev, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_r_fwd, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_r_rev, 0));

    mcpwm_generator_config_t gcfg = {};
    gcfg.gen_gpio_num = L_MOT_FWD; ESP_ERROR_CHECK(mcpwm_new_generator(op_left, &gcfg, &gen_l_fwd));
    gcfg.gen_gpio_num = L_MOT_REV; ESP_ERROR_CHECK(mcpwm_new_generator(op_left, &gcfg, &gen_l_rev));
    gcfg.gen_gpio_num = R_MOT_FWD; ESP_ERROR_CHECK(mcpwm_new_generator(op_right, &gcfg, &gen_r_fwd));
    gcfg.gen_gpio_num = R_MOT_REV; ESP_ERROR_CHECK(mcpwm_new_generator(op_right, &gcfg, &gen_r_rev));

    // Configure PWM actions
    mcpwm_gen_timer_event_action_t act_empty_high = {};
    act_empty_high.direction = MCPWM_TIMER_DIRECTION_UP;
    act_empty_high.event = MCPWM_TIMER_EVENT_EMPTY;
    act_empty_high.action = MCPWM_GEN_ACTION_HIGH;

    mcpwm_gen_compare_event_action_t act_cmp_low = {};
    act_cmp_low.direction = MCPWM_TIMER_DIRECTION_UP;
    act_cmp_low.action = MCPWM_GEN_ACTION_LOW;

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_l_fwd, act_empty_high));
    act_cmp_low.comparator = cmp_l_fwd;
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_l_fwd, act_cmp_low));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_l_rev, act_empty_high));
    act_cmp_low.comparator = cmp_l_rev;
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_l_rev, act_cmp_low));
    
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_r_fwd, act_empty_high));
    act_cmp_low.comparator = cmp_r_fwd;
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_r_fwd, act_cmp_low));
    
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
    if (u >= 0) { 
        set_compare(cmp_fwd, duty); 
        set_compare(cmp_rev, 0); 
    } else { 
        set_compare(cmp_fwd, 0); 
        set_compare(cmp_rev, duty); 
    }
}

static void applyWheelCommands(float left_speed, float right_speed) {
    // Inputs may be in either range depending on ACTOR_ALLOW_REVERSE:
    // - If reverse is disallowed: outputs are in [0,1] (forward-only)
    // - If reverse allowed: outputs are in [-1,1]
    // We want to scale only the turning component (difference between wheels) while preserving
    // the average forward speed (so top speed is unaffected).

    // Compute average (forward) and differential (turn)
    float avg = (left_speed + right_speed) * 0.5f;
    float diff = (left_speed - right_speed) * 0.5f; // positive -> left faster -> turn right

    // Apply turn gain to differential only
    diff *= TURN_GAIN;

    // Reconstruct wheel commands
    float left_adj = avg + diff;
    float right_adj = avg - diff;

    // Clamp to the valid range depending on ACTOR_ALLOW_REVERSE
    if(!ACTOR_ALLOW_REVERSE) {
        left_adj = clamp_f(left_adj, 0.0f, 1.0f);
        right_adj = clamp_f(right_adj, 0.0f, 1.0f);
    } else {
        left_adj = clamp_f(left_adj, -1.0f, 1.0f);
        right_adj = clamp_f(right_adj, -1.0f, 1.0f);
    }

    // Map to PWM using MAX_MOTOR_POWER (preserves top speed)
    float left_pwm = left_adj * MAX_MOTOR_POWER;
    float right_pwm = right_adj * MAX_MOTOR_POWER;

    setWheel(left_pwm, cmp_l_fwd, cmp_l_rev);
    setWheel(right_pwm, cmp_r_fwd, cmp_r_rev);
}

static void stopMotors() {
    set_compare(cmp_l_fwd, 0);
    set_compare(cmp_l_rev, 0);
    set_compare(cmp_r_fwd, 0);
    set_compare(cmp_r_rev, 0);
}

// ---- Sensor Management ----
static bool initSensors() {
    Wire.begin();
    Wire.setClock(I2C_HZ);
    
    // Configure XSHUT pins and reset all sensors
    pinMode(VL_LEFT_XSHUT, OUTPUT);
    pinMode(VL_CENTER_XSHUT, OUTPUT);
    pinMode(VL_RIGHT_XSHUT, OUTPUT);
    digitalWrite(VL_LEFT_XSHUT, LOW);
    digitalWrite(VL_CENTER_XSHUT, LOW);
    digitalWrite(VL_RIGHT_XSHUT, LOW);
    delay(10);
    
    // Initialize sensors one by one with retry
    while (!initOneTOF_SHDN(VL_LEFT_XSHUT, vl_left, VL6180X_LEFT_ADDR)) {
        digitalWrite(VL_LEFT_XSHUT, LOW);
        delay(20);
        LOG_DEBUG("Retrying left VL6180X...\n");
    }
    
    while (!initOneTOF_SHDN(VL_CENTER_XSHUT, vl_center, VL6180X_CENTER_ADDR)) {
        digitalWrite(VL_CENTER_XSHUT, LOW);
        delay(20);
        LOG_DEBUG("Retrying center VL6180X...\n");
    }
    
    while (!initOneTOF_SHDN(VL_RIGHT_XSHUT, vl_right, VL6180X_RIGHT_ADDR)) {
        digitalWrite(VL_RIGHT_XSHUT, LOW);
        delay(20);
        LOG_DEBUG("Retrying right VL6180X...\n");
    }
    
    // Configure sensors for fast continuous reads
    vl6180x_fastInit(VL6180X_LEFT_ADDR);
    vl6180x_fastInit(VL6180X_CENTER_ADDR);
    vl6180x_fastInit(VL6180X_RIGHT_ADDR);
    
    LOG_INFO("All VL6180X sensors initialized and configured for continuous reads\n");
    return true;
}

static float normalizeSensorReading(uint8_t range_mm) {
    // If sensor error (255) or maxed out, return 1.0 (far/clear) as model was trained
    if(range_mm >= 254) return 1.0f;  
    
    float distance = (float)range_mm;
    
    // Sensor max range is 180mm = 1.0, anything beyond that is also 1.0 (far/clear)
    if(distance >= MAX_SENSOR_DISTANCE_MM) return 1.0f;
    
    distance = clamp_f(distance, MIN_SENSOR_DISTANCE_MM, MAX_SENSOR_DISTANCE_MM);
    
    // Normalize to [0,1] where 0 = close obstacle, 1 = far/clear
    return (distance - MIN_SENSOR_DISTANCE_MM) / (MAX_SENSOR_DISTANCE_MM - MIN_SENSOR_DISTANCE_MM);
}

// ---- MPU6050 IMU ----
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

// ---- ESP-NOW ----
static void onDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  if (g_tx_start_us > 0) {
    g_tx_time_us = micros() - g_tx_start_us;
    g_tx_start_us = 0;  // Reset
  }
}

static void sendTelemetryBatch() {
  if (g_batch_index == 0) return;  // Nothing to send

  TelemetryMsg t{};
  t.seq_tx = g_telemetry_seq++;
  t.sample_count = g_batch_index;
  memcpy(t.samples, g_telemetry_buffer, sizeof(TelemetrySample) * g_batch_index);

  // Record transmit start time
  g_tx_start_us = micros();
  esp_now_send(g_peer_mac, (uint8_t*)&t, sizeof(t));

  // Print transmit time when available
  static uint32_t last_print = 0;
  if (g_tx_time_us > 0 && millis() - last_print > 1000) {
    Serial.printf("[TX TIME] ESP-NOW transmit: %lu us (batched %d samples)\n", (unsigned long)g_tx_time_us, g_batch_index);
    g_tx_time_us = 0;
    last_print = millis();
  }

  g_batch_index = 0;  // Reset batch
}



// ---- Timer Callbacks ----
static void inference_tick_cb(void* arg) { 
    g_inference_tick = true; 
}

// ---- Main Functions ----
void setup() {
    LOG_INIT(SERIAL_BAUD);
    delay(100);
    LOG_PRINTLN("=== RL CONTROLLER ===");
    
    // Print model configuration
    LOG_INFO("Model: %d inputs, %d outputs, %d layers\n", 
                  ACTOR_INPUT_DIM, ACTOR_OUTPUT_DIM, ACTOR_NUM_LAYERS);
    LOG_INFO("Past states: %s, count=%d, stride=%d, source=%s\n",
                  PAST_STATES_ENABLED ? "enabled" : "disabled",
                  PAST_STATES_COUNT, PAST_STATES_STRIDE, 
                  PAST_STATES_ENABLED ? (PAST_STATES_USE_SENSORS ? "sensors" : "wheels") : "none");
    
    // Setup hardware
    mcpwm_setup();
    stopMotors();
    
    if(!initSensors()) {
        LOG_ERROR("Sensor initialization failed!\n");
        while(1) delay(1000);
    }
    
    // Setup IMU
    if (!mpu_init()) {
        LOG_ERROR("MPU6050 init FAILED\n");
    } else {
        // Calibrate gyro bias
        calibrate_gyro_bias_blocking(GYRO_CAL_MS);
    }
    
    // Setup ESP-NOW
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
    LOG_INFO("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);

    // Set WiFi channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    delay(100);

    // Optimize WiFi for faster ESP-NOW transmission
    esp_wifi_set_max_tx_power(78);  // Max TX power (78 = 19.5 dBm)
    esp_wifi_config_80211_tx_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS7_SGI);  // High data rate

    if (esp_now_init()!=ESP_OK) {
        LOG_ERROR("ESP-NOW init FAILED, rebooting...\n");
        delay(1000); ESP.restart();
    }
    
    // Optimize ESP-NOW for faster transmission
    esp_now_set_pmk((uint8_t*)"pmk1234567890123");  // Set PMK for faster peer addition
    
    esp_now_register_send_cb(onDataSent);

    // Add transmitter as peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, g_peer_mac, 6);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;  // Specify interface
    
    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        LOG_ERROR("Failed to add peer\n");
    }
    
    // Setup inference timer
    esp_timer_create_args_t inference_args = {};
    inference_args.callback = &inference_tick_cb;
    inference_args.name = "inference_50hz";
    ESP_ERROR_CHECK(esp_timer_create(&inference_args, &g_inference_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_inference_timer, 1000000ULL / INFERENCE_HZ));
    
    LOG_INFO("RL Controller ready - starting autonomous operation\n");
    
    // Wait a bit for sensors to stabilize
    delay(1000);
}

void loop() {
    // Check all sensors on each loop cycle (more efficient for continuous mode)
    if (vl6180x_ready(VL6180X_LEFT_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = vl6180x_read_mm(VL6180X_LEFT_ADDR);
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[0] = normalizeSensorReading(range);
        LOG_TIMING("[SENSOR] Left VL6180X read: %lu us\n", read_time);
    }
    if (vl6180x_ready(VL6180X_CENTER_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = vl6180x_read_mm(VL6180X_CENTER_ADDR);
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[1] = normalizeSensorReading(range);
        LOG_TIMING("[SENSOR] Center VL6180X read: %lu us\n", read_time);
    }
    if (vl6180x_ready(VL6180X_RIGHT_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = vl6180x_read_mm(VL6180X_RIGHT_ADDR);
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[2] = normalizeSensorReading(range);
        LOG_TIMING("[SENSOR] Right VL6180X read: %lu us\n", read_time);
    }
    
    // neural network inference
    if(g_inference_tick) {
        g_inference_tick = false;
        
        uint32_t inference_start = micros();
        
        // Read IMU data before inference
        int16_t accel[3], gyro[3];
        uint32_t imu_read_start = micros();
        mpu_read_accel_gyro_burst(accel, gyro);
        uint32_t imu_read_time = micros() - imu_read_start;
        LOG_TIMING("[IMU] MPU6050 read: %lu us\n", imu_read_time);
        
        // Update yaw integration if IMU is ready
        if (g_imu_ready) {
            float gz_dps = -(float)gyro[2] / GYRO_SENS_DPS;  // sign for turn direction
            float gz_corr = gz_dps - g_gz_bias;
            g_yaw_deg += gz_corr * (1.0f / INFERENCE_HZ) * GYRO_GAIN_CORR;  // Apply gain correction
            if (g_yaw_deg >= 180.0f) g_yaw_deg -= 360.0f;
            else if (g_yaw_deg < -180.0f) g_yaw_deg += 360.0f;
        }
        
        // Update neural network with current sensor readings (reordered: center, right, left)
        float sensors[3] = {g_sensor_distances[1], g_sensor_distances[2], g_sensor_distances[0]};
        g_neural_net.updateSensorState(sensors, 3);
        
        // Run inference
        float wheel_outputs[2];
        g_neural_net.predict(sensors, wheel_outputs);
        
        // Update neural network with wheel outputs (for models that use past wheel states)
        g_neural_net.updateWheelState(wheel_outputs, 2);
        
        // Apply motor commands
        applyWheelCommands(wheel_outputs[0], wheel_outputs[1]);
        
        uint32_t inference_time = micros() - inference_start;

        // Add telemetry sample - pair current IMU readings with PREVIOUS NN outputs
        g_telemetry_buffer[g_batch_index].millis_sent = millis();
        g_telemetry_buffer[g_batch_index].sensor_left = g_sensor_distances[0];
        g_telemetry_buffer[g_batch_index].sensor_center = g_sensor_distances[1];
        g_telemetry_buffer[g_batch_index].sensor_right = g_sensor_distances[2];
        g_telemetry_buffer[g_batch_index].nn_output_left = g_prev_nn_left;   // Use PREVIOUS NN outputs
        g_telemetry_buffer[g_batch_index].nn_output_right = g_prev_nn_right; // Use PREVIOUS NN outputs
        g_telemetry_buffer[g_batch_index].yaw_deg = g_yaw_deg;
        memcpy(g_telemetry_buffer[g_batch_index].accel, accel, sizeof(int16_t) * 3);
        memcpy(g_telemetry_buffer[g_batch_index].gyro, gyro, sizeof(int16_t) * 3);
        g_batch_index++;
        
        // Send batch when full
        if (g_batch_index >= TEL_BATCH_SIZE) {
            sendTelemetryBatch();
        }

        // Update previous NN outputs for next telemetry sample
        g_prev_nn_left = wheel_outputs[0];
        g_prev_nn_right = wheel_outputs[1];

        // Timing output
        LOG_TIMING("[TIMING] Neural network inference + action: %lu us\n", inference_time);
    }
}
