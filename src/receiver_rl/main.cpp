// RL Controller for Seeed XIAO ESP32S3
// - MCPWM @ 20 kHz driving DRV8833 
// - VL6180X distance sensors (3x: Left, Center, Right)
// - Neural network inference at 50Hz
// - Dynamic past states support (sensors or wheel outputs)
// - No ESP-NOW, no IMU for simplicity

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include "driver/mcpwm_prelude.h"
#include "esp_timer.h"
#include "neural_network.h"

// --------------- CONFIG ----------------
#define SERIAL_BAUD     115200

#define INFERENCE_HZ    50

// DRV8833 pins (XIAO ESP32S3) - same as testing receiver
#define L_MOT_FWD       3
#define L_MOT_REV       4
#define R_MOT_FWD       2
#define R_MOT_REV       1

// VL6180X I2C addresses (same as PID code)
#define VL6180X_LEFT_ADDR    0x2A
#define VL6180X_CENTER_ADDR  0x2B  
#define VL6180X_RIGHT_ADDR   0x2C

// VL6180X GPIO pins for XSHUT (shutdown) - using same as PID code
#define VL_LEFT_XSHUT   D10  // Left
#define VL_CENTER_XSHUT D9   // Mid/Center  
#define VL_RIGHT_XSHUT  D8   // Right

// PWM configuration - same as testing receiver
#define PWM_FREQ_HZ     20000
#define PWM_RES_HZ      1000000

// Sensor configuration
#define MAX_SENSOR_DISTANCE_MM  180.0f  // Normalize to this max distance
#define MIN_SENSOR_DISTANCE_MM  10.0f   // Minimum reliable reading

// Motor scaling: model output [0,1] -> PWM [0, 0.4]
#define MAX_MOTOR_POWER 0.4f
// --------------------------------------

// ---- Globals ----
static NeuralNetwork g_neural_net;

static volatile float g_sensor_distances[3] = {0.5f, 0.5f, 0.5f};  // Normalized [0,1]
static volatile bool g_sensor_updated[3] = {false, false, false};
static volatile uint32_t g_last_sensor_read[3] = {0, 0, 0};

static esp_timer_handle_t g_inference_timer = nullptr;
static volatile bool g_inference_tick = false;

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

// ---- VL6180X using Adafruit library (same as PID code) ----
static Adafruit_VL6180X vl_left, vl_center, vl_right;

// I2C speed (same as PID code)
const uint32_t I2C_HZ = 400000;  // 400 kHz

// I2C register access helpers (from PID code)
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

// VL6180X fast continuous config (~50 Hz) - from PID code
void vl6180x_fastInit(uint8_t addr) {
    wr8(addr, 0x0014, 0x04);  // Interrupt config
    wr8(addr, 0x001C, 12);    // Range measurement period
    wr8(addr, 0x010A, 12);    // Averaging sample period
    wr8(addr, 0x001B, 1);     // Range offset
    wr8(addr, 0x0018, 0x03);  // Start continuous ranging
}

// Fast register-based sensor status and read (from PID code)
inline bool vl6180x_ready(uint8_t addr) { 
    return (rd8(addr, 0x004F) & 0x07) == 0x04; 
}

inline uint8_t vl6180x_read_mm(uint8_t addr) {
    uint8_t mm = rd8(addr, 0x0062);
    wr8(addr, 0x0015, 0x07);  // Clear interrupt
    return mm;
}

inline uint8_t clamp180(uint8_t mm) { 
    return (mm > 180) ? 180 : mm; 
}

// Helper function for sensor initialization (from PID code)
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
    // Scale from [0,1] to [-MAX_MOTOR_POWER, MAX_MOTOR_POWER]
    // For now, assume forward-only (no reverse)
    float left_pwm = left_speed * MAX_MOTOR_POWER;
    float right_pwm = right_speed * MAX_MOTOR_POWER;
    
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
    
    // Configure XSHUT pins and reset all sensors (same as PID code)
    pinMode(VL_LEFT_XSHUT, OUTPUT);
    pinMode(VL_CENTER_XSHUT, OUTPUT);
    pinMode(VL_RIGHT_XSHUT, OUTPUT);
    digitalWrite(VL_LEFT_XSHUT, LOW);
    digitalWrite(VL_CENTER_XSHUT, LOW);
    digitalWrite(VL_RIGHT_XSHUT, LOW);
    delay(10);
    
    // Initialize sensors one by one with retry (same as PID code)
    while (!initOneTOF_SHDN(VL_LEFT_XSHUT, vl_left, VL6180X_LEFT_ADDR)) {
        digitalWrite(VL_LEFT_XSHUT, LOW);
        delay(20);
        Serial.println("Retrying left VL6180X...");
    }
    
    while (!initOneTOF_SHDN(VL_CENTER_XSHUT, vl_center, VL6180X_CENTER_ADDR)) {
        digitalWrite(VL_CENTER_XSHUT, LOW);
        delay(20);
        Serial.println("Retrying center VL6180X...");
    }
    
    while (!initOneTOF_SHDN(VL_RIGHT_XSHUT, vl_right, VL6180X_RIGHT_ADDR)) {
        digitalWrite(VL_RIGHT_XSHUT, LOW);
        delay(20);
        Serial.println("Retrying right VL6180X...");
    }
    
    // Configure sensors for fast continuous reads (from PID code)
    vl6180x_fastInit(VL6180X_LEFT_ADDR);
    vl6180x_fastInit(VL6180X_CENTER_ADDR);
    vl6180x_fastInit(VL6180X_RIGHT_ADDR);
    
    Serial.println("All VL6180X sensors initialized and configured for continuous reads");
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



// ---- Timer Callbacks ----
static void inference_tick_cb(void* arg) { 
    g_inference_tick = true; 
}

// ---- Main Functions ----
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);
    Serial.println("=== RL CONTROLLER ===");
    
    // Print model configuration
    Serial.printf("Model: %d inputs, %d outputs, %d layers\n", 
                  ACTOR_INPUT_DIM, ACTOR_OUTPUT_DIM, ACTOR_NUM_LAYERS);
    Serial.printf("Past states: %s, count=%d, stride=%d, source=%s\n",
                  PAST_STATES_ENABLED ? "enabled" : "disabled",
                  PAST_STATES_COUNT, PAST_STATES_STRIDE, PAST_STATES_SOURCE);
    
    // Setup hardware
    mcpwm_setup();
    stopMotors();
    
    if(!initSensors()) {
        Serial.println("Sensor initialization failed!");
        while(1) delay(1000);
    }
    
    // Setup inference timer (sensor reading now in main loop like PID code)
    esp_timer_create_args_t inference_args = {};
    inference_args.callback = &inference_tick_cb;
    inference_args.name = "inference_50hz";
    ESP_ERROR_CHECK(esp_timer_create(&inference_args, &g_inference_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_inference_timer, 1000000ULL / INFERENCE_HZ));
    
    Serial.println("RL Controller ready - starting autonomous operation");
    
    // Wait a bit for sensors to stabilize
    delay(1000);
}

void loop() {
    // Check all sensors on each loop cycle (like PID code - more efficient for continuous mode)
    if (vl6180x_ready(VL6180X_LEFT_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = clamp180(vl6180x_read_mm(VL6180X_LEFT_ADDR));
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[0] = normalizeSensorReading(range);
        g_sensor_updated[0] = true;
        g_last_sensor_read[0] = millis();
        Serial.printf("[TIMING] Left sensor read: %lu us\n", read_time);
    }
    if (vl6180x_ready(VL6180X_CENTER_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = clamp180(vl6180x_read_mm(VL6180X_CENTER_ADDR));
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[1] = normalizeSensorReading(range);
        g_sensor_updated[1] = true;
        g_last_sensor_read[1] = millis();
        Serial.printf("[TIMING] Center sensor read: %lu us\n", read_time);
    }
    if (vl6180x_ready(VL6180X_RIGHT_ADDR)) {
        uint32_t read_start = micros();
        uint8_t range = clamp180(vl6180x_read_mm(VL6180X_RIGHT_ADDR));
        uint32_t read_time = micros() - read_start;
        g_sensor_distances[2] = normalizeSensorReading(range);
        g_sensor_updated[2] = true;
        g_last_sensor_read[2] = millis();
        Serial.printf("[TIMING] Right sensor read: %lu us\n", read_time);
    }
    
    // 50Hz neural network inference
    if(g_inference_tick) {
        g_inference_tick = false;
        
        uint32_t inference_start = micros();
        
        // Update neural network with current sensor readings (reordered: center, right, left)
        float sensors[3] = {g_sensor_distances[1], g_sensor_distances[2], g_sensor_distances[0]};
        g_neural_net.updateSensorState(sensors, 3);
        
        // Run inference
        float wheel_outputs[2];
        g_neural_net.predict(sensors, wheel_outputs);
        
        // Update neural network with wheel outputs (for models that use past wheel states)
        g_neural_net.updateWheelState(wheel_outputs, 2);
        
        uint32_t inference_time = micros() - inference_start;
        
        // Apply motor commands
        applyWheelCommands(wheel_outputs[0], wheel_outputs[1]);
        
        // Timing output
        Serial.printf("[TIMING] Neural network inference: %lu us\n", inference_time);
    }
    
    // Emergency stop via serial
    if(Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if(cmd.equalsIgnoreCase("S") || cmd.equalsIgnoreCase("STOP")) {
            stopMotors();
            Serial.println("EMERGENCY STOP");
        }
    }
}
