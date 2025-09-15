#pragma once

#include <Arduino.h>
#include <cmath>
#include "model_structure.h"

class NeuralNetwork {
public:
    NeuralNetwork();
    
    // Main inference function
    void predict(const float* input, float* output);
    
    // Ring buffer management for past states
    void updateSensorState(const float* sensors, int sensor_count);
    void updateWheelState(const float* wheels, int wheel_count);
    
    // Get current input vector (current + past states)
    void buildInputVector(float* input_vector);
    
private:
    // Activation functions
    static float relu(float x) { return x > 0 ? x : 0; }
    static float tanh_activation(float x) { return tanh(x); }
    
    // Layer forward pass - template function for different weight matrix sizes
    template<int MAX_IN>
    void forwardLayer(const float* input, float* output, 
                     const float weights[][MAX_IN], const float* biases,
                     int input_size, int output_size, const char* activation);
    
    // Ring buffers for past states
    static const int MAX_SENSORS = 3;  // Left, Center, Right distance sensors
    static const int MAX_WHEELS = 2;   // Left, Right wheel outputs
    static const int MAX_BUFFER_SIZE = 100;  // Enough for any reasonable stride/count
    
    // Sensor ring buffer (normalized 0-1 values)
    float sensor_buffer[MAX_BUFFER_SIZE][MAX_SENSORS];
    int sensor_buffer_index;
    bool sensor_buffer_full;
    
    // Wheel output ring buffer (0-1 values)
    float wheel_buffer[MAX_BUFFER_SIZE][MAX_WHEELS];
    int wheel_buffer_index;
    bool wheel_buffer_full;
    
    // Current sensor readings
    float current_sensors[MAX_SENSORS];
    bool sensors_updated;
    
    // Layer temporary storage
    float layer0_output[L0_OUT];
    float layer1_output[L1_OUT];
};

// Implementation
NeuralNetwork::NeuralNetwork() 
    : sensor_buffer_index(0), sensor_buffer_full(false),
      wheel_buffer_index(0), wheel_buffer_full(false),
      sensors_updated(false) {
    
    // Initialize buffers to zero
    memset(sensor_buffer, 0, sizeof(sensor_buffer));
    memset(wheel_buffer, 0, sizeof(wheel_buffer));
    memset(current_sensors, 0, sizeof(current_sensors));
}

void NeuralNetwork::updateSensorState(const float* sensors, int sensor_count) {
    // Update current sensor readings
    for(int i = 0; i < sensor_count && i < MAX_SENSORS; i++) {
        current_sensors[i] = sensors[i];
    }
    sensors_updated = true;
    
    // Add to ring buffer
    for(int i = 0; i < sensor_count && i < MAX_SENSORS; i++) {
        sensor_buffer[sensor_buffer_index][i] = sensors[i];
    }
    
    sensor_buffer_index = (sensor_buffer_index + 1) % MAX_BUFFER_SIZE;
    if(sensor_buffer_index == 0) sensor_buffer_full = true;
}

void NeuralNetwork::updateWheelState(const float* wheels, int wheel_count) {
    // Add to ring buffer
    for(int i = 0; i < wheel_count && i < MAX_WHEELS; i++) {
        wheel_buffer[wheel_buffer_index][i] = wheels[i];
    }
    
    wheel_buffer_index = (wheel_buffer_index + 1) % MAX_BUFFER_SIZE;
    if(wheel_buffer_index == 0) wheel_buffer_full = true;
}

void NeuralNetwork::buildInputVector(float* input_vector) {
    int input_idx = 0;
    
    // Add current sensor readings (always first 3 inputs)
    for(int i = 0; i < MAX_SENSORS; i++) {
        input_vector[input_idx++] = current_sensors[i];
    }
    
    // Add past states if enabled
    if(PAST_STATES_ENABLED && PAST_STATES_COUNT > 0) {
        bool use_sensors = (strcmp(PAST_STATES_SOURCE, "sensors") == 0);
        
        for(int state = 0; state < PAST_STATES_COUNT; state++) {
            // Calculate the buffer index for this past state
            int steps_back = (state + 1) * PAST_STATES_STRIDE;
            int buffer_idx;
            
            if(use_sensors) {
                buffer_idx = (sensor_buffer_index - steps_back + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
                
                // Add past sensor readings
                for(int i = 0; i < MAX_SENSORS; i++) {
                    if(sensor_buffer_full || buffer_idx < sensor_buffer_index) {
                        input_vector[input_idx++] = sensor_buffer[buffer_idx][i];
                    } else {
                        input_vector[input_idx++] = 0.0f;  // No data available yet
                    }
                }
            } else {
                // Use past wheel outputs
                buffer_idx = (wheel_buffer_index - steps_back + MAX_BUFFER_SIZE) % MAX_BUFFER_SIZE;
                
                for(int i = 0; i < MAX_WHEELS; i++) {
                    if(wheel_buffer_full || buffer_idx < wheel_buffer_index) {
                        input_vector[input_idx++] = wheel_buffer[buffer_idx][i];
                    } else {
                        input_vector[input_idx++] = 0.0f;  // No data available yet
                    }
                }
            }
        }
    }
}

template<int MAX_IN>
void NeuralNetwork::forwardLayer(const float* input, float* output,
                                const float weights[][MAX_IN], const float* biases,
                                int input_size, int output_size, const char* activation) {
    
    for(int out = 0; out < output_size; out++) {
        float sum = biases[out];
        
        for(int in = 0; in < input_size; in++) {
            sum += input[in] * weights[out][in];
        }
        
        // Apply activation function
        if(strcmp(activation, "relu") == 0) {
            output[out] = relu(sum);
        } else if(strcmp(activation, "tanh") == 0) {
            output[out] = tanh_activation(sum);
        } else {
            output[out] = sum;  // Linear/no activation
        }
    }
}

void NeuralNetwork::predict(const float* input, float* output) {
    // Build the full input vector with past states
    float full_input[ACTOR_INPUT_DIM];
    buildInputVector(full_input);
    
    // Layer 0: Input -> Hidden1
    forwardLayer<L0_IN>(full_input, layer0_output, L0_WEIGHTS, L0_BIASES, L0_IN, L0_OUT, L0_ACTIVATION);
    
    // Layer 1: Hidden1 -> Hidden2  
    forwardLayer<L1_IN>(layer0_output, layer1_output, L1_WEIGHTS, L1_BIASES, L1_IN, L1_OUT, L1_ACTIVATION);
    
    // Layer 2: Hidden2 -> Output
    forwardLayer<L2_IN>(layer1_output, output, L2_WEIGHTS, L2_BIASES, L2_IN, L2_OUT, L2_ACTIVATION);
    
    // Scale output from tanh [-1,1] to [0,1] for wheel speeds
    for(int i = 0; i < ACTOR_OUTPUT_DIM; i++) {
        output[i] = (output[i] + 1.0f) * 0.5f;  // Map [-1,1] to [0,1]
        output[i] = constrain(output[i], 0.0f, 1.0f);
    }
}