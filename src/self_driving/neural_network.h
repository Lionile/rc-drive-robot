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
    
    // Function pointer type for activation functions
    typedef float (*ActivationFunc)(float);
    
    // Layer forward pass - template function for different weight matrix sizes
    template<int MAX_IN>
    void forwardLayer(const float* input, float* output, 
                     const float weights[][MAX_IN], const float* biases,
                     int input_size, int output_size, ActivationFunc activation_func);
    
    // Ring buffers for past states
    static const int MAX_SENSORS = 3;  // Left, Center, Right distance sensors
    static const int MAX_WHEELS = 2;   // Left, Right wheel outputs
    
    // Calculate buffer configuration at compile time
    static const int BUFFER_SIZE = PAST_STATES_ENABLED ? (PAST_STATES_COUNT * PAST_STATES_STRIDE + 1) : 1;
    static const int BUFFER_STRIDE = PAST_STATES_ENABLED ? 
        (PAST_STATES_USE_SENSORS ? MAX_SENSORS : MAX_WHEELS) : 1;
    
    // Single ring buffer - used for either sensors or wheels based on configuration
    float past_states_buffer[BUFFER_SIZE][BUFFER_STRIDE];
    int buffer_index;
    bool buffer_full;
    
    // Current sensor readings
    float current_sensors[MAX_SENSORS];
    bool sensors_updated;
    
    // Function pointers for each layer (set once at initialization)
    ActivationFunc layer0_activation;
    ActivationFunc layer1_activation; 
    ActivationFunc layer2_activation;
    
    // Layer temporary storage
    float layer0_output[L0_OUT];
    float layer1_output[L1_OUT];
};

// Implementation
NeuralNetwork::NeuralNetwork() 
    : buffer_index(0), buffer_full(false), sensors_updated(false) {
    
    // Initialize buffer to zero
    memset(past_states_buffer, 0, sizeof(past_states_buffer));
    memset(current_sensors, 0, sizeof(current_sensors));
    
    // Set function pointers once at initialization - no more runtime string comparisons!
    layer0_activation = (strcmp(L0_ACTIVATION, "relu") == 0) ? relu : 
                       (strcmp(L0_ACTIVATION, "tanh") == 0) ? tanh_activation : nullptr;
    layer1_activation = (strcmp(L1_ACTIVATION, "relu") == 0) ? relu : 
                       (strcmp(L1_ACTIVATION, "tanh") == 0) ? tanh_activation : nullptr;
    layer2_activation = (strcmp(L2_ACTIVATION, "relu") == 0) ? relu : 
                       (strcmp(L2_ACTIVATION, "tanh") == 0) ? tanh_activation : nullptr;
}

void NeuralNetwork::updateSensorState(const float* sensors, int sensor_count) {
    // Update current sensor readings
    for(int i = 0; i < sensor_count && i < MAX_SENSORS; i++) {
        current_sensors[i] = sensors[i];
    }
    sensors_updated = true;
    
    // Add to ring buffer only if we're using sensor past states
    if(PAST_STATES_ENABLED && PAST_STATES_USE_SENSORS) {
        for(int i = 0; i < sensor_count && i < MAX_SENSORS; i++) {
            past_states_buffer[buffer_index][i] = sensors[i];
        }
        
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
        if(buffer_index == 0) buffer_full = true;
    }
}

void NeuralNetwork::updateWheelState(const float* wheels, int wheel_count) {
    // Add to ring buffer only if we're using wheel past states
    if(PAST_STATES_ENABLED && PAST_STATES_USE_WHEELS) {
        for(int i = 0; i < wheel_count && i < MAX_WHEELS; i++) {
            past_states_buffer[buffer_index][i] = wheels[i];
        }
        
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
        if(buffer_index == 0) buffer_full = true;
    }
}

void NeuralNetwork::buildInputVector(float* input_vector) {
    int input_idx = 0;
    
    // Add current sensor readings (always first 3 inputs)
    for(int i = 0; i < MAX_SENSORS; i++) {
        input_vector[input_idx++] = current_sensors[i];
    }
    
    // Add past states if enabled
    if(PAST_STATES_ENABLED && PAST_STATES_COUNT > 0) {
        constexpr int stride_size = PAST_STATES_USE_SENSORS ? MAX_SENSORS : MAX_WHEELS;
        
        for(int state = 0; state < PAST_STATES_COUNT; state++) {
            // Calculate the buffer index for this past state
            int steps_back = (state + 1) * PAST_STATES_STRIDE;
            int buffer_idx = (buffer_index - steps_back + BUFFER_SIZE) % BUFFER_SIZE;
            
            // Add past state values
            for(int i = 0; i < stride_size; i++) {
                if(buffer_full || buffer_idx < buffer_index) {
                    input_vector[input_idx++] = past_states_buffer[buffer_idx][i];
                } else {
                    input_vector[input_idx++] = 0.0f;  // No data available yet
                }
            }
        }
    }
}

template<int MAX_IN>
void NeuralNetwork::forwardLayer(const float* input, float* output,
                                const float weights[][MAX_IN], const float* biases,
                                int input_size, int output_size, ActivationFunc activation_func) {
    
    for(int out = 0; out < output_size; out++) {
        float sum = biases[out];
        
        for(int in = 0; in < input_size; in++) {
            sum += input[in] * weights[out][in];
        }
        
        // Apply activation function - no string comparison!
        if(activation_func) {
            output[out] = activation_func(sum);
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
    forwardLayer<L0_IN>(full_input, layer0_output, L0_WEIGHTS, L0_BIASES, L0_IN, L0_OUT, layer0_activation);
    
    // Layer 1: Hidden1 -> Hidden2  
    forwardLayer<L1_IN>(layer0_output, layer1_output, L1_WEIGHTS, L1_BIASES, L1_IN, L1_OUT, layer1_activation);
    
    // Layer 2: Hidden2 -> Output
    forwardLayer<L2_IN>(layer1_output, output, L2_WEIGHTS, L2_BIASES, L2_IN, L2_OUT, layer2_activation);
    
    // Scale outputs from [-1,1] to [0,1] if reverse is not allowed
    if(!ACTOR_ALLOW_REVERSE) {
        for(int i = 0; i < ACTOR_OUTPUT_DIM; i++) {
            output[i] = (output[i] + 1.0f) / 2.0f;
        }
    }
}