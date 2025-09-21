#pragma once

// ========================================
// Simple Compile-Time Logging System
// ========================================
// Change this define to enable/disable ALL logging
// Set to 1 for debug builds, 0 for release builds
#define ENABLE_LOGGING 1

#if ENABLE_LOGGING
    // When logging enabled - compile in all print statements
    #define LOG_INIT(baud)      Serial.begin(baud)
    #define LOG_INFO(...)       Serial.printf(__VA_ARGS__)
    #define LOG_PRINTLN(...)    Serial.println(__VA_ARGS__)
    #define LOG_TIMING(...)     Serial.printf(__VA_ARGS__)
    #define LOG_DEBUG(...)      Serial.printf(__VA_ARGS__)
    #define LOG_ERROR(...)      Serial.printf(__VA_ARGS__)
#else
    // When logging disabled - all macros expand to nothing
    // No runtime overhead, no flash storage for strings
    #define LOG_INIT(baud)      ((void)0)
    #define LOG_INFO(...)       ((void)0)
    #define LOG_PRINTLN(...)    ((void)0)
    #define LOG_TIMING(...)     ((void)0)
    #define LOG_DEBUG(...)      ((void)0)
    #define LOG_ERROR(...)      ((void)0)
#endif

// Usage Examples:
// LOG_INIT(115200);                                    // Replace Serial.begin()
// LOG_INFO("Setup complete\n");                        // Replace Serial.println()
// LOG_TIMING("[TIMING] Sensor read: %lu us\n", time); // Replace Serial.printf()
// LOG_DEBUG("Debug info: %d\n", value);               // Debug-specific prints
// LOG_ERROR("Error: %s\n", error_msg);                // Error messages
