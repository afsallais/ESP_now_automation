// Include required libraries
#include "ADC_helpers.h"
#include "COMM_helper.h"

// Sonar structure TaskHandle_t 
depth_t depth_handle;
calibration_t calibration_handle;

/**
 * @brief Measure the depth using the sonar sensor.
 * 
 * This function reads the raw value from the sonar sensor, converts it to the actual
 * depth value, and updates the depth_handle structure with the measured values.
 */
void measuredepth() {
    int raw_val = analogRead(A0);
    // convert sonar sensor raw value to actual depth value
    float depth_val = (raw_val * 5.0) / 1024;
    depth_handle.raw_val = raw_val;
    depth_handle.depth_val = depth_val;
    Serial.print("Raw Value: ");
    Serial.println(depth_handle.raw_val);
    Serial.print("Depth Value: ");
    Serial.println(depth_handle.depth_val);
}
