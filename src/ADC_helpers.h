#ifndef ADC_helpers_h
#define ADC_helpers_h    

// Include required libraries
#include <Arduino.h>
#include <espnow.h>
#include "COMM_helper.h"
#include <config.h>

// Structure for depth sensor values and pump status
typedef struct {
  volatile int raw_val;
  volatile float depth_val;
  float offset;
  float correction;
  bool pump_status;
} depth_t;

// Structure for calibration
typedef struct {
  float offset;
  float correction;
} calibration_t;

// Sonar structure TaskHandle_t
extern depth_t depth_handle;
extern calibration_t calibration_handle;

/**
 * @brief Measure the depth using the sonar sensor.
 * 
 * This function reads the raw value from the sonar sensor, converts it to the actual
 * depth value, and updates the depth_handle structure with the measured values.
 */
void measuredepth();

#endif // ADC_helpers_h