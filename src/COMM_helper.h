#ifndef COMM_helper_h
#define COMM_helper_h

// Include required libraries
#include "ADC_helpers.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include<espnow.h>
#include <config.h>

/* MAC address storage */
extern uint8_t peerMac[6]; // Declare as extern


// Handles for ESP now functions
/**
 * @brief Callback function for ESP-NOW data sent event.
 * 
 * @param mac_addr The MAC address of the receiver.
 * @param sendStatus The status of the send operation (0 for success, non-zero for failure).
 * 
 * This function is called when data is sent via ESP-NOW. It prints the status of the
 * send operation to the serial monitor.
 */
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus); // Send data
/**
 * @brief Callback function for ESP-NOW data received event.
 * 
 * @param mac_addr The MAC address of the sender.
 * @param incomingData The received data.
 * @param len The length of the received data.
 * 
 * This function is called when data is received via ESP-NOW. It updates the calibration_handle
 * structure with the received data and prints the received values to the serial monitor.
 */
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len); // Receive data
/**
 * @brief Function to scan for the slave device.
 * 
 * This function scans for the slave device with the SSID "RX" and stores the MAC address of the slave device.
*/
void ScanForSlave(); // Scan for slave device
#endif