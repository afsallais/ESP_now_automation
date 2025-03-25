// Include required header file
#include "COMM_helper.h"
#include "ADC_helpers.h"

// Channel
int8_t CHANNEL;
/* Define peerMac here */
uint8_t peerMac[6] = {0};

// Handles for ESP now functions
// on data sent callback function
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if(DEVICE_ROLE == MASTER)
  {
    // Send calibration and offset data to slave
    Serial.println("Sending Data via ESP-NO to Slave");
    esp_now_send(mac_addr,(uint8_t*)&calibration_handle,sizeof(calibration_handle));
  }
  else{
  Serial.println("Sending Data via ESP-NO to MASTER");
  Serial.println(sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
  esp_now_send(mac_addr,(uint8_t*)&depth_handle,sizeof(depth_handle));
}
}
// on data received callback function
void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
  if(DEVICE_ROLE == MASTER)
  {
    // copy depth data to MASTER depth_handle
    memcpy(&depth_handle,incomingData,sizeof(depth_handle));
    if(sizeof(depth_handle) != len)
    {
      Serial.println("MASTER: Received Data Size Mismatch");
      return;
    }
    Serial.print("Data Received from Slave: ");
    // Print the depth values
    Serial.print("depth_handle.raw_val: ");
    Serial.println(depth_handle.raw_val);
    Serial.print("depth_handle.depth_val: ");
    Serial.println(depth_handle.depth_val);
  }
  else{
  // Copy the received data to the calibration_handle structure
  memcpy(&calibration_handle, incomingData, sizeof(calibration_handle));
  if (sizeof(calibration_handle) != len) {
    Serial.println("Slave: Received Data Size Mismatch");
    return;
  }
  Serial.print("Data Received from MASTER: ");
  // Print the offset values
  Serial.print("calibration_handle.offset: ");
  Serial.println(calibration_handle.offset);
  // Print the calibration values
  Serial.print("calibration_handle.offset: ");
  Serial.println(calibration_handle.correction);
}
}
/*
  * @brief Function to scan for the slave device.
  * 
  * This function scans for the slave device with the SSID "RX" and stores the MAC address of the slave device.
*/

void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  for (int i = 0; i < scanResults; i++) {
    String SSID = WiFi.SSID(i);
    String BSSIDstr = WiFi.BSSIDstr(i);

    if (SSID.indexOf("RX") == 0) {  // Checking for the slave's SSID
      int mac[6];
      if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", 
                      &mac[0], &mac[1], &mac[2], 
                      &mac[3], &mac[4], &mac[5])) {
        for (int ii = 0; ii < 6; ++ii) {
          peerMac[ii] = (uint8_t)mac[ii];
        }
      }

      Serial.println("Slave Found!");
      
      // Print the MAC address of the found slave
      Serial.print("Peer MAC: ");
      for (int j = 0; j < 6; j++) {
        Serial.printf("%02X:", peerMac[j]);
      }
      Serial.println();

      // Check if the peer is already added
      if (!esp_now_is_peer_exist(peerMac)) {  
        Serial.println("Peer not found, adding...");

        if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_SLAVE, CHANNEL, NULL, 0) != 0) {
          Serial.println("Failed to add peer");
        } else {
          Serial.println("Peer added successfully");
        }
      } else {
        Serial.println("Peer already exists, no need to add.");
      }
      break;
    }
  }
}
