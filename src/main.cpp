// Include the necessary libraries
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "ADC_helpers.h"
#include "COMM_helper.h"
#include <time.h>
#include <Ticker.h>  // Include the Ticker library
#include <config.h>



// MAC address for master and slave
uint8_t slave_mac[] = {0x80, 0x7D, 0x3A, 0x40, 0xFD, 0x25};
uint8_t master_mac[] = {0xC4, 0xD8, 0xD5, 0x38, 0x73, 0x19};
extern int8_t CHANNEL;

// Define the pins for the status LED
#define LED1_GREEN D1  // Define the pin for the green LED
#define LED2_RED D2    // Define the pin for the red LED
// ON/OFF Macro
#define ON 1           // Define the ON state
#define OFF 0          // Define the OFF state
// Relay pin 
#define RELAY_PIN D3    // Define the pin for the relay

// Define hours, minutes, seconds
volatile int hours = 0, minutes = 0, seconds = 0; // default night 12am

// Time calculation function
void milli_time(unsigned long int milli);
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus);
void OnDataRecv(uint8_t *mac_addr, uint8_t *data, uint8_t data_len);
void ScanForSlave();
Ticker Timer1;// Create a Ticker object to manage the timer interrupt


/**
 * @brief Interrupt service routine to handle the timer interrupt.
 * 
 * 
 * This function gets called as interrupt handler when the timer interrupt occurs.
 * It checks the depth value and sends the pump ON or OFF command based on the depth value.
 */
void interrupt_Handler()
{
  static int retryCount = 0; // Counter to track retries
  const int maxRetries = 5; // Maximum number of retries
  const unsigned long retryDelay = 2000; // Delay between retries in milliseconds
  static unsigned long lastRetryTime = 0; // Timestamp of the last retry

  if(DEVICE_ROLE == MASTER)
  {
    if(depth_handle.pump_status == ON)
    {
    // Turn on relay at pin D3
    Serial.println("Master: turning on pump");
    // print depth value
    Serial.print("Depth: ");
    Serial.println(depth_handle.depth_val);
    digitalWrite(RELAY_PIN, HIGH); // Turn on relay at pin D3
    }
    else
    {
    Serial.println("Master: turning off pump");
    digitalWrite(RELAY_PIN, LOW); // Turn off relay at pin D3
    }
  }
  else {
    // Slave code
    unsigned long currentTime = millis();
    if (retryCount < maxRetries || currentTime - lastRetryTime >= retryDelay) {
      int result = esp_now_send(master_mac, (uint8_t*)&depth_handle, sizeof(depth_handle));
      if (result == 0) {
        Serial.println("Slave: Data sent successfully");
        retryCount = 0; // Reset retry counter on success
      } else {
        Serial.println("Slave: Failed to send data");
        retryCount++;
        lastRetryTime = currentTime; // Update the last retry time
      }
    } else if (retryCount >= maxRetries) {
      Serial.println("Slave: Max retries reached, skipping further attempts");
    }
  }
}

/**
 * @brief Print the depth value to the serial monitor.
 * 
 * This function prints the raw and depth values to the serial monitor.
 */
void print_depth() {
  Serial.print("Raw: ");
  Serial.println(depth_handle.raw_val);
  Serial.print("Depth: ");
  Serial.println(depth_handle.depth_val);
}

/**
 * @brief Setup function to initialize the ESP-NOW and other configurations.
 * 
 * This function sets up the serial communication, initializes the ESP-NOW protocol,
 * sets the device role, registers the send and receive callbacks, and configures the
 * status LED pins.
 */
void setup() {
  int INTR_TIME;
  // Check if the device is a master
  if(DEVICE_ROLE == MASTER){

    INTR_TIME = 8000; // Set the timer interrupt time to 200ms
    // Setup the serial communication
    Serial.begin(115200);
    Serial.println("MAster Setup");
    // setup ESP now for sending data to receiver
    WiFi.disconnect();
    WiFi.mode(WIFI_STA); // Station mode
    //initialize ESP now
    if (esp_now_init() != 0) {
      Serial.println("ESP-NOW initialization failed!");
      return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent); // Register callback function
    esp_now_register_recv_cb(OnDataRecv);// Register callback function
  
    ScanForSlave(); // Scan for MAC address of slave  
    // Add peer manually since ESP8266 doesn't have peer_info struct
    if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_SLAVE, CHANNEL, NULL, 0) != 0) {
    Serial.println("Failed to add peer");
    }
  }
  if(DEVICE_ROLE == SLAVE){
  INTR_TIME = 4000; // Set the timer interrupt time to 1000ms
  // Setup the serial communication
  Serial.begin(115200);
  Serial.println("Slave Setup");
  WiFi.softAP("RX_1", "RX_1_Password", CHANNEL, 0);

  esp_now_init();
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_send_cb(OnDataSent); // Register callback function
  esp_now_register_recv_cb(OnDataRecv);// Register callback function

  }
  // Define pins for LED
  pinMode(LED1_GREEN, OUTPUT);
  pinMode(LED2_RED, OUTPUT);
  unsigned long int milli = millis();
  milli_time(milli);
  // Print the time
  Serial.print("Time: ");
  Serial.print(hours);
  Serial.print(":");
  Serial.print(minutes);
  Serial.print(":");
  Serial.println(seconds);

//get and print the MAC address of the device
Serial.print("MAC Address: ");
Serial.println(WiFi.macAddress());
// Attach the timerISR function to run every 1000 milliseconds
Timer1.attach_ms(INTR_TIME, interrupt_Handler);
Serial.println("Timer interrupt attached.");
}

/**
 * @brief Main loop function to measure depth and control the pump.
 * 
 * This function repeatedly measures the depth, checks the depth value, and
 * controls the pump based on the depth value. It also sends the depth data
 * via ESP-NOW.
 */
void loop() {
  if(DEVICE_ROLE == MASTER) {
    // Master code
    for(int8_t i=0;i<10;i++)
    {
    ScanForSlave(); // Scan for MAC address of slave  
   
    Serial.println("Master: Monitoring Depth");
    print_depth(); // print depth value
    if(depth_handle.depth_val > 3)
    {
      // update pump status
      depth_handle.pump_status = ON;
    }
    else
    {
      // update pump status
      depth_handle.pump_status = OFF;
    }
    // delay 1second
    delay(100);
  }}
  else{
  // put your main code here, to run repeatedly:
  measuredepth();
  delay(1000);
  // Send the depth data via ESP-NOW
  //esp_now_send(master_mac,(uint8_t*)&depth_handle,sizeof(depth_handle));
}
}

/**
 * @brief Calculate the time in hours, minutes, and seconds from milliseconds.
 * 
 * @param milli The time in milliseconds.
 * 
 * This function converts the given time in milliseconds to hours, minutes, and seconds.
 */
void milli_time(unsigned long int milli) {
  hours = (milli / (1000 * 60 * 60)) % 24;
  minutes = (milli / (1000 * 60)) % 60;
  seconds = (milli / 1000) % 60;
}



