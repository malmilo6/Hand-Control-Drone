 
#include <esp_now.h>
#include <WiFi.h>
#include "sbus.h"

// Assuming Serial2 is used for SBUS and requires inversion, modify according to your hardware setup
bfs::SbusTx sbus_tx(&Serial1,37,2,true);

typedef struct struct_message {
  char a[32];
  float accX;
  float accY;
  float accZ;

  float gyroX;
  float gyroY;
  float gyroZ;
} struct_message;

// Create a struct_message called myData
struct_message myData;
bfs::SbusData sbusData;

// callback function that will be executed when data is received
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));

  // Assuming the accelerometer and gyroscope values should be mapped to 1000 - 2000 range for SBUS
  sbusData.ch[0] = map(myData.accX * 1000, -32768, 32767, 1000, 2000);  // Roll
  sbusData.ch[1] = map(myData.accY * 1000, -32768, 32767, 1000, 2000);  // Pitch
  sbusData.ch[2] = map(myData.gyroZ * 1000, -32768, 32767, 1000, 2000); // Yaw - typically gyro Z-axis
  sbusData.ch[3] = map(myData.accZ * 1000, -32768, 32767, 1000, 2000);  // Throttle - assuming you want to use accZ for throttle

  // AUX channels can be mapped from additional data or set to default values if not used
  sbusData.ch[4] = 1500; // AUX 1 - Default to mid-position
  sbusData.ch[5] = 1500; // AUX 2 - Default to mid-position
  sbusData.ch[6] = 1500; // AUX 3 - Default to mid-position
  sbusData.ch[7] = 1500; // AUX 4 - Default to mid-position

  // Update and transmit SBUS data
  sbus_tx.data(sbusData);
  sbus_tx.Write();

  // Debugging output
  Serial.print("Received data: AccX=");
  Serial.print(myData.accX);
  Serial.print(", GyroX=");
  Serial.println(myData.gyroX);
}

 
void setup() {
  Serial.begin(115200);
  Serial2.begin(100000, SERIAL_8E2); // Configure Serial2 for SBUS
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv); // Register callback for receiving data
}
 
void loop() {

}