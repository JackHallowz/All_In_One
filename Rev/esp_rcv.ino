// Include Libraries
#include <esp_now.h>
#include <WiFi.h>

//B0:B2:1C:A8:83:D4 MAC address main esp32

// Define a data structure to receive
typedef struct struct_message {
  char a[32];
  float b;
  float c;
  float d;
  double e;
  int f;
  int g;
  int h;
  int i;
  int l;
  int m;
} struct_message;
//
uint8_t broadcastAddress [] = {0xB0, 0xB2, 0x1C, 0xA8, 0x83, 0xD4};
// Create a structured object
struct_message myData;
esp_now_peer_info_t peerInfo;
typedef struct delivery
{
  String A;
} delivery;
delivery deli_package;
String data;

//Task Handle
TaskHandle_t Task1;
TaskHandle_t Task2;
char dat;
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print(myData.a);Serial.print("\r,");
  Serial.print(myData.b);Serial.print(",");
  Serial.print(myData.c);Serial.print(",");
  Serial.print(myData.d);Serial.print(",");
  Serial.print(myData.e);Serial.print(",");
  Serial.print(myData.f);Serial.print(",");
  Serial.print(myData.g);Serial.print(",");
  Serial.print(myData.h);Serial.print(",");
  Serial.print(myData.i);Serial.print(",");
  Serial.print(myData.l);Serial.print(",");
  Serial.println(myData.m);
  
  vTaskDelay(1/ portTICK_PERIOD_MS);
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESES ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
   // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
     return;
  }
  Serial.println("Ready To Use");
  Serial.println("Please Enter The Command");
}
void serialEvent()
{
  if(Serial.available()>0)
  { 
    deli_package.A = Serial.readString();
    Serial.println(deli_package.A);
    //int strlength = data.length();
    //deli_package.A = data.charAt(strlength - 1);
    //dat = data.charAt(strlength - 1);
    //Serial.println(dat);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &deli_package, sizeof(deli_package));
    if (result == ESP_OK) 
    {
      Serial.println("Sending confirmed");
    }
    else 
    { 
      Serial.println("Sending error");
    }
    
  }
}
void loop() {
 
}
