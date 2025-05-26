/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-auto-pairing-esp32-esp8266/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
  Based on JC Servaye example: https://github.com/Servayejc/esp_now_sender/

  modify for personal used by
  NetYan
*/
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "SHT3xSensor.h"
#include <wire.h>

// Set your Board and Server ID 
#define BOARD_ID 2    // Sensor Number 
#define MAX_CHANNEL 13  // 11 in North America or 13 in Europe
int LED_BUILTIN = 2;



uint8_t serverAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t clientMacAddress[6];

// Structure to send data
// Must match the receiver structure
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint8_t msgType;
  uint8_t id;
  float temp;
  float hum;
  unsigned int readingId;
} struct_message;

typedef struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

esp_now_peer_info_t peer;

// Create 2 struct_message 
struct_message myData;  // data to send
struct_message inData;  // data received
struct_pairing pairingData;

enum PairingStatus {NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED,};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType {PAIRING, DATA, RESET, ADD_SERVER, CONFIRM};  //++ Add RESET to enum for Send Reset Command, add ADD_SERVER , CONFIRM
MessageType messageType;

#ifdef SAVE_CHANNEL
  int lastChannel;
#endif  
int channel = 1;
 
// simulate temperature and humidity data
// float t = 0;
// float h = 0;
// Sensor object
//++ modify for add many server along running
#define MAX_PEERS 5   //only 5 Server add per sender 
#define MAX_PAIRED_SERVERS 5
uint8_t pairedServers[MAX_PEERS][6];
int pairedCount = 0;
//++ for check server mac_addr is in more than limit or not
bool isAlreadyPaired(const uint8_t * mac_addr) {
  for (int i = 0; i < pairedCount; i++) {
    if (memcmp(pairedServers[i], mac_addr, 6) == 0) {
      Serial.println("bool isAlreadyPaired is ture:: mean there are exiting peer mac_addr");
      return true;
    }
  }
  return false;
}
//++ for check adding server mac_addr is AlreadyPaired or not to prevent duplicated mac_addr
bool addPairedServer(const uint8_t * mac_addr) {
  if (pairedCount >= MAX_PAIRED_SERVERS) {
    Serial.println("It Reach Max Peer(Server) mac_addr in memory") ;
    return false;
  }
  if (isAlreadyPaired(mac_addr)) return false;

  memcpy(pairedServers[pairedCount], mac_addr, 6);
  pairedCount++;
  return true;
}
//++ for send EspNow data to all server in listed
void sendToAllServers(struct_message data) {
  for (int i = 0; i < pairedCount; i++) {
    Serial.println("Start sendToAllServers Function");
    esp_now_send(pairedServers[i], (uint8_t *)&data, sizeof(data));
  }
}

//-- modify for add many server along running

SHT3xSensor sht3x;
bool sht3xAvailable = false;  // Track sensor availability
//++ for check I2C connect or not in this case I2C = Sht3x
// Replace with your actual device's I2C address
#define DEVICE_ADDR 0x44
bool checkI2CDevice(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();

  return (error == 0); // true if device ACKs
}
//-- for check I2C connect or not in this case I2C = Sht3x

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 100000;        // Interval at which to publish sensor readings
unsigned long start;                // used to measure Pairing time
unsigned int readingId = 0;   


void readGetMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
  clientMacAddress[0] = baseMac[0];
  clientMacAddress[1] = baseMac[1];
  clientMacAddress[2] = baseMac[2];
  clientMacAddress[3] = baseMac[3];
  clientMacAddress[4] = baseMac[4];
  clientMacAddress[5] = baseMac[5];
}

// ++ NotUse when connect with RealSensor
// simulate temperature reading
// float readDHTTemperature() {
//   t = random(0,40);
//   return t;
// }

// // simulate humidity reading
// float readDHTHumidity() {
//   h = random(0,100);
//   return h;
// }
// -- NotUse when connect with RealSensor
//++ Sht30 Reading Function
void Sht30_Reading() {
  if (sht3x.readMeasurement(myData.temp, myData.hum)) {
    Serial.print("Sht30Temperature: ");
    Serial.print(myData.temp);
    Serial.print(" C\tHumidity: ");
    Serial.print(myData.hum);
    Serial.println(" %");
  } else {
    Serial.print("Error reading measurement: ");
    Serial.println(sht3x.getLastError());
  }
}
//-- Sht30 Reading Function


void addPeer(const uint8_t * mac_addr, uint8_t chan){
  Serial.println("AddPeer Funtion Begin");
  ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  Serial.println("End addPeer");
  if (esp_now_add_peer(&peer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Finding Receiveer Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received with ");
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type) {
  case DATA :      // we received data from server
    memcpy(&inData, incomingData, sizeof(inData));
    Serial.print("ID  = ");
    Serial.println(inData.id);
    Serial.print("Setpoint temp(Get form Reveiver Peer) = ");
    Serial.println(inData.temp);
    Serial.print("SetPoint humidity(Get form Reveiver Peer) = ");
    Serial.println(inData.hum);
    Serial.print("reading Id(Receiver Peer sending count)  = ");
    Serial.println(inData.readingId);

    if (inData.readingId % 2 == 1){
      digitalWrite(LED_BUILTIN, LOW);
    } else { 
      digitalWrite(LED_BUILTIN, HIGH);
    }
    break;

  case PAIRING:    // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == 0) {              // the message comes from server
      Serial.print("Pairing done for MAC Address: ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel " );
      Serial.print(pairingData.channel);    // channel used by the server
      Serial.print(" in ");
      Serial.print(millis()-start);
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list 
      #ifdef SAVE_CHANNEL
        lastChannel = pairingData.channel;
        EEPROM.write(0, pairingData.channel);
        EEPROM.commit();
      #endif  
      pairingStatus = PAIR_PAIRED;             // set the pairing status
    }
    break;

  case RESET: // Incase ESP32 peers who connect router have troble Wifi connection 
    for (int i = 0; i < 10; i++)
    {
      Serial.println("Get Reset Command from Peer Server");
      Serial.println("prepare to Reset");
      delay(1000);
    }
    ESP.restart();
    break;

  case ADD_SERVER:  // we received add server request from new server
    struct_pairing pairingData;
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (addPairedServer(mac_addr)) {
      Serial.print("New server added: ");
      printMAC(mac_addr);
      // send confirmation back to the new server
      struct_message confirmData;
      confirmData.msgType = CONFIRM;
      confirmData.id = BOARD_ID;
      esp_now_send(mac_addr, (uint8_t *)&confirmData, sizeof(confirmData));
    } else {
      Serial.println("Failed to add new server");
    }
    break;
  }  
}

PairingStatus autoPairing(){
  switch(pairingStatus) {
    case PAIR_REQUEST:
      Serial.print("Pairing request on channel "  );
      Serial.println(channel);

      // set WiFi channel   
      ESP_ERROR_CHECK(esp_wifi_set_channel(channel,  WIFI_SECOND_CHAN_NONE));
      if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
      }

      // set callback routines
      esp_now_register_send_cb(OnDataSent);
      esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
    
      // set pairing data to send to the server
      pairingData.msgType = PAIRING;
      pairingData.id = BOARD_ID;     
      pairingData.channel = channel;
      pairingData.macAddr[0] = clientMacAddress[0];
      pairingData.macAddr[1] = clientMacAddress[1];
      pairingData.macAddr[2] = clientMacAddress[2];
      pairingData.macAddr[3] = clientMacAddress[3];
      pairingData.macAddr[4] = clientMacAddress[4];
      pairingData.macAddr[5] = clientMacAddress[5];

      // add peer and send request
      addPeer(serverAddress, channel);
      esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
      previousMillis = millis();
      pairingStatus = PAIR_REQUESTED;
      break;

    case PAIR_REQUESTED:
      // time out to allow receiving response from server
      currentMillis = millis();
      if(currentMillis - previousMillis > 1000) {
        previousMillis = currentMillis;
        // time out expired,  try next channel
        channel ++;
        if (channel > MAX_CHANNEL){
          channel = 1;
        }   
        pairingStatus = PAIR_REQUEST;
      }
    break;

    case PAIR_PAIRED:
      // nothing to do here 
    break;
  }
  return pairingStatus;
}  

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);

  //++ for check I2C connect or not in this case I2C = Sht3x
    Wire.begin(21, 22); // SDA, SCL

    bool connected = checkI2CDevice(DEVICE_ADDR);
    
    if (connected) {
      Serial.println("checkI2CDevice Device is connected!");
      // ++Attempt to initialize the SHT30 sensor
      sht3xAvailable = sht3x.begin();
      if (sht3xAvailable) {
        Serial.println("SHT3x sensor initialized successfully.");
      } else {
        Serial.println("SHT3x sensor not found, using random data instead.");
      }
// --Attempt to initialize the SHT30 sensor
    } else {
      Serial.println("checkI2CDevice Device not found!");
      sht3xAvailable = false;
    }
    //-- for check I2C connect or not in this case I2C = Sht3x
  

  
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  Serial.print("Client Board MAC Address:  ");
  readGetMacAddress();
  WiFi.disconnect();
  start = millis();

  #ifdef SAVE_CHANNEL 
    EEPROM.begin(10);
    lastChannel = EEPROM.read(0);
    Serial.println(lastChannel);
    if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL) {
      channel = lastChannel; 
    }
    Serial.println(channel);
  #endif  
  pairingStatus = PAIR_REQUEST;
}  

void loop() {
  if (autoPairing() == PAIR_PAIRED) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      //Set values to send
      myData.msgType = DATA;
      myData.id = BOARD_ID;
      // myData.temp = readDHTTemperature();
      // myData.hum = readDHTHumidity();
// ++for test with SHT30
      if (sht3xAvailable) {  
        Sht30_Reading();  // Function to read data via SHT30
      } else {
        // Generate random data since SHT30 is not available
        float t = random(25,40);
        myData.temp =  t;
        float h = random(65,99);
        myData.hum = h;
        for (int i = 0; i<10; ++i)
        {
          Serial.println("Sht30 Error Reading");
          Serial.print("Simulate Temp = ");
          Serial.println(t);
          Serial.print("Simulate Humidity = ");
          Serial.println(h);


        }
    }
// --for test with SHT30
      myData.readingId = readingId++;
      sendToAllServers(myData);
      Serial.println("End Read and send data");
    }

    // Broadcast channel and MAC address to find new servers
    static unsigned long lastBroadcastTime = 0;
    if (millis() - lastBroadcastTime >= 10000) {
      Serial.println("Continuiously finding new Server ");
      lastBroadcastTime = millis();
      struct_pairing pairingData;
      pairingData.msgType = PAIRING;
      pairingData.id = BOARD_ID;
      memcpy(pairingData.macAddr, clientMacAddress, 6);
      pairingData.channel = channel;
      esp_now_send(serverAddress, (uint8_t *)&pairingData, sizeof(pairingData));
      Serial.println("End loop finding new Server");
    }
  }
}