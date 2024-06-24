/*
  Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLETests/SampleNotify.cpp
  Ported to Arduino ESP32 by Evandro Copercini
  updated by chegewara and MoThunderz
  added sensor specific interpretation by Claudiu Cercel
  EC (electical conductivity) library used and modified for the project: https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK
*/

//libraries used for the BLE server
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//library used to create the notified message 
#include <sstream>

//libraries used for the ec sensor
#include <Wire.h> 
#include "DFRobot_ESP_EC.h"
#include "EEPROM.h"

//initialise the server, characteristic and descriptor
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
BLEDescriptor *pDescr;
BLE2902 *pBLE2902;

//create an ec (electrical conductivity sensor) object
DFRobot_ESP_EC ec;

bool deviceConnected = false;
bool oldDeviceConnected = false;

//Lolin 32 specific variables used to convert the analog values into voltage
const float VCC = 3.3; // Power supply voltage
const int ADC_BITS = 12; // ADC resolution
const int ADC_COUNTS = 1 << ADC_BITS; // 2^ADC_BITS

/*  See the following for generating UUIDs:
https://www.uuidgenerator.net/  */
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

/*  function used to extract the ph value from a PH-4502C sensor
check https://cdn.awsli.com.br/969/969921/arquivos/ph-sensor-ph-4502c.pdf for calibration instructions  */
float getPh(){
  int measurings = 0;
  for (int i = 0; i < 10; i++){
    measurings += analogRead(A0);
    delay(5);
  }
  float voltage = VCC / ADC_COUNTS * measurings / 10;
  float PH = -9.0909 * voltage + 22.374;
  return PH;
}

/*  function used to extrat the turbidity value from a grove turbidity sensor: https://www.seeedstudio.com/Grove-Turbidity-Sensor-p-4399.html
chech the thesis for calibration instructions and also the sensor datasheet: https://files.seeedstudio.com/wiki/Grove-Turbidity-Sensor/res/Turbidity-Sensor-Datasheet.pdf */
float getTurbidity(){
  float voltage = VCC / ADC_COUNTS * analogRead(A3);
  float NTU = -1 * voltage * voltage - 5 * voltage +20;
  return NTU;
}

/*  function used to extract the ec value from a DFRobot ec kit: https://www.dfrobot.com/product-1123.html
make sure to modify the DFRobot_ESP_EC.h and .cpp to have the correct K value and to not show the values on the serial monitor*/
float getConductivity(){
  float voltage = analogRead(32);
  float temperature = 25;
  float ecValue = ec.readEC(voltage, temperature);
  return ecValue;
}

void setup() {
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,BLECharacteristic::PROPERTY_NOTIFY);                   

  // Create a BLE Descriptor
  pDescr = new BLEDescriptor((uint16_t)0x2901);
  pDescr->setValue("A very interesting variable");
  pCharacteristic->addDescriptor(pDescr);
  
  // Add 2902 descriptor to the characteristic
  pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902);

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  EEPROM.begin(32);  //needed EEPROM.begin to store calibration K in eeprom
  ec.begin();        //by default lib store calibration K since 10 change it by set ec.begin(30); to start from 30
}

void loop() {
  // notify changed value
  if (deviceConnected) {
    //collect the parameters
    float ph = getPh();
    delay(10);
    float turbidity = getTurbidity();
    delay(10);
    float conductivity = getConductivity();

    //assemble the message using the collected values
    std::ostringstream messageStream;
    messageStream << ph << " " << turbidity << " " << conductivity;
    std::string message = messageStream.str();

    //set the message ad notify
    pCharacteristic->setValue(message);
    pCharacteristic->notify();
    
    //set the delay to 1sec
    delay(1000);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("Start advertising!");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
    Serial.println("Client connected successfully!");
  }
}