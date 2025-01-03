/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-mqtt-publish-bme680-arduino/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <WiFi.h>
extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include "Sensirion_upt_i2c_auto_detection.h"


#define WIFI_SSID "Soft_AP"
#define WIFI_PASSWORD "12345678"

// Raspberry Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 4, 3)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_PM1P0 "SEN55/PM1P0"
#define MQTT_PUB_PM2P5 "SEN55/PM2P5"
#define MQTT_PUB_PM4P0 "SEN55/PM4P0"
#define MQTT_PUB_PM10P0 "SEN55/PM10P0"
#define MQTT_PUB_HUM "SEN55/HUM"
#define MQTT_PUB_TEMP "SEN55/TEMP"
#define MQTT_PUB_VOC "SEN55/VOC"
#define MQTT_PUB_NOX "SEN55/NOX"

//SEN55 Initialisierung
I2CAutoDetector i2CAutoDetector(Wire);
SensorManager sensorManager(i2CAutoDetector);
int maxNumSensors;
const MeasurementList **dataPointers;

// Variables to hold sensor readings
float PM1P0_value;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

void getSEN55Readings(){
   Serial.setTimeout(2000);
  // 2.1 Task SensorManager to fetch sensor data
  sensorManager.refreshAndGetSensorReadings(dataPointers);

  const MeasurementList measurementList = *dataPointers[4];
  const Measurement PM1P0 = measurementList.getMeasurement(0);
  const Measurement PM2P5 = measurementList.getMeasurement(1);
  const Measurement PM4P0 = measurementList.getMeasurement(2);
  const Measurement PM10P0 = measurementList.getMeasurement(3);
  const Measurement RH = measurementList.getMeasurement(4);
  const Measurement T = measurementList.getMeasurement(5);
  const Measurement VOC = measurementList.getMeasurement(6);
  const Measurement NOX = measurementList.getMeasurement(7);

  PM1P0_value = PM1P0.dataPoint.value;

/*------------------------------------------------------------------------------
                                      PM1P0                                
  ----------------------------------------------------------------------------*/
  Serial.print("PM1P0: ");
  Serial.print(PM1P0_value);
  Serial.println(" ug/m3");
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.setCredentials("Raspberry", "f8ÜZg02P82KK");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();

// 1.2 Initialize the I2C bus on pins 35 (SDA) and 36 (SCL)
  int sda_pin = 35;
  int scl_pin = 36;
  Wire.begin(sda_pin, scl_pin);

  // 1.3 Build the "reverse PO Box" we'll use to retrieve sensor data
  maxNumSensors = sensorManager.getMaxNumberOfSensors();
  dataPointers = new const MeasurementList* [maxNumSensors] { nullptr };
  
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  connectToWifi();
  
}

void loop() {
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    
    getSEN55Readings();
    Serial.println();
    Serial.printf("PM1P0 = %.2f ºC \n", PM1P0_value);
    
    // Publish an MQTT message on topic esp/bme680/temperature
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_PM1P0, 1, true, String(PM1P0_value).c_str());
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_PM1P0, packetIdPub1);
    Serial.printf("Message: %.2f \n", PM1P0_value);

  }
}
