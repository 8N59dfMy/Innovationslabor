//#include <Arduino.h> // not necessary in the ESP32 Library
#include <WiFi.h>

const char ssid[] = "Soft_AP"; // SSID
const char password[] = "12345678"; // at least 8 characters!
IPAddress hostadresse;
int ESP_wifi_mode;

void setup() {
  Serial.begin(115200);
  delay(1000); // Wait for Serial to initialize
  
  Serial.println("Setup WIFI Network in SoftAP Mode.");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Secret password: ");
  Serial.println(password);

  WiFi.mode(WIFI_AP); // Set WIFI-Mode in AP-Mode
  WiFi.softAP(ssid, password);  // Start ESP as Access-Point
  hostadresse = WiFi.softAPIP();
  ESP_wifi_mode = WiFi.getMode();
}

void loop() {
  Serial.println("********************");
  Serial.print("WiFi-Modus: ");
  Serial.println(ESP_wifi_mode);
  Serial.print("Hostadresse: ");
  Serial.println(hostadresse);
  delay(10000);
}
  