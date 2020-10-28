#ifndef WIFI_MANAGER
#define WIFI_MANAGER

#include <WiFi.h>
// Replace the next variables with your SSID/Password combination
const char *ssid = "FRITZ!Box Fon WLAN 7320";
const char *password = "1019470067459406";

WiFiClient espClient;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  int tries = 10;

  while ((WiFi.status() != WL_CONNECTED)&& tries) {
    delay(500);
    Serial.print(".");
    tries --;
  }

  if (!tries) esp_restart();

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}


#endif