#pragma once
// #ifndef WIFI_H
// #define WIFI_H
#include "Utils.h"
#include "WiFiS3.h"
#include "arduino_secrets.h"

void wifi_init(){
  print_INFO("WIFI","setup wifi");
  delay(1000);
  
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    print_INFO("WIFI","Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    print_WARN("WIFI","Please upgrade the firmware");
  }

  int status = WL_IDLE_STATUS;
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    print_INFO("WIFI",String("Attempting to connect to SSID: " + String(WIFI_SSID)).c_str());

    status = WiFi.begin(WIFI_SSID, WIFI_PASS);

    delay(1000);
  }

  print_INFO("WIFI","Connected!");

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  print_INFO("WIFI",String("IP Address: " + ip.toString()).c_str());
}

String get_MAC(){
  uint8_t mac[6];
  String MAC = "";
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      MAC += ":";
    }
    if (mac[i] < 16) {
      MAC += "0";
    }
    MAC += String(mac[i],HEX);
  }
  return MAC;
}
// #endif