#include "wifi.h"
#include <WiFi.h>

bool wifiConnect(const char* ssid, const char* pass, unsigned timeout)
{
    WiFi.begin(ssid, pass);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < timeout) {
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("Wi-Fi OK: %s", WiFi.localIP().toString().c_str());
        return true;
    }
    Serial.print("Wi-Fi FAILED");
    return false;
}