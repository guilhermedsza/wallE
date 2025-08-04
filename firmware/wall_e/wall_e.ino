#include "ota.h"
#include "wifi.h"
#include "servo.h"
#include "logger.h"
#include "controller.h"

#include "../config/pins.h"
#include "../config/secrets.h"

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
// https://dronebotworkshop.com/esp32-servo/

void setup () {
  wifiConnect(WIFI_SSID, WIFI_PASS);
  loggerSetup();
  otaSetup();

  //PCA setup
  pca9685.begin();
  servoSetup(pca9685); 

  controllerSetup(PS5_MAC);

  LOG("Setup complete, OTA + RemoteDebug ready");
}

void loop() {
  loggerLoop();
  otaLoop();
  controllerLoop();
}







