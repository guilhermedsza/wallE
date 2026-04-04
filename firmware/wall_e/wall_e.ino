/*
  Needs to be on v 3.1.3 of esp32 by Espressif Systems to work (https://forum.arduino.cc/t/compilar-library-ps4-controler/1373660/4)
  
  Required Libraries for this project:

  1. Adafruit PWM Servo Driver
     - Open Arduino IDE
     - Go to: Sketch → Include Library → Manage Libraries...
     - Search: "Adafruit PWM Servo Driver"
     - Install the official library by Adafruit

  2. RemoteDebug
     - Open Arduino IDE
     - Go to: Sketch → Include Library → Manage Libraries...
     - Search: "RemoteDebug"
     - Install the library by Joao Lopes

  3. PS5 Controller for ESP32
     - Download manually from: https://github.com/rodneybakiskan/ps5-esp32
     - In Arduino IDE, go to: Sketch → Include Library → Add .ZIP Library...
     - Select the downloaded ZIP file
*/

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







