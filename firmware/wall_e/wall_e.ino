#include <ESP32Servo.h>

#include <Wire.h> // Wire is for I2C
#include <Adafruit_PWMServoDriver.h>
#include <ps5Controller.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <RemoteDebug.h>
#include "../config/secrets.h"

RemoteDebug Debug;

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
// https://dronebotworkshop.com/esp32-servo/

#define SERVOMIN 80
#define SERVOMAX 600

#define LEFTEYE 0
#define RIGHTEYE 1
#define HEAD 2
#define NECKTOP 3
#define NECKBOTTOM 4
#define LEFTARM 5
#define RIGHTARM 6

int pwm0, pwm1, pwm2, pwm3, pwm4, pwm5, pwm6;

float yawAngle = 95.0;
float yawSpeed = 180.0; //degrees/sec

float pitchSpeed = 180.0;
float pitchAngle = 90.0;

//Servo functions

void runArmsTest() {
  int rightArmHigh = 110;
  int rightArmLow = 40;

  int leftArmLow = 180 - rightArmLow;
  int leftArmHigh = 180 - rightArmHigh;

  setServoPosition(&pwm5, LEFTARM, leftArmHigh);
  setServoPosition(&pwm6, RIGHTARM, rightArmHigh);
  delay(500);

  setServoPosition(&pwm5, LEFTARM, leftArmLow);
  setServoPosition(&pwm6, RIGHTARM, rightArmLow);
  delay(500);
  
  setServoPosition(&pwm5, LEFTARM, leftArmHigh);
  setServoPosition(&pwm6, RIGHTARM, rightArmHigh);
  delay(500);

  setServoPosition(&pwm5, LEFTARM, leftArmLow);
  setServoPosition(&pwm6, RIGHTARM, rightArmLow);
  delay(500);
}


void runNeckTopTest() {
  int neckTopHigh = 180;
  int neckTopLow = 10;

  setServoPosition(&pwm3, NECKTOP, neckTopHigh);
  delay(500);

  setServoPosition(&pwm3, NECKTOP, neckTopLow);
  delay(500);

  setServoPosition(&pwm3, NECKTOP, neckTopHigh);
  delay(500);

  setServoPosition(&pwm3, NECKTOP, neckTopLow);
  delay(500);
}

void runNeckBottomTest() {
  int neckBottomHigh = 180;
  int neckBottomLow = 90;
  // 50

  setServoPosition(&pwm4, NECKBOTTOM, neckBottomHigh);
  delay(500);

  setServoPosition(&pwm4, NECKBOTTOM, neckBottomLow);
  delay(500);

  setServoPosition(&pwm4, NECKBOTTOM, neckBottomHigh);
  delay(500);

  setServoPosition(&pwm4, NECKBOTTOM, neckBottomLow);
  delay(500);
}

void runHeadTest() {
// head -> 100 degrees he looks straight
  int headHigh = 180;
  int headLow = 10;

  setServoPosition(&pwm2, HEAD, headHigh);
  delay(500);
  
  setServoPosition(&pwm2, HEAD, headLow);
  delay(500);

  setServoPosition(&pwm2, HEAD, headHigh);
  delay(500);

  setServoPosition(&pwm2, HEAD, headLow);
  delay(500);

  setServoPosition(&pwm2, HEAD, headHigh);
  delay(500);

  setServoPosition(&pwm2, HEAD, headLow);
  delay(500);

  setServoPosition(&pwm2, HEAD, 95);
  delay(500);
}

void runEyesTest() {
  int rightEyeHigh = 40;
  int rightEyeLow = 80;

  int leftEyeHigh = 180-rightEyeHigh;
  int leftEyeLow = 180-rightEyeLow;

  setServoPosition(&pwm0, LEFTEYE, leftEyeLow);
  setServoPosition(&pwm1, RIGHTEYE, rightEyeLow);
  delay(500);

  setServoPosition(&pwm0, LEFTEYE, leftEyeHigh);
  setServoPosition(&pwm1, RIGHTEYE, rightEyeHigh);
  delay(500);

  setServoPosition(&pwm0, LEFTEYE, leftEyeLow); 
  setServoPosition(&pwm1, RIGHTEYE, rightEyeHigh);
  delay(500);

  setServoPosition(&pwm0, LEFTEYE, leftEyeHigh); 
  setServoPosition(&pwm1, RIGHTEYE, rightEyeHigh);
  delay(500);

  setServoPosition(&pwm0, LEFTEYE, leftEyeLow); 
  setServoPosition(&pwm1, RIGHTEYE, rightEyeHigh);
  delay(500);

  setServoPosition(&pwm0, LEFTEYE, leftEyeHigh);
  setServoPosition(&pwm1, RIGHTEYE, rightEyeLow);  
  delay(500);

  setServoPosition(&pwm1, RIGHTEYE, rightEyeHigh);  
  delay(500);
}

void setServoPosition(int* pwm, int servoChannel, int angle) {
  *pwm = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pca9685.setPWM(servoChannel, 0 , *pwm); 
}

//Dualshock 5 functions

float stickDeadzone = 0.1;

unsigned long lastLogTimeStamp = 0;
unsigned long lastControllerUpdate = 0;

void rightStickController() {
  unsigned long now = millis();
  float deltaTime = (now - lastControllerUpdate) / 1000.0; //.0 to make sure this is a floating-point division instead of an integer division
  lastControllerUpdate = now;

  rxController(deltaTime);
  ryController(deltaTime);
}


void rxController(float deltaTime) {
  float rx = ps5.RStickX()/128.0;
  if(abs(rx) < stickDeadzone) rx = 0.0;
  yawAngle += rx*yawSpeed*deltaTime;
  yawAngle = constrain(yawAngle, 0, 180);
  setServoPosition(&pwm2, HEAD, yawAngle);
}

void ryController(float deltaTime) {
  float ry = ps5.RStickY()/128.0;

  if(abs(ry) < stickDeadzone) ry = 0.0;
  pitchAngle += ry*pitchSpeed*deltaTime;
  pitchAngle = constrain(pitchAngle, 0, 270); 

  int pitchTop, pitchBottom;

  if (pitchAngle <= 90) {
    //Step 1: Top only moves
    pitchBottom = 90;
    pitchTop = map(pitchAngle, 0, 90, 180, 90);
    
  } else if (pitchAngle <= 180) {
    //Step 2: Both move together
    pitchBottom = map(pitchAngle, 90, 180, 90, 180);
    pitchTop = map(pitchAngle, 90, 180, 90, 180);

  } else {
    //Step 3: Bottom stays at max, top decreases
    pitchBottom = 180;
    pitchTop = map(pitchAngle, 180, 270, 180, 10);
  }

  // Safety clamps just in case
  pitchBottom = constrain(pitchBottom, 90, 180);
  pitchTop    = constrain(pitchTop, 10, 180);

  setServoPosition(&pwm3, NECKTOP, pitchTop);
  setServoPosition(&pwm4, NECKBOTTOM, pitchBottom);
}

// void notify() {
//   char messageString[200];
//   sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
//   ps5.LStickX(),
//   ps5.LStickY(),
//   ps5.RStickX(),
//   ps5.RStickY(),
//   ps5.Left(),
//   ps5.Down(),
//   ps5.Right(),
//   ps5.Up(),
//   ps5.Square(),
//   ps5.Cross(),
//   ps5.Circle(),
//   ps5.Triangle(),
//   ps5.L1(),
//   ps5.R1(),
//   ps5.L2(),
//   ps5.R2(),  
//   ps5.Share(),
//   ps5.Options(),
//   ps5.PSButton(),
//   ps5.Touchpad(),
//   ps5.Charging(),
//   ps5.Audio(),
//   ps5.Mic(),
//   ps5.Battery());

//   //Only needed to print the message properly on serial monitor. Else we dont need it.
//   if (millis() - lastLogTimeStamp > 50)
//   {
//     log(messageString);
//     lastLogTimeStamp = millis();
//   }
// }

void onConnect() {
  log("Connected!.");
}

void onDisconnect() {
  log("Disconnected!.");    
}

void log(String text) {
  Serial.println(text);
  Debug.println(text);
}

void setup () {
  Serial.begin(115200);
  //Wifi setup
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    log(".");
  }
  log("WiFi connected");

  //OTA setup
  ArduinoOTA.setHostname("wall-e"); 
  ArduinoOTA.setPassword(nullptr); //default password is OTAPASSWORD
  ArduinoOTA.begin();

  //RemoteDebug setup
  Debug.begin("wall-e");  
  Debug.setResetCmdEnabled(true);
  Debug.showProfiler(true);
  Debug.showColors(true);

  log("Setup complete, OTA + RemoteDebug ready");

  //PCA setup
  pca9685.begin();
  pca9685.setPWMFreq(50);

  //ps5 setup
  // ps5.attach(notify);
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisconnect);
  ps5.begin("4C:B9:9B:AD:03:BF");
  while(ps5.isConnected() == false) {
    log("Dualshock 5 not found");
    delay(3000);
  }
  log("Dualshock 5 ready");

  lastControllerUpdate = millis();
}

void loop() {
  ArduinoOTA.handle();
  Debug.handle();

  if(!ps5.isConnected()) return;

  if(ps5.Triangle()){
    runEyesTest();
  }

  if(ps5.Square()){
    log(String("Wifi ssid: ") + WIFI_SSID);
    log(String("Wifi pass: ") + WIFI_PASS);
  }

  rightStickController();
  
  // runEyesTest();
  // delay(1000);
  // runHeadTest();
  // delay(1000);
  // runNeckBottomTest();
  // delay(1000);
  // runNeckTopTest();
  // delay(1000);
  // runArmsTest();
  // delay(1000);
}







