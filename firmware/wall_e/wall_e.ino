#include <ESP32Servo.h>
#include <Wire.h> // Wire is for I2C
#include <Adafruit_PWMServoDriver.h>

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

void setup () {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Test");
  pca9685.begin();
  pca9685.setPWMFreq(50);
}

void loop() {
  runEyesTest();
  delay(1000);
  runHeadTest();
  delay(1000);
  runNeckBottomTest();
  delay(1000);
  runNeckTopTest();
  delay(1000);
  // runArmsTest();
  // delay(1000);
}







