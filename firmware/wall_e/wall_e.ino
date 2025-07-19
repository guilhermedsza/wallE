#include <ESP32Servo.h>

Servo leftEye;
Servo rightEye;
Servo head;
Servo neckTop;
Servo neckBottom;
Servo rightArm;
Servo leftArm;

void runArmsTest() {
  int rightArmHigh = 110;
  int rightArmLow = 40;

  int leftArmLow = 180 - rightArmLow;
  int leftArmHigh = 180 - rightArmHigh;

  leftArm.write(leftArmHigh);
  rightArm.write(rightArmHigh);
  delay(500);

  leftArm.write(leftArmLow);
  rightArm.write(rightArmLow);
  delay(500);
  
  leftArm.write(leftArmHigh);
  rightArm.write(rightArmHigh);
  delay(500);

  leftArm.write(leftArmLow);
  rightArm.write(rightArmLow);
  delay(500);
}

void runNeckBottomTest() {
  int neckBottomHigh = 180;
  int neckBottomLow = 90;
  // 50

  neckBottom.write(neckBottomHigh);
  delay(500);

  neckBottom.write(neckBottomLow);
  delay(500);
}

void runNeckTopTest() {
  int neckTopHigh = 180;
  int neckTopLow = 10;

  neckTop.write(neckTopHigh);
  delay(500);

  neckTop.write(neckTopLow);
  delay(500);

  neckTop.write(neckTopHigh);
  delay(500);

  neckTop.write(neckTopLow);
  delay(500);
}

void runHeadTest() {
// head -> 100 degrees he looks straight
  int headHigh = 180;
  int headLow = 10;

  head.write(headHigh);
  delay(500);
  
  head.write(headLow);
  delay(500);

  head.write(headHigh);
  delay(500);

  head.write(headLow);
  delay(500);

  head.write(headHigh);
  delay(500);

  head.write(headLow);
  delay(500);

  head.write(95);
  delay(500);
}

void runEyesTest() {
  // left eye: 140, 100
  int rightEyeHigh = 40;
  int rightEyeLow = 80;

  int leftEyeHigh = 180-rightEyeHigh;
  int leftEyeLow = 180-rightEyeLow;

  leftEye.write(leftEyeLow);  
  rightEye.write(rightEyeLow);
  delay(500);

  leftEye.write(leftEyeHigh);  
  rightEye.write(rightEyeHigh);  
  delay(500);

  leftEye.write(leftEyeLow);  
  rightEye.write(rightEyeHigh);
  delay(500);

  leftEye.write(leftEyeHigh);  
  rightEye.write(rightEyeLow);  
  delay(500);

   leftEye.write(leftEyeLow);  
  rightEye.write(rightEyeHigh);
  delay(500);

   leftEye.write(leftEyeHigh);  
  rightEye.write(rightEyeLow);   
  delay(500);

  leftEye.write(leftEyeHigh);  
  rightEye.write(rightEyeHigh);  
  delay(500);
}

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  ESP32PWM::allocateTimer(4);
  ESP32PWM::allocateTimer(5);
  ESP32PWM::allocateTimer(6);

  leftEye.setPeriodHertz(50);    // Standard 50 hz servo
  leftEye.attach(12, 500, 2500); // pin, min, max in microseconds

  rightEye.setPeriodHertz(50);   
  rightEye.attach(13, 500, 2500); 

  head.setPeriodHertz(50);   
  head.attach(12, 500, 2500);

  neckTop.setPeriodHertz(50);
  neckTop.attach(12, 500, 2500);

  neckBottom.setPeriodHertz(50);   
  neckBottom.attach(13, 500, 2500); 

  leftArm.setPeriodHertz(50);   
  leftArm.attach(12, 500, 2500);

  rightArm.setPeriodHertz(50);   
  rightArm.attach(13, 500, 2500);

  runEyesTest();
}

void loop() {
  // Nothing here — static position test
}