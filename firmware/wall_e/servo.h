#pragma once
#include <stdint.h>
#include <Adafruit_PWMServoDriver.h>

enum ServoChannel : uint8_t {
  LEFTEYE    = 0,
  RIGHTEYE   = 1,
  HEAD       = 2,
  NECKTOP    = 3,
  NECKBOTTOM = 4,
  LEFTARM    = 5,
  RIGHTARM   = 6
};

// Call once from setup() after pca9685.begin()
void servoSetup(Adafruit_PWMServoDriver& driver);

void setServoDeg(ServoChannel ch, uint16_t degrees);

void servoWave();
