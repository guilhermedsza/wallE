#include "servo.h"
#include <Arduino.h> // constrain(), map()
#include "../config/pins.h"
// #include <Wire.h> // Wire is for I2C

namespace {
  constexpr uint8_t  NUM_SERVOS = 7;

  Adafruit_PWMServoDriver* drv = nullptr;
  uint16_t pulse[NUM_SERVOS] = {0}; // last pulse we sent to each channel
}

void servoSetup(Adafruit_PWMServoDriver& driver)
{
  drv = &driver;
  drv->setPWMFreq(50); // 50 Hz standard servo rate
}

void setServoDeg(ServoChannel ch, uint16_t deg)
{
  if (!drv || ch >= NUM_SERVOS) return;       
  deg          = constrain(deg, 0u, 180u);
  pulse[ch]    = map(deg, 0, 180, SERVO_MIN, SERVO_MAX);
  drv->setPWM(ch, 0, pulse[ch]);
}

