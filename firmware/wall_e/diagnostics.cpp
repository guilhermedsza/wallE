#include "diagnostics.h"

#if ENABLE_DIAGS

#include <Arduino.h>
#include "servo.h"

static void wait() { delay(500); }

void runEyesTest()
{
  const uint16_t rightHigh = 40, rightLow = 80;
  const uint16_t leftHigh = 180 - rightHigh;
  const uint16_t leftLow = 180 - rightLow;

  setServoDeg(LEFTEYE , leftLow);  setServoDeg(RIGHTEYE, rightLow);   wait();
  setServoDeg(LEFTEYE , leftHigh); setServoDeg(RIGHTEYE, rightHigh);  wait();
  setServoDeg(LEFTEYE , leftLow);  setServoDeg(RIGHTEYE, rightHigh);  wait();
  setServoDeg(LEFTEYE , leftHigh); setServoDeg(RIGHTEYE, rightHigh);  wait();
  setServoDeg(LEFTEYE , leftLow);  setServoDeg(RIGHTEYE, rightHigh);  wait();
  setServoDeg(LEFTEYE , leftHigh); setServoDeg(RIGHTEYE, rightLow);   wait();
  setServoDeg(RIGHTEYE, rightHigh);                                   wait();
}

void runHeadTest()
{
  const uint16_t headHigh = 180, headLow = 10;
  setServoDeg(HEAD, headHigh); wait();
  setServoDeg(HEAD, headLow); wait();
  setServoDeg(HEAD, headHigh); wait();
  setServoDeg(HEAD, headLow); wait();
  setServoDeg(HEAD, headHigh); wait();
  setServoDeg(HEAD, headLow); wait();
  setServoDeg(HEAD, 95); wait();
}

void runNeckTopTest()
{
  const uint16_t neckTopHigh = 180, neckTopLow = 10;

  setServoDeg(NECKTOP, neckTopHigh); wait();
  setServoDeg(NECKTOP, neckTopLow); wait();
  setServoDeg(NECKTOP, neckTopHigh); wait();
  setServoDeg(NECKTOP, neckTopLow); wait();
}

void runNeckBottomTest()
{
  const uint16_t neckBottomHigh = 180, neckBottomLow = 90;

  setServoDeg(NECKBOTTOM, neckBottomHigh); wait();
  setServoDeg(NECKBOTTOM, neckBottomLow); wait();
  setServoDeg(NECKBOTTOM, neckBottomHigh); wait();
  setServoDeg(NECKBOTTOM, neckBottomLow); wait();
}

void runArmsTest()
{
  const uint16_t rightArmHigh = 110, rightArmLow = 40;
  const uint16_t leftArmHigh = 180 - rightArmHigh;
  const uint16_t leftArmLow = 180 - rightArmLow;

  setServoDeg(LEFTARM , leftArmHigh); setServoDeg(RIGHTARM, rightArmHigh); wait();
  setServoDeg(LEFTARM , leftArmLow); setServoDeg(RIGHTARM, rightArmLow); wait();
  setServoDeg(LEFTARM , leftArmHigh); setServoDeg(RIGHTARM, rightArmHigh); wait();
  setServoDeg(LEFTARM , leftArmLow); setServoDeg(RIGHTARM, rightArmLow); wait();
}


#endif  // ENABLE_DIAGS