#pragma once
#include "../config/build_flags.h"

#if USE_PS5
  #include <ps5Controller.h>
  void controllerSetup(const char* mac); // call from setup()
  void controllerLoop(); // call each loop()
#else
  inline void controllerSetup(const char* mac) {}
  inline void controllerLoop()  {}
#endif