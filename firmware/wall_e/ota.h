#pragma once
#include "../config/build_flags.h"

#if USE_OTA
  void otaSetup(const char* hostname = "wall-e");
  void otaLoop();

#else
  inline void otaSetup(const char* = nullptr) {}
  inline void otaLoop() {}
#endif