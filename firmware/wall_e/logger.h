#pragma once
#include "../config/build_flags.h"
#include <Arduino.h>

#if USE_REMOTE_DEBUG
  #include <RemoteDebug.h>
  extern RemoteDebug Debug;
#endif

void loggerSetup(); // call once from setup()

// Simple message (String, const char*, etc.)
void LOG(const String& msg);

/* printf-style helper */
void LOGf(const char* fmt, ...);

#if USE_REMOTE_DEBUG
  void loggerLoop();
#else
  inline void loggerLoop() {}
#endif