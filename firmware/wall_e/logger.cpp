#include "logger.h"
#include <stdarg.h> // va_list

#if USE_REMOTE_DEBUG
  #include <RemoteDebug.h>
  RemoteDebug Debug;

  void loggerLoop() { Debug.handle(); }
#endif

void loggerSetup()
{
#if USE_SERIAL_LOG
  Serial.begin(115200);
#endif
#if USE_REMOTE_DEBUG
  Debug.begin("wall-e");
  Debug.setResetCmdEnabled(true);
  Debug.showProfiler(true);
  Debug.showColors(true);
#endif
}

/* plain String / const char* overload */
void LOG(const String& msg)
{
#if USE_SERIAL_LOG
  Serial.println(msg);
#endif
#if USE_REMOTE_DEBUG
  Debug.println(msg);
#endif
}

/* printf-style (usage: LOGf("Angle %d", deg);) */
void LOGf(const char* fmt, ...)
{
#if USE_SERIAL_LOG || USE_REMOTE_DEBUG
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  LOG(String(buf));
#endif
}