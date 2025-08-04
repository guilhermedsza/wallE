#include "ota.h"
#if USE_OTA // compile the body only when flag is 1

#include <ArduinoOTA.h>
#include "logger.h"

void otaSetup(const char* hostname)
{
    ArduinoOTA.setHostname(hostname);
    ArduinoOTA.setPassword(nullptr); // default "OTAPASSWORD"
    ArduinoOTA.onStart([]() { LOG("OTA: Start"); });
    ArduinoOTA.onEnd  ([]() { LOG("OTA: End");   });
    ArduinoOTA.onError([](ota_error_t err) {
        LOGf("OTA Error %d", err);
    });
    ArduinoOTA.begin();
    LOG("OTA ready");
}

void otaLoop()
{
    ArduinoOTA.handle();
}

#endif   // USE_OTA