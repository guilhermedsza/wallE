
#pragma once
#include <ESPAsyncWebServer.h>

extern AsyncWebServer server; //Exposes global server so .ino will be able to keep ownership
extern AsyncWebSocket    ws;

// call this from setup()
void websocketSetup();

void websocketSend(const String &msg = "DONE");