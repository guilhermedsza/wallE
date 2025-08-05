#include "websocket.h"
#include <AsyncTCP.h>              // has to come before ESPAsyncWebServer
#include <ESPAsyncWebServer.h>
#include "servo.h"
#include "logger.h"
#include "diagnostics.h"

AsyncWebServer server(80); //HTTP and Websocket will share the same port (80)
AsyncWebSocket ws("/ws"); //Clients should connect to ws://<ip>/ws

static void onWsEvent (AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type != WS_EVT_DATA) return;

  AwsFrameInfo *info = reinterpret_cast<AwsFrameInfo *>(arg);
  if(info->opcode != WS_TEXT) return; //this line ignores binary frames

  String msg(reinterpret_cast<char *>(data), len);
  LOGf("WS -> %s", msg.c_str());

  if(msg.c_str() == "WAVE") {
    LOG("IS WAVING");
    servoWave();
    websocketSend("DONE");
  }

  if(msg.c_str() == "EYES") {
    LOG("WILL RUN EYES TEST");
    runEyesTest();
    websocketSend("DONE");
  }
}

void websocketSetup()
{
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.onNotFound([](AsyncWebServerRequest *req) { req->send(404, "text/plain", "Not here"); }); // 404 for normal HTTP requests so the port isn’t blank

  server.begin();
  LOG("WebSocket server started at /ws");
}

void websocketSend(const String &msg)
{
  ws.textAll(msg); //this message will get broadcasted to every connected client
}