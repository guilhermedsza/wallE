#include "controller.h"
#include "logger.h" // for LOG()
#include <Arduino.h> // millis()
#include "diagnostics.h"

#if USE_PS5

//Static state
static unsigned long lastUpdate = 0;
static constexpr float DEADZONE = 0.10f;
static float yawAngle   = 95.0f,  yawSpeed   = 180.0f; //degrees/sec
static float pitchAngle = 90.0f,  pitchSpeed = 180.0f;

// Forward declaration of helpers (lives in servo.h/cpp)
#include "servo.h"
static void rxController(float dt);
static void ryController(float dt);

//Callbacks
static void onConnect()    { LOG("DualSense 5 Connected!");   }
static void onDisconnect() { LOG("DualSense 5 Disconnected!"); }

// Public API
void controllerSetup(const char* mac)
{
    ps5.attachOnConnect(onConnect);
    ps5.attachOnDisconnect(onDisconnect);
    ps5.begin(mac);

    LOG("Waiting for PS-5 controller…");
    while (!ps5.isConnected()) {
        delay(500);
        LOG(".");
    }
    LOG("Dualshock 5 ready");
    lastUpdate = millis();
}

void controllerLoop()
{
    if (!ps5.isConnected()) return;

    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f; // seconds
    lastUpdate = now;

    rxController(dt);
    ryController(dt);

// Eyes test
    static bool triPrev = false;
    bool triNow = ps5.Triangle();
    if (triNow && !triPrev) {  
        LOG("Running eyes test");
        runEyesTest();
    }
    triPrev = triNow;
}

// Helpers
static void rxController(float dt)
{
    float rx = ps5.RStickX() / 128.0f;
    if (fabs(rx) < DEADZONE) rx = 0;
    yawAngle  += rx * yawSpeed * dt;
    yawAngle   = constrain(yawAngle, 0, 180);
    setServoDeg(HEAD, yawAngle);
}

static void ryController(float dt)
{
    float ry = ps5.RStickY() / 128.0f;
    if (fabs(ry) < DEADZONE) ry = 0;
    pitchAngle += ry * pitchSpeed * dt;
    pitchAngle  = constrain(pitchAngle, 0, 270);

    uint16_t top, bottom;
    if (pitchAngle <= 90) { // phase 1
        bottom =  90;
        top    = map(pitchAngle, 0, 90, 180,  90);
    } else if (pitchAngle <= 180) { // phase 2
        bottom = map(pitchAngle,  90, 180,  90, 180);
        top    = map(pitchAngle,  90, 180,  90, 180);
    } else { // phase 3
        bottom = 180;
        top    = map(pitchAngle, 180, 270, 180,  10);
    }

    setServoDeg(NECKTOP,    top);
    setServoDeg(NECKBOTTOM, bottom);
}

#endif 