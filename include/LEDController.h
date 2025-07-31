#pragma once

#include <FastLED.h>
#include <functional>

namespace LEDController {

    // Konfiguration
    constexpr int NUM_LEDS  = 4;
    constexpr int DATA_PIN  = 8;
    constexpr int CLOCK_PIN = 3;

    // Initialisierung
    void begin();

    void setStateStatusLED(int status);
    void toggleStatusLED();
    void setSerialLED(bool state);
    void setCanbusLED(bool state);
}
