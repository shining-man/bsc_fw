#include "LEDController.h"
#include <Arduino.h>
#include "defines.h"
#include <FastLED.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


namespace LEDController {

  static CRGB leds[NUM_LEDS];

  static uint8_t m_stateStatusLed;
  static uint8_t m_toggleLedState;

  static SemaphoreHandle_t ledMutex;

  void begin() {
    FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
    FastLED.clear(true);
    FastLED.setBrightness(20);
  
    m_toggleLedState = 0;

    // Staus LED
    m_stateStatusLed = 2;
    setBit(m_toggleLedState, 0);

    // Mutex erstellen
    if (ledMutex == nullptr) {
      ledMutex = xSemaphoreCreateMutex();
    }
  }

  void setStateStatusLED(int status) {
    m_stateStatusLed = status;
  }

  void toggleStatusLED() {
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
      if(isBitSet(m_toggleLedState, 0) == false) {
        setBit(m_toggleLedState, 0);
        fadeToBlackBy(&leds[0], 1, 200);
      }
      else {
        clearBit(m_toggleLedState, 0);
        if(m_stateStatusLed == 0) leds[0] = CRGB::Red;
        else if(m_stateStatusLed == 1) leds[0] = CRGB::Green;
        else if(m_stateStatusLed == 2) leds[0] = CRGB::Blue;
        else leds[0] = CRGB::Red;
      }

      FastLED.show();
      xSemaphoreGive(ledMutex);
    }
  }


  void setSerialLED(bool state) {
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
      if(state) leds[2] = CRGB::Red;
      else leds[2] = CRGB::Black;

      FastLED.show();
      xSemaphoreGive(ledMutex);
    }
  }


  void setCanbusLED(bool state) {
    if (xSemaphoreTake(ledMutex, portMAX_DELAY) == pdTRUE) {
      if(state) leds[3] = CRGB::Red;
      else leds[3] = CRGB::Black;

      FastLED.show();
      xSemaphoreGive(ledMutex);
    }
  }

}
