#ifndef kapraran_FreqCountESP_h
#define kapraran_FreqCountESP_h

#include <Arduino.h>

#define USE_PCNT  // Use ESP32 hardware pulse counter instead of per-pulse ISR.

#ifdef USE_PCNT
extern "C" {
  #include "soc/pcnt_struct.h"
}
#include <driver/pcnt.h>
#endif

void IRAM_ATTR onRise();
void IRAM_ATTR onTimer();

class _FreqCountESP
{
private:
  uint8_t mPin;
  uint8_t mTriggerPin;
  hw_timer_t *mTimer;
#ifdef USE_PCNT
  pcnt_isr_handle_t mIsrHandle;
#endif
  void _begin(uint8_t freqPin, uint8_t freqPinIOMode);

public:
  static volatile uint8_t sIsFrequencyReady;
  static volatile uint32_t sCount;
  static volatile uint32_t sFrequency;
#ifdef USE_PCNT
  static volatile uint32_t sLastPcnt;
#endif

  static portMUX_TYPE sMux;

  _FreqCountESP();
  ~_FreqCountESP();

  // Frequency counter using internal interval timer.
  void begin(uint8_t pin, uint16_t timerMs, uint8_t hwTimerId = 0, uint8_t freqPinIOMode = INPUT);
  // Frequency counter using external trigger pulse input.
    void beginExtTrig(uint8_t pin, uint8_t extTriggerPin, uint8_t freqPinIOMode = INPUT, uint8_t extTriggerMode = RISING);
  void runMeasure();
  uint32_t read();
  uint8_t available();
  void end();
};

extern _FreqCountESP FreqCountESP;

#endif // kapraran_FreqCountESP_h
