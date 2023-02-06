#include "FreqCountESP.h"

volatile uint8_t _FreqCountESP::sIsFrequencyReady = false;
volatile uint32_t _FreqCountESP::sCount = 0;
volatile uint32_t _FreqCountESP::sFrequency = 0;

#ifdef USE_PCNT  // Use ESP32 hardware pulse counter instead of per-pulse ISR.
// Thanks to jgustavoam and Rui Viana for tips gleaned from
// https://www.esp32.com/viewtopic.php?t=17018

volatile uint32_t _FreqCountESP::sLastPcnt = 0;

#define PCNT_HIGH_LIMIT 32767  // largest +ve value for int16_t.
#define PCNT_LOW_LIMIT  0

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_CHANNEL PCNT_CHANNEL_0

portMUX_TYPE pcntMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR onHLim(void *backupCounter)
{
  // 16 bit pulse counter hit high limit; increment the 32 bit backup.
  portENTER_CRITICAL_ISR(&pcntMux);
  *(volatile uint32_t *)backupCounter += PCNT_HIGH_LIMIT;
  PCNT.int_clr.val = BIT(PCNT_UNIT);  // Clear the interrupt.
  portEXIT_CRITICAL_ISR(&pcntMux);
}

static pcnt_isr_handle_t setupPcnt(uint8_t pin, volatile uint32_t *backupCounter) { 
  pcnt_config_t pcntConfig = {
    .pulse_gpio_num = pin,
    .ctrl_gpio_num = -1,
    .pos_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
    .neg_mode = PCNT_CHANNEL_EDGE_ACTION_HOLD,
    .counter_h_lim = PCNT_HIGH_LIMIT,
    .counter_l_lim = PCNT_LOW_LIMIT,
    .unit = PCNT_UNIT,
    .channel = PCNT_CHANNEL,
  };
  pcnt_unit_config(&pcntConfig);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);  // Interrupt on high limit.
  pcnt_isr_handle_t isrHandle;
  pcnt_isr_register(onHLim, (void *)backupCounter, 0, &isrHandle);
  pcnt_intr_enable(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
  return isrHandle;
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&_FreqCountESP::sMux);
  int16_t pulseCount;
  uint32_t pcntTotal = _FreqCountESP::sCount;
  pcnt_get_counter_value(PCNT_UNIT, &pulseCount);
  if (pulseCount < 1000) {
    // Maybe counter just rolled over? Re-read 32 bit basis.
    pcntTotal = _FreqCountESP::sCount;
  }
  pcntTotal += pulseCount;
  _FreqCountESP::sFrequency = (uint32_t)(pcntTotal - _FreqCountESP::sLastPcnt);
  _FreqCountESP::sLastPcnt = pcntTotal;
  _FreqCountESP::sIsFrequencyReady = true;
  portEXIT_CRITICAL_ISR(&_FreqCountESP::sMux);
}

void teardownPcnt(pcnt_isr_handle_t isrHandle)
{
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_intr_disable(PCNT_UNIT);
  pcnt_isr_unregister(isrHandle);
}

#else // !USE_PCNT

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&_FreqCountESP::sMux);
  _FreqCountESP::sFrequency = _FreqCountESP::sCount;
  _FreqCountESP::sCount = 0;
  _FreqCountESP::sIsFrequencyReady = true;
  portEXIT_CRITICAL_ISR(&_FreqCountESP::sMux);
}
#endif // !USE_PCNT

portMUX_TYPE _FreqCountESP::sMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onRise()
{
  portENTER_CRITICAL_ISR(&_FreqCountESP::sMux);
  _FreqCountESP::sCount++;
  portEXIT_CRITICAL_ISR(&_FreqCountESP::sMux);
}

_FreqCountESP::_FreqCountESP()
{
  mTimer = NULL;
}

_FreqCountESP::~_FreqCountESP()
{
  end();
}

void _FreqCountESP::_begin(uint8_t freqPin, uint8_t freqPinIOMode)
{
  // Configure counting on frequency input pin.
  mPin = freqPin;
  sIsFrequencyReady = false;
  sCount = 0;
  sFrequency = 0;

  pinMode(mPin, freqPinIOMode);

#ifdef USE_PCNT
  _FreqCountESP::sLastPcnt = 0;
  mIsrHandle = setupPcnt(mPin, &_FreqCountESP::sCount);
#else  // !USE_PCNT
  attachInterrupt(mPin, &onRise, RISING);
#endif  // USE_PCNT
  /*if(mTriggerPin == 0) {
    // Not external trigger, start internal timer.
    timerAlarmEnable(mTimer);
  }*/
}

void _FreqCountESP::begin(uint8_t freqPin, uint16_t timerMs, uint8_t hwTimerId, uint8_t freqPinIOMode)
{
  // Count frequency using internal timer.
  // mTriggerPin == 0 means we're using internal timer.
  mTriggerPin = 0;
  mTimer = timerBegin(hwTimerId, 80, true);
  timerAttachInterrupt(mTimer, &onTimer, true);
  timerAlarmWrite(mTimer, timerMs * 1000, false);

  _begin(freqPin, freqPinIOMode);
}

void _FreqCountESP::beginExtTrig(uint8_t freqPin, uint8_t extTriggerPin, uint8_t freqPinIOMode, uint8_t extTriggerMode)
{
  // Count frequency between events from an external trigger input.
  // mTriggerPin == 0 means we're using internal timer.
  assert(extTriggerPin > 0);
  mTriggerPin = extTriggerPin;
  pinMode(mTriggerPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(mTriggerPin), &onTimer, extTriggerMode);

  _begin(freqPin, freqPinIOMode);
}

void _FreqCountESP::runMeasure()
{
  timerAlarmEnable(mTimer);
}

uint32_t _FreqCountESP::read()
{
  sIsFrequencyReady = false;
  return sFrequency;
}

uint8_t _FreqCountESP::available()
{
  return sIsFrequencyReady;
}

void _FreqCountESP::end()
{
#ifdef USE_PCNT
  teardownPcnt(mIsrHandle);
#else 
  detachInterrupt(mPin);
#endif
  if(mTriggerPin == 0) {
    timerAlarmDisable(mTimer);
    timerDetachInterrupt(mTimer);
    timerEnd(mTimer);
  } else {
    detachInterrupt(digitalPinToInterrupt(mTriggerPin));
  }
}

_FreqCountESP FreqCountESP;
