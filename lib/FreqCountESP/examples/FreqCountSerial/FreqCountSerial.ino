#include "FreqCountESP.h"

int inputPin = 14;
int timerMs = 1000;

void setup()
{
  Serial.begin(9600);
  FreqCountESP.begin(inputPin, timerMs);
}

void loop()
{
  if (FreqCountESP.available())
  {
    uint32_t frequency = FreqCountESP.read();
    Serial.println(frequency);
  }
}
