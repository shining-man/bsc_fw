#include "FreqCountESP.h"
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27);

int inputPin = 14;
int timerMs = 1000;

void setup()
{
  FreqCountESP.begin(inputPin, timerMs);

  // setup lcd
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home();
}

void loop()
{
  if (FreqCountESP.available())
  {
    uint32_t frequency = FreqCountESP.read();
    lcd.clear();
    lcd.home();
    lcd.print("Freq:" + String(frequency));
  }
}
