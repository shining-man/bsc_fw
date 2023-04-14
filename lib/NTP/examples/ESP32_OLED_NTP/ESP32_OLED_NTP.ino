// example ESP32 board with oled, use oled lib https://github.com/squix78/esp8266-oled-ssd1306

#include <Arduino.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include "SSD1306.h"
#include "NTP.h"

char ssid[] = "yourSSID";
char password[] = "yourPASSWORD";

SSD1306 display(0x3c, 5, 4);
WiFiUDP wifiUdp;
NTP ntp(wifiUdp);

void display_text(String text){
  display.clear();
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 15, text);
  display.display();
  }

void setup() {
  Serial.begin(9600);
  display.init();
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    display_text("Connecting ...");
    delay (500);
    }
  display.clear();
  display_text("Connected");
  delay (500);
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  // ntp.isDST(false);
  // ntp.timeZone(1);
  // ntp.offset(0, 0, 0, 0);
  ntp.begin();
  display_text("start NTP");
  delay (500);
  }

void loop() {
  ntp.update();
  display.clear();
  display.fillRect(1, 0, 126 * ntp.seconds() / 59, 2);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 5, ntp.formattedTime("%d. %B %Y"));
  display.drawString(64, 15, ntp.formattedTime("%A %T"));
  display.drawString(64, 25, ntp.ruleDST());
  display.drawString(64, 35, ntp.ruleSTD());
  display.drawString(64, 45, ntp.tzName());
  display.display();
  delay(500);
  }
