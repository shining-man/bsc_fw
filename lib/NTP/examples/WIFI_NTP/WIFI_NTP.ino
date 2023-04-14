// example for WIFI based boards like ESP8266, ESP32, Nano RP2040 Connect, WiFi 101 shield or MKR1000

#include "Arduino.h"
// change next line to use with another board/shield
//#include <ESP8266WiFi.h>
//#include <WiFi.h> // for WiFi shield or ESP32
//#include <WiFi101.h> // for WiFi 101 shield or MKR1000
#include <WiFiNINA.h> // for e.g. Nano RP2040 Connect
//#include "WiFiUdp.h" // not needed for WiFiNINA
#include "NTP.h"

char ssid[]     = "yourSSID";
char password[] = "yourPASSWORD";

WiFiUDP wifiUdp;
NTP ntp(wifiUdp);

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting ...");
    delay(500);
    }
  Serial.println("Connected");  
  ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timetone +120min (+1 GMT + 1h summertime offset)
  ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
  ntp.begin();
  Serial.println("start NTP");
  }

void loop() {
  ntp.update();
  Serial.println(ntp.formattedTime("%d. %B %Y")); // dd. Mmm yyyy
  Serial.println(ntp.formattedTime("%A %T")); // Www hh:mm:ss
  delay(1000);
  }
