# **NTP**
The **NTP** library allows you to receive time information from the Internet. It also have support for
different timezones and daylight saving time (DST).
This NTP library uses the functions of the time.h standard library.<br>

## Changes for 1.7

- support for AVR
- optimizations 

## Changes for 1.6

- change of begin(), now you can start with hostname or IP address 
- no blocking
- better documentation

## Example
Example for WIFI boards like ESP32 or MKR1000, NANO RP2040 Connect and other, prints formatted time and date strings to console.

```cpp
// change next line to use with another board/shield
//#include <ESP8266WiFi.h>
//#include <WiFi.h> // for WiFi shield or ESP32
//#include <WiFi101.h> // for WiFi 101 shield or MKR1000
#include <WiFiNINA.h> // for UNO Wifi Rev 2 or Nano RP2040 connect
//#include "WiFiUdp.h" // not needed for WiFiNINA
#include "NTP.h"

const char *ssid     = "yourSSID"; // your network SSID
const char *password = "yourPASSWORD"; // your network PW

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
```

# Documentation

## Class Definitions

```cpp
NTP(UDP& udp);
```
Constructor for a NTP object

Example, this should done before ```setup()```
```cpp
WiFiUDP wifiUdp;
NTP ntp(wifiUdp);
``` 

```cpp
~NTP();
```
Destructor for a NTP object


## begin()

```cpp
void begin(const char* server = "pool.ntp.org");
void begin(IPAddress serverIP);
```
Start the underlaying UDP client with a hostname or IP address.

Example, this must done in ```setup()```
```cpp
ntp.begin(); // start the NTP client with the standard host
``` 

## stop()

```cpp
void stop();
```
Stop the underlaying UDP client

Example, this must done in ```setup()```
```cpp
ntp.stop();
```


## update()

```cpp
void update();
```
This must called in the main ```loop()```

Example
```cpp
loop() {
  ntp.update();
  }
``` 

## updateInterval()

```cpp
void updateInterval(uint32_t interval);
```
Set the update interval for connecting the NTP server in ms, default is 60000ms (60s)

Example, this must done in ```setup()```
```cpp
ntp.updateInterval(1000); // update every second
```

## ruleDST()

```cpp
void ruleDST(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);
```
Set the rules for the daylight save time settings

- tzname is the name of the timezone, e.g. "CEST" (central europe summer time)
- week Last, First, Second, Third, Fourth (0 - 4)
- wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
- month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
- hour the local hour when rule changes
- tzOffset timezone offset in minutes

Example, this must done in ```setup()```
```cpp
ntp.ruleDST("CEST", Last, Sun, Mar, 2, 120); // last sunday in march 2:00, timezone +120min (+1 GMT + 1h summertime offset)
```

## Return ruleDST()

```cpp
const char* ruleDST();
```
Returns  the DST time back, formatted as an ```ctime``` string

## ruleSTD()

```cpp
void ruleSTD(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);
```

Set the rules for the standard time settings

- tzname is the name of the timezone, e.g. "CET" (central europe time)
- week Last, First, Second, Third, Fourth (0 - 4)
- wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
- month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
- hour the local hour when rule changes
- tzOffset timezone offset in minutes

Example, this must done in ```setup()```
```cpp
ntp.ruleSTD("CET", Last, Sun, Oct, 3, 60); // last sunday in october 3:00, timezone +60min (+1 GMT)
```

## Return ruleSTD()

```cpp
const char* ruleSTD();
```
Return the STD time back, formatted as an ctime string

## Return tzName()

```cpp
const char* tzName();
```
Return you the name of the current timezone, based on your rule settings

## timeZone()

```cpp
void timeZone(int8_t tzHours, int8_t tzMinutes = 0);
```

Only use this function when you don't made the rules setting,
you have to the set isDST(false)

## isDST()

```cpp
void isDST(bool dstZone);
```

Use in conjunction with timeZone, when there is no DST!

## Return isDST()

```cpp
bool isDST();
```

Return the DST status back, true if summertime

## epoch()

```cpp
time_t epoch();
```

Return the Unix epoch timestamp

## year(), month(), day(), weekDay(), hours(), minutes(), seconds()

```cpp
int16_t year();
int8_t month();
int8_t day();
int8_t weekDay();
int8_t hours();
int8_t minutes();
int8_t seconds();
```

Return the datas from the tm structure of the "time.h" library

## Return formattedTime()

```cpp
const char* formattedTime(const char *format);
```

Return a string, formated with strftime function of standard time library

Example
```cpp
loop() {
  ntp.update();
  Serial.println(ntp.formattedTime("%d. %B %Y")); // dd. Mmm yyyy
  Serial.println(ntp.formattedTime("%A %T")); // Www hh:mm:ss
  delay(1000);
  }
``` 

Format symbols:
```
| symbol | explanation
/* General */
| % | writes literal %. The full conversion specification must be %%.
| n | writes newline character
| t | writes horizontal tab character
/* Year */
| Y | writes year as a decimal number, e.g. 2017
| y | writes last 2 digits of year as a decimal number (range [00,99])
| C | writes first 2 digits of year as a decimal number (range [00,99])
| G | writes ISO 8601 week-based year, i.e. the year that contains the specified week. 
	  In IS0 8601 weeks begin with Monday and the first week of the year must satisfy the following requirements:
	  - Includes January 4 
	  - Includes first Thursday of the year
| g | writes last 2 digits of ISO 8601 week-based year, i.e. the year that contains the specified week (range [00,99]).
	  In IS0 8601 weeks begin with Monday and the first week of the year must satisfy the following requirements:
	  - Includes January 4
	  - Includes first Thursday of the year
/* Month */
| b | writes abbreviated month name, e.g. Oct (locale dependent)
| h | synonym of b
| B | writes full month name, e.g. October (locale dependent)
| m | writes month as a decimal number (range [01,12])
/* Week */
| U | writes week of the year as a decimal number (Sunday is the first day of the week) (range [00,53])
| W | writes week of the year as a decimal number (Monday is the first day of the week) (range [00,53])
| V | writes ISO 8601 week of the year (range [01,53]).
	  In IS0 8601 weeks begin with Monday and the first week of the year must satisfy the following requirements:
	  - Includes January 4
	  - Includes first Thursday of the year
/* Day of the year/month */
| j | writes day of the year as a decimal number (range [001,366])
| d | writes day of the month as a decimal number (range [01,31])
| e | writes day of the month as a decimal number (range [1,31]).
	  Single digit is preceded by a space.
/* Day of the week */
| a | writes abbreviated weekday name, e.g. Fri (locale dependent)
| A | writes full weekday name, e.g. Friday (locale dependent)
| w | writes weekday as a decimal number, where Sunday is 0 (range [0-6])
| u | writes weekday as a decimal number, where Monday is 1 (ISO 8601 format) (range [1-7])
/* Hour, minute, second */
| H | writes hour as a decimal number, 24 hour clock (range [00-23])
| I | writes hour as a decimal number, 12 hour clock (range [01,12])
| M | writes minute as a decimal number (range [00,59])
| S | writes second as a decimal number (range [00,60])
/* Other */
| c | writes standard date and time string, e.g. Sun Oct 17 04:41:13 2010 (locale dependent)	
| x | writes localized date representation (locale dependent)
| X | writes localized time representation (locale dependent)
| D | equivalent to "%m/%d/%y"
| F | equivalent to "%Y-%m-%d" (the ISO 8601 date format)
| r | writes localized 12-hour clock time (locale dependent)
| R | equivalent to "%H:%M"
| T | equivalent to "%H:%M:%S" (the ISO 8601 time format)
| p | writes localized a.m. or p.m. (locale dependent)
```

## Return utc()

 ```cpp
 uint32_t utc();
 ```

 Return the timestamp received from the ntp server in Unix timestamp format

 ## Return ntp()

 ```cpp
 uint32_t ntp();
 ```

 Return the timestamp received from the ntp server in the NTP timestamp format



