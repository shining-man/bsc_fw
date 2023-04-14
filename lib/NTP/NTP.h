/**
 * NTP library for Arduino framewoek
 * The MIT License (MIT)
 * (c) 2022 sstaub
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef NTP_H
#define NTP_H

#include "Arduino.h"
#include <time.h>
#include <Udp.h>

#define SEVENTYYEARS 2208988800UL
#define UNIXOFFSET 946684800UL

#ifdef __AVR__
  //#define POSIX_OFFSET NTP_OFFSET - SEVENTYYEARS// - UNIX_OFFSET// + 30 years
  #define POSIX_OFFSET UNIXOFFSET
#else
  #define POSIX_OFFSET -SEVENTYYEARS
#endif

#define NTP_PACKET_SIZE 48
#define NTP_PORT 123
#define SECS_PER_MINUTES 60
#define SECS_PER_DAY 86400

#define GMT_MESSAGE "GMT +/- offset"
#define RULE_DST_MESSAGE "no DST rule"
#define RULE_STD_MESSAGE "no STD rule"

enum week_t {Last, First, Second, Third, Fourth}; 
enum dow_t {Sun, Mon, Tue, Wed, Thu, Fri, Sat};
enum month_t {Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};

class NTP {
  public:
    /**
     * @brief constructor for the NTP object
     * 
     * @param udp 
     */
    NTP(UDP& udp);

    /**
     * @brief destructor for the NTP object
     * 
     */
    ~NTP();

    /**
     * @brief starts the underlying UDP client with the default local port
     * 
     * @param server NTP server host name
     * @param serverIP NTP server IP address
     */
    void begin(const char* server = "pool.ntp.org");
    void begin(IPAddress serverIP);

    /**
     * @brief stops the underlying UDP client
     * 
     */
    void stop();

    /**
     * @brief This should be called in the main loop of your application. By default an update from the NTP Server is only
     * made every 60 seconds. This can be configured in the NTPTime constructor.
     * 
     * @return true on success
     * @return false on no update or update failure
     */
    int8_t update();

    /**
     * @brief set the update interval
     * 
     * @param updateInterval in ms, default = 60000ms
     */
    void updateInterval(uint32_t interval);

    /**
     * @brief set the rule for DST (daylight saving time)
     * start date of DST 
     * 
     * @param tzName name of the time zone
     * @param week Last, First, Second, Third, Fourth (0 - 4)
     * @param wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
     * @param month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
     * @param hour the local hour when rule chages
     * @param tzOffset sum of summertime and timezone offset
     */
    void ruleDST(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);

    /**
     * @brief get the DST time as a ctime string
     * 
     * @return char* time string
     */
    const char* ruleDST();

    /**
     * @brief set the rule for STD (standard day)
     * end date of DST
     * 
     * @param tzName name of the time zone
     * @param week Last, First, Second, Third, Fourth (0 - 4)
     * @param wday Sun, Mon, Tue, Wed, Thu, Fri, Sat (0 - 7)
     * @param month Jan, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec (0 -11)
     * @param hour the local hour when rule chages
     * @param tzOffset timezone offset
     */
    void ruleSTD(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset);

    /**
     * @brief get the STD time as a ctime string
     * 
     * @return char* time string
     */
    const char* ruleSTD();

    /**
     * @brief get the name of the timezone
     * 
     * @return char* name of the timezone
     */
    const char* tzName();

   /**
     * @brief set the timezone manually 
     * this should used if there is no DST!
     * 
     * @param tzHours 
     * @param tzMinutes 
     */
    void timeZone(int8_t tzHours, int8_t tzMinutes = 0);

    /**
     * @brief set daylight saving manually! 
     * use in conjunction with timeZone, when there is no DST!
     * 
     * @param dstZone 
     */
    void isDST(bool dstZone);

    /**
     * @brief returns the DST state
     * 
     * @return int 1 if summertime, 0 no summertime
     */
    bool isDST();
	
    /**
     * @brief get the Unix epoch timestamp
     * 
     * @return time_t timestamp
     */
    time_t epoch();

    /**
     * @brief get the year
     * 
     * @return int year
     */
    int16_t year();

    /**
     * @brief get the month
     * 
     * @return int month, 1 = january 
     */
    int8_t month();

    /**
     * @brief get the day of a month
     * 
     * @return int day
     */
    int8_t day();

    /**
     * @brief get the day of a week
     * 
     * @return int day of the week, 0 = sunday
     */
    int8_t weekDay();

    /**
     * @brief get the hour of the day
     * 
     * @return int 
     */
    int8_t hours();

    /**
     * @brief get the minutes of the hour
     * 
     * @return int minutes
     */
    int8_t minutes();

    /**
     * @brief get the seconds of a minute
     * 
     * @return int seconds
     */
    int8_t seconds();    

    /**
     * @brief returns a formatted string
     * 
     * @param format for strftime
     * @return char* formated time string
     */
    const char* formattedTime(const char *format);

    /**
     * @brief returns NTP timestamp
     * 
     * @return uint32_t 
     */
    uint32_t ntp();

    /**
     * @brief returns UNIX timestamp
     * 
     * @return uint32_t 
     */
    uint32_t utc();

  private:
    UDP *udp;
    const char* server = nullptr;
    IPAddress serverIP;
    uint8_t ntpRequest[NTP_PACKET_SIZE]; // = {0xE3, 0x00, 0x06, 0xEC};
    uint8_t ntpQuery[NTP_PACKET_SIZE];
    time_t utcCurrent = 0;
    time_t local = 0;
    struct tm *current;
    uint32_t interval = 60000;
    uint32_t lastUpdate = 0;
    uint8_t tzHours = 0;
    uint8_t tzMinutes = 0;
    int32_t timezoneOffset;
    int16_t dstOffset = 0;
    bool dstZone = true;
    uint32_t timestamp;
    uint32_t ntpTime = 0;
    uint32_t utcTime = 0;   
    time_t utcSTD, utcDST;
    time_t dstTime, stdTime;
    uint16_t yearDST;
    char timeString[64];
    struct ruleDST {
	    char tzName[6]; // five chars max
	    int8_t week;   // First, Second, Third, Fourth, or Last week of the month
	    int8_t wday;   // day of week, 0 = Sun, 2 = Mon, ... 6 = Sat
	    int8_t month;  // 0 = Jan, 1 = Feb, ... 11=Dec
	    int8_t hour;   // 0 - 23
	    int tzOffset;   // offset from UTC in minutes
      } dstStart, dstEnd;
    void init();
    int8_t ntpUpdate();
    time_t localTime();
    void currentTime();
    void beginDST();
    time_t calcDateDST(struct ruleDST rule, int year);
    bool summerTime();
};

#endif
