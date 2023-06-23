/**
 * NTP library for Arduino framework
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

#include "NTP.h"

//static const char *TAG = "NTP";

NTP::NTP(UDP& udp) {
	this->udp = &udp;
	}

NTP::~NTP() {
	stop();
	}

void NTP::begin(const char* server) {
	this->server = server;
	init(); 
	}

void NTP::begin(IPAddress serverIP) {
	this->serverIP = serverIP;
	init();
	}

void NTP::init() {
	memset(ntpRequest, 0, NTP_PACKET_SIZE);
  ntpRequest[0] = 0b11100011; // LI, Version, Mode
  ntpRequest[1] = 0;          // Stratum, or type of clock
  ntpRequest[2] = 6;          // Polling Interval
  ntpRequest[3] = 0xEC;       // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  ntpRequest[12]  = 49;
  ntpRequest[13]  = 0x4E;
  ntpRequest[14]  = 49;
  ntpRequest[15]  = 52;
	udp->begin(NTP_PORT);
	ntpUpdate();
	lastUpdate=0;
	if (dstZone) {
		timezoneOffset = dstEnd.tzOffset * SECS_PER_MINUTES;
		dstOffset = (dstStart.tzOffset - dstEnd.tzOffset) * SECS_PER_MINUTES;
		currentTime();
		beginDST();
		}
	}

void NTP::stop() {
	udp->stop();
	}

int8_t NTP::update() {
	//ESP_LOGI(TAG,"NTP update: %i, %i, %i",millis()-lastUpdate,lastUpdate,interval);
	if ((millis() - lastUpdate >= interval) || lastUpdate == 0) {
		return ntpUpdate();
		}
	return 3;
	}

int8_t NTP::ntpUpdate() {
	if (server == nullptr) udp->beginPacket(serverIP, NTP_PORT);
	else udp->beginPacket(server, NTP_PORT);
	udp->write(ntpRequest, NTP_PACKET_SIZE);
	udp->endPacket();
	uint8_t timeout = 0;
	uint8_t size = 0;
	do {
		delay (10);
		size = udp->parsePacket();
		if (timeout > 100) return 2;
		timeout++;
		} while (size != 48);
	lastUpdate = millis() - (10 * (timeout + 1));
	udp->read(ntpQuery, NTP_PACKET_SIZE);
	#ifdef __AVR__
 		unsigned long highWord = word(ntpQuery[40], ntpQuery[41]);
		unsigned long lowWord = word(ntpQuery[42], ntpQuery[43]);
		timestamp = highWord << 16 | lowWord;
		if (timestamp != 0) {
 			ntpTime = timestamp;
 			utcTime = ntpTime - NTP_OFFSET;
			}
		else return 0;
 	#else
 		timestamp = ntpQuery[40] << 24 | ntpQuery[41] << 16 | ntpQuery[42] << 8 | ntpQuery[43];
		if (timestamp != 0) {
 			ntpTime = timestamp;
 			utcTime = ntpTime - SEVENTYYEARS;
			}
		else return 0;
 	#endif
	return 1;
	}

void NTP::updateInterval(uint32_t interval) {
	this->interval = interval;
	}

void NTP::ruleDST(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset) { //Sommerzeit
	strcpy(dstStart.tzName, tzName);
	dstStart.week = week;
	dstStart.wday = wday;
	dstStart.month = month;
	dstStart.hour = hour;
	dstStart.tzOffset = tzOffset;
	}

const char* NTP::ruleDST() {
	if(dstZone) {
		return ctime(&dstTime);
		}
	else return RULE_DST_MESSAGE;
	}

void NTP::ruleSTD(const char* tzName, int8_t week, int8_t wday, int8_t month, int8_t hour, int tzOffset) {
	strcpy(dstEnd.tzName, tzName);
	dstEnd.week = week;
	dstEnd.wday = wday;
	dstEnd.month = month;
	dstEnd.hour = hour;
	dstEnd.tzOffset = tzOffset;
	}
		
const char* NTP::ruleSTD() {
	if(dstZone) {
		return ctime(&stdTime);
		}
	else return RULE_STD_MESSAGE;
	}

const char* NTP::tzName() {
	if (dstZone) {
		if (summerTime()) return dstStart.tzName;
		else return dstEnd.tzName;
		}
	return GMT_MESSAGE;
	}

void NTP::timeZone(int8_t tzHours, int8_t tzMinutes) {
	this->tzHours = tzHours;
	this->tzMinutes = tzMinutes;
	timezoneOffset = tzHours * 3600;
	if (tzHours < 0) {
		timezoneOffset -= tzMinutes * 60;
		}
	else {
		timezoneOffset += tzMinutes * 60;
		}
	}

void NTP::isDST(bool dstZone) {
	this->dstZone = dstZone;
	}

bool NTP::isDST() {
	return summerTime();
	}

time_t NTP::epoch() {
	currentTime();
	return utcCurrent; 
	}

time_t NTP::localEpoch() {
	currentTime();
	return local; 
	}

void NTP::currentTime() {
	utcCurrent = utcTime + ((millis() - lastUpdate) / 1000); 
	if (dstZone) {
		if (summerTime()) {
			local = utcCurrent + dstOffset + timezoneOffset;
			current = gmtime(&local);    
			}
		else {
			local = utcCurrent + timezoneOffset;
			current = gmtime(&local);
			}
		if ((current->tm_year + 1900) > yearDST) beginDST();
		}
	else {
		local = utcCurrent + timezoneOffset;
		current = gmtime(&local);
		}
	}

int16_t NTP::year() {
	currentTime();
	return current->tm_year + 1900;
	}

int8_t NTP::month() {
	currentTime();
	return current->tm_mon + 1;
	}

int8_t NTP::day() {
	currentTime();
	return current->tm_mday;
	}

int8_t NTP::weekDay() {
	currentTime();
	return current->tm_wday;
	}

int8_t NTP::hours() {
	currentTime();
	return current->tm_hour;
	}

int8_t NTP::minutes() {
	currentTime();
	return current->tm_min;
	}

int8_t NTP::seconds() {
	currentTime();
	return current->tm_sec;
	}

const char* NTP::formattedTime(const char *format) {
	currentTime();
	memset(timeString, 0, sizeof(timeString));
	strftime(timeString, sizeof(timeString), format, current);
	return timeString;
	}

void NTP::beginDST() {
	dstTime = calcDateDST(dstStart, current->tm_year + 1900);
	utcDST = dstTime - (dstEnd.tzOffset * SECS_PER_MINUTES);
	stdTime = calcDateDST(dstEnd, current->tm_year + 1900);
	utcSTD = stdTime - (dstStart.tzOffset * SECS_PER_MINUTES);
	yearDST = current->tm_year + 1900;
	}

time_t NTP::calcDateDST(struct ruleDST rule, int year) {
	uint8_t month = rule.month;
	uint8_t week = rule.week;
	if (week == 0) {
		if (month++ > 11) {
			month = 0;
			year++;
			}
		week = 1;
		}

	struct tm tm;
	tm.tm_hour = rule.hour;
	tm.tm_min = 0;
	tm.tm_sec = 0;
	tm.tm_mday = 1;
	tm.tm_mon = month;
	tm.tm_year = year - 1900;
	time_t t = mktime(&tm);

	t += ((rule.wday - tm.tm_wday + 7) % 7 + (week - 1) * 7 ) * SECS_PER_DAY;
	if (rule.week == 0) t -= 7 * SECS_PER_DAY;
	return t;
	}

bool NTP::summerTime() {
	if ((utcCurrent > utcDST) && (utcCurrent <= utcSTD)) {
		return true;
		}
	else {
		return false;
		}
	}

uint32_t NTP::ntp() {
	return ntpTime;
	}

uint32_t NTP::utc() {
	return utcTime;
	}
