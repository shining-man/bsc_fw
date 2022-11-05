// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "debug.h"
#include "SoftwareSerial.h"

SoftwareSerial debugPort;

void debugInit()
{
  debugPort.enableIntTx(false);
  debugPort.enableRx(false);
  debugPort.begin(115200, SWSERIAL_8N1, 3, 1, false, 64, 11);
}

/*void debugPrintTest()
{
    
    uint8_t i=55;
    uint8_t testArr[5];
    testArr[1]='M';

    //Print
    debugPrint("Test1_");
    debugPrint(F("Test2_"));
    debugPrint(String(testArr[1]));
    debugPrint(123);

    //Println
    debugPrintln("Test1_");
    debugPrintln(F("Test2_"));
    debugPrintln(String(testArr[1]));
    debugPrintln(123);

    //printf
    debugPrintf("Test printf: i=%i\n",i);
}*/

//print
void debugPrint(const char str[])
{
    debugPort.print(str);
}

size_t debugPrint(const String &s)
{
    return debugPort.write(s.c_str(), s.length());
}

void debugPrint(int n)
{
    debugPort.print(n);
}


//println
void debugPrintln(const char str[])
{
    debugPort.print(str);
    debugPort.print("\n");

    Serial.print("");
}

void debugPrintln(const String &s)
{
    debugPrint(s);
    debugPrint("\n");
}

void debugPrintln(int n)
{
    debugPort.print(n);
    debugPort.print("\n");
}

//printf
size_t debugPrintf(const char *format, ...)
{
    char loc_buf[64];
    char * temp = loc_buf;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
    va_end(copy);
    if(len < 0) {
        va_end(arg);
        return 0;
    };
    if(len >= sizeof(loc_buf)){
        temp = (char*) malloc(len+1);
        if(temp == NULL) {
            va_end(arg);
            return 0;
        }
        len = vsnprintf(temp, len+1, format, arg);
    }
    va_end(arg);
    //len = write((uint8_t*)temp, len);
    len = debugPort.write((uint8_t*)temp, len);
    if(temp != loc_buf){
        free(temp);
    }
    return len;
}