// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#ifndef NEEYBALANCER_H
#define NEEYBALANCER_H

#include "Arduino.h"
#include "defines.h"

//Head
#define OFFSET_NEEYBAL4A_HEAD_STARTFRAME1             0
#define OFFSET_NEEYBAL4A_HEAD_STARTFRAME2             1
#define OFFSET_NEEYBAL4A_HEAD_ADRESS                  2
#define OFFSET_NEEYBAL4A_HEAD_RW                      3
#define OFFSET_NEEYBAL4A_HEAD_CMD1                    4
#define OFFSET_NEEYBAL4A_HEAD_CMD2                    5
#define OFFSET_NEEYBAL4A_HEAD_DATALEN                 6

//Data 0x02
#define OFFSET_NEEYBAL4A_DATA0x2_SERIALNR                 8
#define OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE               9
#define OFFSET_NEEYBAL4A_DATA0x2_CELLRESISTANCE         105
#define OFFSET_NEEYBAL4A_DATA0x2_TOTALVOLTAGE           201
#define OFFSET_NEEYBAL4A_DATA0x2_AVERAGEVOLTAGE         205
#define OFFSET_NEEYBAL4A_DATA0x2_DELTACELLVOLTAGE       209
#define OFFSET_NEEYBAL4A_DATA0x2_MAXVOLTCELLNR          213
#define OFFSET_NEEYBAL4A_DATA0x2_MINVOLTCELLNR          214
#define OFFSET_NEEYBAL4A_DATA0x2_SINGLENR               215
#define OFFSET_NEEYBAL4A_DATA0x2_BALANCING              216
#define OFFSET_NEEYBAL4A_DATA0x2_BALANCINGCUR           217
#define OFFSET_NEEYBAL4A_DATA0x2_TEMPERATUR             221
#define OFFSET_NEEYBAL4A_DATA0x2_ERRCELLDETECTION       229 //Cell detection failed bitmask (24 bits = 1 bit per cell)
#define OFFSET_NEEYBAL4A_DATA0x2_ERRCELLOV              232 //Cell overvoltage bitmask (24 cells)
#define OFFSET_NEEYBAL4A_DATA0x2_ERRCELLUV              235 //Cell undervoltage bitmask (24 cells)
#define OFFSET_NEEYBAL4A_DATA0x2_ERRCELLPOLARITY        238 //Cell polarity error bitmask (24 cells)

#define OFFSET_NEEYBAL4A_DATA0x2_ERRHIGHLINERESISTANCE  241 //Excessive line resistance bitmask (24 cells)
#define OFFSET_NEEYBAL4A_DATA0x2_ERRSYSOVERHEATING      244 //System overheating (Bit0: Sensor 1 warning, Bit1: Sensor 2 warning)
#define OFFSET_NEEYBAL4A_DATA0x2_CHARGINGFAULT          245 //Charging fault (0x00: Off, 0x01: On)
#define OFFSET_NEEYBAL4A_DATA0x2_DISCHARGEFAULT         246 //Discharge fault (0x00: Off, 0x01: On)


  // Byte Len  Payload                Content              Coeff.      Unit        Example value
  // 0     2   0x55 0xAA              Header
  // 2     1   0x11                   Device address
  // 3     1   0x01                   Function (read)
  // 4     2   0x02 0x00              Command (cell info)
  // 6     2   0x2C 0x01              Length (300 bytes)
  // 8     1   0x38                   Frame counter
  // 9     4   0xE7 0xFA 0x50 0x40              Cell 1 voltage
  // 13    4   0xB6 0x04 0x51 0x40              Cell 2 voltage
  // 17    4   0x85 0x0E 0x51 0x40              Cell 3 voltage
  // 21    4   0xF0 0x05 0x51 0x40              Cell 4 voltage
  // 25    4   0xB6 0x04 0x51 0x40              Cell 5 voltage
  // 29    4   0x75 0x1E 0x51 0x40              Cell 6 voltage
  // 33    4   0x7F 0x4F 0x51 0x40              Cell 7 voltage
  // 37    4   0x43 0x02 0x51 0x40              Cell 8 voltage
  // 41    4   0x1C 0x3D 0x51 0x40              Cell 9 voltage
  // 45    4   0x78 0x6A 0x51 0x40              Cell 10 voltage
  // 49    4   0xFE 0x82 0x51 0x40              Cell 11 voltage
  // 53    4   0x16 0x7E 0x51 0x40              Cell 12 voltage
  // 57    4   0xBC 0x76 0x51 0x40              Cell 13 voltage
  // 61    4   0x16 0x7E 0x51 0x40              Cell 14 voltage
  // 65    4   0x8B 0x80 0x51 0x40              Cell 15 voltage
  // 69    4   0xCA 0x66 0x51 0x40              Cell 16 voltage
  // 73    4   0x00 0x00 0x00 0x00              Cell 17 voltage
  // 77    4   0x00 0x00 0x00 0x00              Cell 18 voltage
  // 81    4   0x00 0x00 0x00 0x00              Cell 19 voltage
  // 85    4   0x00 0x00 0x00 0x00              Cell 20 voltage
  // 89    4   0x00 0x00 0x00 0x00              Cell 21 voltage
  // 93    4   0x00 0x00 0x00 0x00              Cell 22 voltage
  // 97    4   0x00 0x00 0x00 0x00              Cell 23 voltage
  // 101   4   0x00 0x00 0x00 0x00              Cell 24 voltage
  // 105   4   0x35 0x93 0x24 0x3E              Cell 1 resistance
  // 109   4   0x68 0x94 0x26 0x3E              Cell 2 resistance
  // 113   4   0x3D 0x25 0x1B 0x3E              Cell 3 resistance
  // 117   4   0x90 0x8E 0x1B 0x3E              Cell 4 resistance
  // 121   4   0xB3 0xF3 0x23 0x3E              Cell 5 resistance
  // 125   4   0x2E 0x91 0x25 0x3E              Cell 6 resistance
  // 127   4   0xC6 0x1B 0x1A 0x3E              Cell 7 resistance
  // 133   4   0x4A 0x7C 0x1C 0x3E              Cell 8 resistance
  // 137   4   0x6F 0x1B 0x1A 0x3E              Cell 9 resistance
  // 141   4   0xC2 0x43 0x1B 0x3E              Cell 10 resistance
  // 145   4   0x85 0x1E 0x18 0x3E              Cell 11 resistance
  // 149   4   0x4B 0x27 0x19 0x3E              Cell 12 resistance
  // 153   4   0x5E 0xDF 0x18 0x3E              Cell 13 resistance
  // 157   4   0xD0 0xEB 0x1A 0x3E              Cell 14 resistance
  // 161   4   0xE6 0xD4 0x18 0x3E              Cell 15 resistance
  // 165   4   0x0C 0xFE 0x18 0x3E              Cell 16 resistance
  // 169   4   0x00 0x00 0x00 0x00              Cell 17 resistance
  // 173   4   0x00 0x00 0x00 0x00              Cell 18 resistance
  // 177   4   0x00 0x00 0x00 0x00              Cell 19 resistance
  // 181   4   0x00 0x00 0x00 0x00              Cell 20 resistance
  // 185   4   0x00 0x00 0x00 0x00              Cell 21 resistance
  // 189   4   0x00 0x00 0x00 0x00              Cell 22 resistance
  // 193   4   0x00 0x00 0x00 0x00              Cell 23 resistance
  // 197   4   0x00 0x00 0x00 0x00              Cell 24 resistance
  // 201   4   0xDE 0x40 0x51 0x42              Total voltage
  // 205   4   0xDE 0x40 0x51 0x40              Average cell voltage
  // 209   4   0x00 0x17 0x08 0x3C              Delta Cell Voltage
  // 213   1   0x0A                             Max voltage cell
  // 214   1   0x00                             Min voltage cell
  // 215   1   0x0F                             Single number (not exposed at the android app)
  // 216   1   0x05                             Operation status                      OFFSET_NEEYBAL4A_DATA0x2_BALANCING
  //                                              0x01: Wrong cell count
  //                                              0x02: AcqLine Res test
  //                                              0x03: AcqLine Res exceed
  //                                              0x04: Systest Completed
  //                                              0x05: Balancing
  //                                              0x06: Balancing finished
  //                                              0x07: Low voltage
  //                                              0x08: System Overtemp
  //                                              0x09: Host fails
  //                                              0x0A: Low battery voltage - balancing stopped
  //                                              0x0B: Temperature too high - balancing stopped
  //                                              0x0C: Self-test completed
  // 217   4   0x19 0xA1 0x82 0xC0              Balancing current
  // 221   4   0xC3 0xF5 0x48 0x42              Temperature 1
  // 225   4   0xC3 0xF5 0x48 0x42              Temperature 2
  // 229   3   0x00 0x00 0x00                   Cell detection failed bitmask (24 bits = 1 bit per cell)
  // 232   3   0x00 0x00 0x00                   Cell overvoltage bitmask (24 cells)
  // 235   3   0x00 0x00 0x00                   Cell undervoltage bitmask (24 cells)
  // 238   3   0x00 0x00 0x00                   Cell polarity error bitmask (24 cells)
  // 241   3   0x00 0x00 0x00                   Excessive line resistance bitmask (24 cells)
  // 244   1   0x00                             System overheating
  //                                              Bit0: Temperature sensor 1 warning
  //                                              Bit1: Temperature sensor 2 warning
  // 245   1   0x00                             Charging fault
  //                                              0x00: Off
  //                                              0x01: On
  // 246   1   0x00                             Discharge fault
  //                                              0x00: Off
  //                                              0x01: On
  // 247   1   0x00                             Unknown
  //                                              Bit0: Read failed
  //                                              Bit1: Write failed
  // 248   6   0x00 0x00 0x00 0x00 0x00 0x00    Reserved
  // 254   4   0x76 0x2E 0x09 0x00              Uptime?
  // 258   40  0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  //           0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00
  // 298   1   0xB6
  // 299   1   0xFF


class NeeyBalancer {
public:
  NeeyBalancer();

  void init();
  static void neeyBalancerCopyData(uint8_t devNr, uint8_t* pData, size_t length);


private:

};

#endif
