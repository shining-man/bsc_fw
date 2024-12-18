// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "devices/DeviceUtils.hpp"

namespace deviceUtils
{
  DeviceUtils::DeviceUtils() {
    ;
  }

  DeviceUtils::~DeviceUtils() {
    ;
  }


  uint8_t DeviceUtils::AsciiHexToByte(char a, char b)
  {
    a = (a<='9') ? a-'0' : (a&0x7)+9;
    b = (b<='9') ? b-'0' : (b&0x7)+9;
    return (a<<4)+b;
  }

  uint16_t DeviceUtils::AsciiHexToU16(uint8_t *t_message, uint8_t pos)
  {
  return (uint16_t)(((uint16_t)AsciiHexToByte(t_message[pos+2], t_message[pos+3]) << 8) |
    AsciiHexToByte(t_message[pos], t_message[pos+1]));
  }

  uint32_t DeviceUtils::AsciiHexToU32(uint8_t *t_message, uint8_t pos)
  {
  return (uint32_t)
    (
    (uint32_t)AsciiHexToByte(t_message[pos+6], t_message[pos+7]) << 24|
    (uint32_t)AsciiHexToByte(t_message[pos+4], t_message[pos+5]) << 16|
    (uint32_t)AsciiHexToByte(t_message[pos+2], t_message[pos+3]) << 8 |
    AsciiHexToByte(t_message[pos], t_message[pos+1])
    );
  }


  char DeviceUtils::ByteToAsciiHex(uint8_t v)
  {
    return v>=10 ? 'A'+(v-10) : '0'+v;
  }


  void DeviceUtils::ByteToAsciiHex(uint8_t *dest, uint8_t *data, size_t length)
  {
    if(length==0) return;

    for(size_t i=0; i<length; i++)
    {
      dest[2*i] = ByteToAsciiHex((data[i] & 0xF0) >> 4);
      dest[2*i+1] = ByteToAsciiHex(data[i] & 0x0F);
    }
  }

}