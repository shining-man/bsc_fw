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

  uint8_t DeviceUtils::AsciiHexToNibble(char a)
  {
    if (a >= '0' && a <= '9') return a - '0';
    if (a >= 'A' && a <= 'F') return a - 'A' + 10;
    if (a >= 'a' && a <= 'f') return a - 'a' + 10;
    return 0xFF; // Ungültiger Wert
  }

  uint8_t DeviceUtils::AsciiHexToByte(char a, char b)
  {
    if (AsciiHexToNibble(a) == 0xFF || AsciiHexToNibble(b) == 0xFF) {
      return 0xFF;
    }
    return (AsciiHexToNibble(a) << 4) | AsciiHexToNibble(b);
  }

  uint16_t DeviceUtils::AsciiHexToU16(const uint8_t *t_message, uint8_t pos)
  {
      if (t_message == nullptr) {
          return 0xFFFF; 
      }

      return static_cast<uint16_t>(
              (AsciiHexToByte(t_message[pos + 2], t_message[pos + 3]) << 8) |
              AsciiHexToByte(t_message[pos], t_message[pos + 1]));
  }

  uint32_t DeviceUtils::AsciiHexToU32(const uint8_t *t_message, uint8_t pos)
  {
      if (t_message == nullptr) {
          return 0xFFFFFFFF; // Fehlerwert
      }

      return (static_cast<uint32_t>(AsciiHexToByte(t_message[pos + 6], t_message[pos + 7])) << 24) |
            (static_cast<uint32_t>(AsciiHexToByte(t_message[pos + 4], t_message[pos + 5])) << 16) |
            (static_cast<uint32_t>(AsciiHexToByte(t_message[pos + 2], t_message[pos + 3])) << 8) |
            static_cast<uint32_t>(AsciiHexToByte(t_message[pos], t_message[pos + 1]));
  }


  char DeviceUtils::ByteToAsciiHex(uint8_t v)
  {
      if (v >= 16) return 'F';
      return (v >= 10) ? ('A' + (v - 10)) : ('0' + v);
  }

  void DeviceUtils::ByteToAsciiHex(uint8_t *dest, const uint8_t *data, size_t length)
  {
    if (dest == nullptr || data == nullptr || length == 0) {
        return;
    }

    for (size_t i = 0; i < length; i++) {
        dest[2 * i] = ByteToAsciiHex((data[i] & 0xF0) >> 4);
        dest[2 * i + 1] = ByteToAsciiHex(data[i] & 0x0F);
    }
  } 

}