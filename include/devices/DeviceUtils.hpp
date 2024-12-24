#ifndef DEVICE_UTILS_HPP
#define DEVICE_UTILS_HPP

#include <iostream>
#include <string>

namespace deviceUtils {
  class DeviceUtils {
  private:

  public:
    DeviceUtils();  // Konstruktor-Deklaration
    ~DeviceUtils(); // Destruktor-Deklaration

    char ByteToAsciiHex(uint8_t v);
    void ByteToAsciiHex(uint8_t *dest, const uint8_t *data, size_t length);

    uint8_t  AsciiHexToByte(char a, char b);
    uint8_t  AsciiHexToNibble(char a);
    uint16_t AsciiHexToU16(const uint8_t *t_message, uint8_t pos);
    uint32_t AsciiHexToU32(const uint8_t *t_message, uint8_t pos);

  };
}

#endif // DEVICE_UTILS_HPP