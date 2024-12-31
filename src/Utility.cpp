// Copyright (c) 2024 tobias
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Utility.h"
#include "defines.h"


static const char *TAG = "UTILITY";

// Private
std::string toHexString(const uint8_t* number) 
{
  std::stringstream ss;
  ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(*number);
  return ss.str();
}


// Public
bool parseMacAddress(const std::string& macStr, uint8_t* mac)
{
  std::istringstream ss(macStr);
  int value;
  char delimiter;

  for (int i = 0; i < 6; ++i)
  {
    if (!(ss >> std::hex >> value))
    {
      return false; // Fehler beim Lesen eines Teils der MAC-Adresse
    }

    if (value < 0x00 || value > 0xFF)
    {
      return false; // Ungültiger Wert außerhalb des Bereichs
    }
    mac[i] = static_cast<uint8_t>(value);

    // Lese das Trennzeichen (":"), außer nach dem letzten Byte
    if (i < 5 && !(ss >> delimiter && delimiter == ':'))
    {
      return false; // Fehler: Trennzeichen nicht gefunden
    }
  }

  return true;
}


std::string macAddressToString(const uint8_t* mac)
{
  std::ostringstream ss;
  for (int i = 0; i < 6; ++i)
  {
    // Schreibe jedes Byte als zweistellige Hex-Zahl in den Stream
    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac[i]);
    if (i < 5) ss << ":"; 
  }
  return ss.str();
}


std::string floatToString(float value, uint8_t decimalPlaces)
{
  char buffer[20];

  std::string format = "%." + std::to_string(decimalPlaces) + "f";
  snprintf(buffer, sizeof(buffer), format.c_str(), value);
  std::string tmpStr(buffer);
  return tmpStr;
}


void buffer2Log(uint8_t *p_message, uint8_t len)
{
  std::string recvBytes = "";
  uint8_t logByteCount = 0;
  BSC_LOGI(TAG,"len=%i", len);
  for(uint8_t x = 0; x < len; x++)
  {
    logByteCount++;
    recvBytes += "0x";
    recvBytes += toHexString(&p_message[x]);
    //recvBytes += p_message[x];
    recvBytes += " ";
    if(logByteCount == 20)
    {
      BSC_LOGI(TAG, "%s", recvBytes.c_str());
      recvBytes = "";
      logByteCount = 0;
    }
  }
  if(logByteCount > 0) BSC_LOGI(TAG, "%s", recvBytes.c_str());
}