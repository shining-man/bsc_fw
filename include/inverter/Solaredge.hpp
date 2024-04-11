#pragma once

#include <cstdint>
#include "inverter/Inverter.hpp"

uint16_t bum=0;
uint16_t bum1=0;
int16_t bum2=0;

struct data355_SOLAREDGERWS //Anpassung SolaredgeRWS
    {
           uint16_t soc; //state of charge
           uint16_t soh; //state of health
           uint16_t bum; //leere Bytes
           uint16_t bum1; //leere Bytes
    };

struct data356_SOLAREDGERWS //Anpassung SolaredgeRWS
    {
      int16_t voltage;
      int16_t current;
      int16_t temperature;
      int16_t bum2;//leere Bytes
    };