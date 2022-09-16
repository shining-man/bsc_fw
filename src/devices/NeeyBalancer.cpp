// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT


#include "devices/NeeyBalancer.h"
#include "BmsData.h"


NeeyBalancer::NeeyBalancer() {
};

void NeeyBalancer::init() {
}

void NeeyBalancer::neeyBalancerCopyData(uint8_t devNr, uint8_t* pData, size_t length) {

  if(length==241)
  {
    float f_lTmpValue;
    //Cellspannung
    for(uint8_t i=0;i<24;i++)
    {
      memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_CELVOLTAGE+(i*4), 4);
      setBmsCellVoltage(devNr,i,(uint32_t)(f_lTmpValue*1000));
    }

    //Avg. Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_AVERAGEVOLTAGE, 4);
    setBmsAvgVoltage(devNr,(uint32_t)(f_lTmpValue*1000));

    //Delta Cell Voltage
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_DELTACELLVOLTAGE, 4);
    setBmsMaxCellDifferenceVoltage(devNr,(uint32_t)(f_lTmpValue*1000));
   
    /*
    Errors:
    STATUS_OK                0
    STATUS_CELL_OVP          1   //bit0 single cell overvoltage protection 
    STATUS_CELL_UVP          2   //bit1 single cell undervoltage protection    
    STATUS_PACK_OVP          4   //bit2 whole pack overvoltage protection 
    STATUS_PACK_UVP          8   //bit3 Whole pack undervoltage protection     
    STATUS_CHG_OTP          16   //bit4 charging over-temperature protection 
    STATUS_CHG_UTP          32   //bit5 charging low temperature protection 
    STATUS_DSG_OTP          64   //bit6 Discharge over temperature protection  
    STATUS_DSG_UTP         128   //bit7 discharge low temperature protection   
    STATUS_CHG_OCP         256   //bit8 charging overcurrent protection 
    STATUS_DSG_OCP         512   //bit9 Discharge overcurrent protection       
    STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection              
    STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error 
    STATUS_SOFT_LOCK      4096   //bit12 software lock MOS 
    */
    uint32_t u32_lErrors = getBmsErrors(devNr); //Alte Fehler lesen
    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_ERRCELLOV, 4);
    if(f_lTmpValue==0){u32_lErrors &= ~(1 << 0);}
    else{u32_lErrors |= (1 << 0);}

    memcpy(&f_lTmpValue, pData+OFFSET_NEEYBAL4A_DATA0x2_ERRCELLUV, 4);
    if(f_lTmpValue==0){u32_lErrors &= ~(1 << 1);}
    else{u32_lErrors |= (1 << 1);}

    setBmsErrors(devNr, u32_lErrors);
 
    bmsDataSemaphoreTake();
    memcpy(&bmsCellResistance[devNr][0], pData+OFFSET_NEEYBAL4A_DATA0x2_CELLRESISTANCE, 4*24); 
    memcpy(&bmsTotalVoltage[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_TOTALVOLTAGE, 4);
    memcpy(&bmsMaxVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MAXVOLTCELLNR, 1);
    memcpy(&bmsMinVoltageCellNumber[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_MINVOLTCELLNR, 1);
    memcpy(&bmsIsBalancingActive[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCING, 1);
    memcpy(&bmsBalancingCurrent[devNr], pData+OFFSET_NEEYBAL4A_DATA0x2_BALANCINGCUR, 4);
    memcpy(&bmsTempature[devNr][0], pData+OFFSET_NEEYBAL4A_DATA0x2_TEMPERATUR, 4*2);
    bmsDataSemaphoreGive();

    uint32_t f_lMacZellVoltage = getBmsCellVoltage(devNr,getBmsMaxVoltageCellNumber(devNr));
    uint32_t f_lMinZellVoltage = getBmsCellVoltage(devNr,getBmsMinVoltageCellNumber(devNr));
    setBmsMaxCellDifferenceVoltage(devNr, f_lMacZellVoltage-f_lMinZellVoltage);

    setBmsMaxCellVoltage(devNr, f_lMacZellVoltage);
  }
  else if(length==59)
  {

  }

}

