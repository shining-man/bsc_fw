// Copyright (c) 2022 tobias
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#ifndef JBDBMS_H
#define JBDBMS_H

#include "Arduino.h"
#include "defines.h"

#define JBDBMS_MAX_ANSWER_LEN   0xff


#define JBD_BYTE_OFFSET 4
#define JBD_BYTE_TOTAL_VOLTAGE     JBD_BYTE_OFFSET +  0
#define JBD_BYTE_CURRENT           JBD_BYTE_OFFSET +  2
#define JBD_BYTE_BALANCE_CAPACITY  JBD_BYTE_OFFSET +  4
#define JBD_BYTE_FULL_CAPACITY     JBD_BYTE_OFFSET +  6
#define JBD_BYTE_CYCLE             JBD_BYTE_OFFSET +  8
#define JBD_BYTE_PRODUCTION_DATE   JBD_BYTE_OFFSET + 10
#define JBD_BYTE_BALANCE_STATUS    JBD_BYTE_OFFSET + 12
#define JBD_BYTE_BALANCE_STATUS_2  JBD_BYTE_OFFSET + 14
#define JBD_BYTE_CURRENT_ERRORS    JBD_BYTE_OFFSET + 16
#define JBD_BYTE_SOFTWARE_VERSION  JBD_BYTE_OFFSET + 18
#define JBD_BYTE_RSOC              JBD_BYTE_OFFSET + 19
#define JBD_BYTE_FET_STATUS        JBD_BYTE_OFFSET + 20
#define JBD_BYTE_BATTERY_SERIES    JBD_BYTE_OFFSET + 21
#define JBD_BYTE_NTC_NUMBER        JBD_BYTE_OFFSET + 22
#define JBD_BYTE_NTCn              JBD_BYTE_OFFSET + 23



//Protection State
#define JBDBMS_STATUS_OK                0
#define JBDBMS_STATUS_CELL_OVP          1   //bit0 single cell overvoltage protection 
#define JBDBMS_STATUS_CELL_UVP          2   //bit1 single cell undervoltage protection    
#define JBDBMS_STATUS_PACK_OVP          4   //bit2 whole pack overvoltage protection 
#define JBDBMS_STATUS_PACK_UVP          8   //bit3 Whole pack undervoltage protection     
#define JBDBMS_STATUS_CHG_OTP          16   //bit4 charging over-temperature protection 
#define JBDBMS_STATUS_CHG_UTP          32   //bit5 charging low temperature protection 
#define JBDBMS_STATUS_DSG_OTP          64   //bit6 Discharge over temperature protection  
#define JBDBMS_STATUS_DSG_UTP         128   //bit7 discharge low temperature protection   
#define JBDBMS_STATUS_CHG_OCP         256   //bit8 charging overcurrent protection 
#define JBDBMS_STATUS_DSG_OCP         512   //bit9 Discharge overcurrent protection       
#define JBDBMS_STATUS_SHORT_CIRCUIT  1024   //bit10 short circuit protection              
#define JBDBMS_STATUS_AFE_ERROR      2048   //bit11 Front-end detection IC error 
#define JBDBMS_STATUS_SOFT_LOCK      4096   //bit12 software lock MOS 
#define JBDBMS_STATUS_RESERVED1      8192   //bit13 Reserved 
#define JBDBMS_STATUS_RESERVED2     16384   //bit14 Reserved                                             
#define JBDBMS_STATUS_RESERVED3     32768   //bit15 Reserved 


bool JbdBms_readBmsData(Stream *port, uint8_t devNrv, uint8_t txEnRS485pin, uint8_t u8_addData);


#endif 
