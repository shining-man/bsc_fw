// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "Json.h"

Json::Json()
{  
};


int arrSize(const char *a)
{
    int it = 0;
    int size = 0;
    while(a[it] != '\0')
    {
        ++size;
        ++it;
    }
    return size;
}


bool Json::jsonIndexPos_Array(const char *json, int idx, long &startPos, long& endPos)
{
    int32_t  posArrayFirstOpen = -1;
    uint32_t posElementOpen = 0;
    uint32_t posElementClose = 0;
    uint8_t  elementOpenCnt = 0;
    uint8_t  elementCloseCnt = 0;

    uint8_t arrayNr = 0;

    for (uint32_t i = startPos; i < arrSize(json); i++)
    {
        char c = json[i];
        if (c == '{') 
        {
            if (elementOpenCnt == elementCloseCnt)
            {
                posArrayFirstOpen = i;
            }
            posElementOpen = i; 
            elementOpenCnt++;
        }
        else if (c == '}') 
        { 
            posElementClose = i; 
            elementCloseCnt++; 

            //
            if (elementCloseCnt > 0 && elementOpenCnt == elementCloseCnt)
            { 
                if (arrayNr == idx) //Index gefunden
                {
                    startPos = posArrayFirstOpen;
                    endPos = posElementClose;
                    return true;
                }
                arrayNr++;
            }
        }
    }

    startPos = 0;
    endPos = 0;
    return true;
}


uint16_t Json::getArraySize(const char *json, long startPos)
{
    uint8_t  arrayOpenCnt = 0;
    uint8_t  arrayCloseCnt = 0;
    uint8_t  elementOpenCnt = 0;
    uint8_t  elementCloseCnt = 0;

    uint16_t arrayCnt = 0;
    uint8_t  arrayCntMerker = 1;

    for (uint32_t i = startPos; i < arrSize(json); i++)
    {
        char c = json[i];
        if (c == '[') 
        { 
            arrayOpenCnt++; 
        }
        else if (c == ']')
        {
            arrayCloseCnt++;
        }
        else if (c == '{') 
        {
            arrayCntMerker=1;
            elementOpenCnt++;
        }
        else if (c == '}') 
        { 
            elementCloseCnt++; 
        }

        //
        if (elementCloseCnt > 0 && elementOpenCnt == elementCloseCnt && arrayCntMerker==1)
        { 
            arrayCnt++;
            arrayCntMerker=0;
        }
        if (arrayOpenCnt>0 && arrayOpenCnt == arrayCloseCnt)
        {
            return arrayCnt;
        }
    }

    return arrayCnt;
}

bool Json::getValue(const char * json, int idx, String name, uint32_t searchStartPos, String& retValue, uint32_t& arrayStart)
{
    arrayStart = 0;
    retValue = "";
    long startPos = searchStartPos;
    long endPos = 0;
    jsonIndexPos_Array(json, idx, startPos, endPos);

    String label = "";
    String sName = "\"";
    sName += name;
    sName += "\":";
    const char* cName = sName.c_str();
    uint8_t cNameLen = strlen(cName);
    uint8_t cNameFoundToPos = 0;
    uint8_t searchState = 0;
    uint8_t tmpStringAnfang = 0;

    for (uint32_t p = startPos; p < endPos+1; p++)
    {
        if (searchState == 0)
        {
      
            if (json[p] == cName[cNameFoundToPos]) //Ersten Zeichen gefunden
            {
                cNameFoundToPos++;
                if (cNameLen == cNameFoundToPos)
                {
                    searchState++;
                }
            }
            else
            {
                cNameFoundToPos = 0;
            }
        }
        else if (searchState == 1)
        {
            if (json[p] == '[') //Es kommt jetzt ein Array
            {
                searchState = 2;
            }
            else //kein Array
            {
                //Wenn json[startPos-1] kein "'" ist dann startPos--
                if (p > 0)
                {
                    if (json[p] != '"')
                    {
                        p--;
                    }
                }
                searchState = 3;
            }
        }
        else if (searchState == 2) //Array
        {
            arrayStart = (p-1);
            return true;
        }
        else if (searchState == 3) //Kein Array
        {
            if ((json[p]=='"') || ((json[p]==',')&&(json[p+1]!=' ')) || (json[p]=='}')) //String Ende
            {
                retValue = label;
                return true;
            }
            else
            {
                label += json[p];
            }
        }
    }

    return false; //Nichts gefunden
}

