/* framehandler.cpp
 *
 * Arduino library to read from Victron devices using VE.Direct protocol.
 * Derived from Victron framehandler reference implementation.
 *
 * The MIT License
 *
 * Copyright (c) 2019 Victron Energy BV
 * Portions Copyright (C) 2020 Chris Terwilliger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * 2020.05.05 - 0.2 - initial release
 * 2020.06.21 - 0.2 - add MIT license, no code changes
 * 2020.08.20 - 0.3 - corrected #include reference
 *
 */

#include "devices/VeDirectFrameHandler.h"

#define MODULE "VE.Frame"	// Victron seems to use this to find out where logging messages were generated
#define MAX_HEX_CALLBACK 10	// initial number - buffer is dynamically increased if necessary

// The name of the record that contains the checksum.
static constexpr char checksumTagName[] = "CHECKSUM";

VeDirectFrameHandler::VeDirectFrameHandler() :
        veName(),
	veValue(),
	frameIndex(0),
	veEnd(0),
	veHEnd(0),
	//mStop(false),	// don't know what Victron uses this for, not using
	logEF(0),
	mState(IDLE),
	mChecksum(0),
	mTextPointer(0),
	tempName(),
	tempValue(),
	veHexCBList(0),
	veCBEnd(0),
	maxCB(MAX_HEX_CALLBACK)
{
}

VeDirectFrameHandler::~VeDirectFrameHandler() {
  	if (veHexCBList) delete veHexCBList;
}

/*
 *	rxData
 *  This function is called by the application which passes a byte of serial data
 *  It is unchanged from Victron's example code
 */
void VeDirectFrameHandler::rxData(uint8_t inbyte)
{
	//if (mStop) return;
	if ( (inbyte == ':') && (mState != CHECKSUM) ) {
	  	vePushedState = mState; //hex frame can interrupt TEXT
		mState = RECORD_HEX;
		veHEnd = 0;
	}
	if (mState != RECORD_HEX) {
		mChecksum += inbyte;
	}
	inbyte = toupper(inbyte);

	switch(mState) {
	case IDLE:
		/* wait for \n of the start of an record */
		switch(inbyte) {
		case '\n':
			mState = RECORD_BEGIN;
			break;
		case '\r': /* Skip */
		default:
			break;
		}
		break;
	case RECORD_BEGIN:
		mTextPointer = mName;
		*mTextPointer++ = inbyte;
		mState = RECORD_NAME;
		break;
	case RECORD_NAME:
		// The record name is being received, terminated by a \t
		switch(inbyte) {
		case '\t':
			// the Checksum record indicates a EOR
			if ( mTextPointer < (mName + sizeof(mName)) ) {
				*mTextPointer = 0; /* Zero terminate */
				if (strcmp(mName, checksumTagName) == 0) {
					mState = CHECKSUM;
					break;
				}
			}
			mTextPointer = mValue; /* Reset value pointer */
			mState = RECORD_VALUE;
			break;
		default:
			// add byte to name, but do no overflow
			if ( mTextPointer < (mName + sizeof(mName)) )
				*mTextPointer++ = inbyte;
			break;
		}
		break;
	case RECORD_VALUE:
		// The record value is being received.  The \r indicates a new record.
		switch(inbyte) {
		case '\n':
			// forward record, only if it could be stored completely
			if ( mTextPointer < (mValue + sizeof(mValue)) ) {
				*mTextPointer = 0; // make zero ended
				textRxEvent(mName, mValue);
			}
			mState = RECORD_BEGIN;
			break;
		case '\r': /* Skip */
			break;
		default:
			// add byte to value, but do no overflow
			if ( mTextPointer < (mValue + sizeof(mValue)) )
				*mTextPointer++ = inbyte;
			break;
		}
		break;
	case CHECKSUM:
	{
		bool valid = mChecksum == 0;
		if (!valid)
			logE(MODULE,"[CHECKSUM] Invalid frame");
		mChecksum = 0;
		mState = IDLE;
		frameEndEvent(valid);
		break;
	}
	case RECORD_HEX:
	        mState = hexRxEvent(inbyte);
		break;
	}
}

/*
 * textRxEvent
 * This function is called every time a new name/value is successfully parsed.  It writes the values to the temporary buffer.
 */
void VeDirectFrameHandler::textRxEvent(char * mName, char * mValue) {
    strcpy(tempName[frameIndex], mName);    // copy name to temporary buffer
    strcpy(tempValue[frameIndex], mValue);  // copy value to temporary buffer
	frameIndex++;
}

/*
 *	frameEndEvent
 *  This function is called at the end of the received frame.  If the checksum is valid, the temp buffer is read line by line.
 *  If the name exists in the public buffer, the new value is copied to the public buffer.	If not, a new name/value entry
 *  is created in the public buffer.
 */
void VeDirectFrameHandler::frameEndEvent(bool valid) {
	if ( valid ) {
		for ( int i = 0; i < frameIndex; i++ ) {				// read each name already in the temp buffer
			bool nameExists = false;
			for ( int j = 0; j <= veEnd; j++ ) {				// compare to existing names in the public buffer
				if ( strcmp(tempName[i], veName[j]) == 0 ) {
					strcpy(veValue[j], tempValue[i]);			// overwrite tempValue in the public buffer
					nameExists = true;
					break;
				}
			}
			if ( !nameExists ) {
				strcpy(veName[veEnd], tempName[i]);				// write new Name to public buffer
				strcpy(veValue[veEnd], tempValue[i]);			// write new Value to public buffer
				veEnd++;										// increment end of public buffer
				if ( veEnd >= buffLen ) {						// stop any buffer overrun
					veEnd = buffLen - 1;
				}
			}
		}
	}
	frameIndex = 0;	// reset frame
}

/*
 *	logE
 *  This function included for continuity and possible future use.
 */
void VeDirectFrameHandler::logE(const char * module, const char * error) {
	//Serial.print("MODULE: ");
    //Serial.println(module);
    //Serial.print("ERROR: ");
    //Serial.println(error);
  if (logEF)
    (*logEF)(module,error);
}

/*
 *	addHexCallback
 *  This function record a new callback for hex frames
 */
void VeDirectFrameHandler::addHexCallback(hexCallback cb, void* data) {
  if (veHexCBList==0) { // first time, allocate callbacks buffer
    veHexCBList=new VeHexCB[maxCB];
    veCBEnd=0;
  }
  else if (veCBEnd==maxCB) { // we need to resize the callbacks buffer, we double the max size
    int newMax=maxCB*2;
    VeHexCB* tmpb=new VeHexCB[newMax];
    memcpy(tmpb, veHexCBList, maxCB*sizeof(VeHexCB));
    maxCB=newMax;
    delete veHexCBList;
    veHexCBList=tmpb;
  }
  veHexCBList[veCBEnd].cb=cb;
  veHexCBList[veCBEnd].data=data;
  veCBEnd++;
}

/*
 *	hexIsValid
 *  This function compute checksum and validate hex frame
 */
#define ascii2hex(v) (v-48-(v>='A'?7:0))
#define hex2byte(b) (ascii2hex(*(b)))*16+((ascii2hex(*(b+1))))
static bool hexIsValid(const char* buffer, int size) {
  uint8_t checksum=0x55-ascii2hex(buffer[1]);
  for (int i=2; i<size; i+=2) checksum -= hex2byte(buffer+i);
  return (checksum==0);
}

/*
 *	hexRxEvent
 *  This function records hex answers or async messages
 */
int VeDirectFrameHandler::hexRxEvent(uint8_t inbyte) {
  int ret=RECORD_HEX; // default - continue recording until end of frame

  switch (inbyte) {
  case '\n':
    // message ready - call all callbacks
    if (hexIsValid(veHexBuffer,veHEnd)) {
      for(int i=0; i<veCBEnd; i++) {
	(*(veHexCBList[i].cb))(veHexBuffer, veHEnd, veHexCBList[i].data);
      }
    }
    else {
      logE(MODULE,"[CHECKSUM] Invalid hex frame");
    }
    // restore previous state
    ret=vePushedState;

  default:
    veHexBuffer[veHEnd++]=inbyte;
    if (veHEnd>=hexBuffLen) { // oops -buffer overflow - something went wrong, we abort
      logE(MODULE,"hexRx buffer overflow - aborting read");
      veHEnd=0;
      ret=IDLE;
    }
  }

  return ret;
}
