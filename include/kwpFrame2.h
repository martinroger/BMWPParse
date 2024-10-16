#pragma once
#include <Arduino.h>
#include "driver/twai.h"

#pragma region DEFINES
#ifndef ECU_ID
    #define ECU_ID 0xF1
#endif
#ifndef TARGET_ID
    #define TARGET_ID 0x12
#endif
#pragma endregion

#pragma region UTILS
template <typename T>
T swap_endian(T u)
{
	static_assert (CHAR_BIT == 8, "CHAR_BIT != 8");

	union
	{
		T u;
		unsigned char u8[sizeof(T)];
	} source, dest;

	source.u = u;

	for (size_t k = 0; k < sizeof(T); k++)
		dest.u8[k] = source.u8[sizeof(T) - k - 1];

	return dest.u;
}
#pragma endregion

#pragma region ENUMS
enum KWP_FRAME_TYPE : byte
{
	singleFrame         =   0x00,
	firstFrame          =   0x10,
	continuationFrame   =   0x20,
	flowControlFrame    =   0x30,
	invalidFrameType    =   0xFF
};

#pragma endregion

#pragma region CLASS DEFINITION


class kwpFrame
{
    public:
        byte target                 =   TARGET_ID;          //Usually the first CAN data byte
        byte sender                 =   ECU_ID;             //Usually the last byte of the CAN frame identifier
        byte SID                    =   0x00;               //Determined only once in RX, set in TX
        uint16_t length             =   2;                  //Length of payload + SID byte
        uint16_t bufferLength       =   1;                  //Length of useful payload after the SID
        byte cursor                 =   0;                  //Indicate currently writable byte in payload Buffer. Always less than bufferLength
        byte buffer[255]            =   {0x00};             //255 bytes RX or TX buffer, indexable with cursor
        bool rxComplete             =   true;               //Indicates if the frame is completely received or not. Useful only in RX
        bool multiFrame             =   false;              //MultiCAN frame (or not) for both TX and RX

        void setMetadaData(twai_message_t* canMetaFrame);
        void calculateMetaData();
        bool appendCanFrameBuffer(twai_message_t* canFrame, uint8_t startPos = 2, uint8_t endPos = 7);
        void reset(byte _target = TARGET_ID, byte _sender = ECU_ID);
};


#pragma endregion