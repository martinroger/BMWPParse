#pragma once
#include <ESP32-TWAI-CAN.hpp>

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

enum KWP_FRAME_TYPE : byte
{
    singleFrame         =   0x00,
    firstFrame          =   0x10,
    continuationFrame   =   0x20,
    flowControlFrame    =   0x30,
    invalidFrameType    =   0xFF
};

class kwpFrame
{
public:
    byte target = 0x00;
    byte sender = 0x00;
    byte SID = 0x00;
    uint16_t length = 2;
    uint16_t payloadLength = 1;
    byte cursor = 0;
    byte payload[255];
    KWP_FRAME_TYPE frameType;

    bool RXComplete     =   true;
    bool pendingFCFrame =   false;
    bool TXComplete     =   true;

    void parseMetaData(CanFrame* rxFrame);
    KWP_FRAME_TYPE processCanFrame(CanFrame* rxFrame);
    void sendKwpFrame(bool singleShot = false,bool loopBack = false);
    void printKwpFrame(Stream& targetStream = Serial);
    void resetFrame();
    void attachDebugSerial(Stream& targetSerial);
private:
    byte seqNumber;
    //cursor could be private too
    Stream& _debugSerial = Serial;
};

