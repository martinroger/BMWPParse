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
    void sendKwpFrame();
    void printKwpFrame();
private:
    byte seqNumber;
};

kwpFrame exampleFrame;

void buildExampleFrame() {
    exampleFrame.target = 0x12;
    exampleFrame.sender = 0xF1;
    exampleFrame.SID    = 0x2C;
    exampleFrame.length = 56;
    exampleFrame.payloadLength = 55;
    exampleFrame.cursor = 0;
    byte tempPayload[] = { 0xF0, 0x02, 0x01, 0x01, 0x58, 0x0C, 0x01, 0x02, 0x02, 0x02, 0x58, 0xF0, 0x01, 0x02, 0x04, 0x02, 0x5A, 0xBC, 0x01, 0x02, 0x06, 0x02, 0x58, 0xDD, 0x01, 0x02, 0x08, 0x01, 0x58, 0x0D, 0x01, 0x02, 0x09, 0x01, 0x44, 0x02, 0x01, 0x02, 0x0A, 0x01, 0x58, 0x1F, 0x01, 0x02, 0x0B, 0x01, 0x58, 0x05, 0x01, 0x02, 0x0C, 0x01, 0x58, 0x0F, 0x01 };
    for (int i = 0; i < 55; i++)
    {
        exampleFrame.payload[i] = tempPayload[i];
    }
    
}