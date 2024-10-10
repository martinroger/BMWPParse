#include "kwpFrame.h"

void kwpFrame::parseMetaData(CanFrame* rxFrame)
{
    sender = rxFrame->identifier & 0xFF;
    target = rxFrame->data[0];
    byte checkType = (rxFrame->data[1] & 0xF0);
    if (checkType > 0x30 ) {
        frameType = invalidFrameType;
    }
    else {
        frameType = (KWP_FRAME_TYPE)checkType;
    }
}

KWP_FRAME_TYPE kwpFrame::processCanFrame(CanFrame *rxFrame)
{
    parseMetaData(rxFrame);
    
    switch(frameType) {
        case singleFrame:
            if(!RXComplete) Serial.println("ERROR: SingleFrame received while another frame is incomplete");
            else {
                length          = rxFrame->data[1];
                payloadLength   = length-1;
                SID             = rxFrame->data[2];
                cursor          = 0;

                for(int i=3;i<3+payloadLength;i++) {
                    payload[cursor] = rxFrame->data[i];
                    cursor++;
                } 
            }
            break;
        case firstFrame:
            if(!RXComplete) Serial.println("ERROR: FirstFrame received while another frame is incomplete");
            else {
                length          = (swap_endian<uint16_t>(*((uint16_t*)&(rxFrame->data[1]))) & 0x0FFF);
                payloadLength   = length -1;
                SID             = rxFrame->data[3];
                RXComplete      = false;
                cursor          = 0;

                for(int i = 4; i < 8;i++) {
                    payload[cursor] =   rxFrame->data[i];
                    cursor++;
                }
            }
            break;
        case continuationFrame:
            if(!RXComplete) {
                for(int i = 2; i<8; i++) {
                    if(cursor<payloadLength) {
                        payload[cursor] = rxFrame->data[i];
                        cursor++;
                    }
                    else {
                        RXComplete = true;
                        break;
                    }
                }
            }
            else Serial.println("ERROR: ContinuationFrame without FirstFrame");
            break;
        case flowControlFrame:
            break;
        default:
            Serial.println("ERROR: Not a valid FrameType");
            break;
    }
    return frameType;
}

void kwpFrame::sendKwpFrame()
{
}

void kwpFrame::printKwpFrame()
{
    if(frameType != invalidFrameType || frameType != flowControlFrame || ((frameType == continuationFrame) && !RXComplete)) {
        Serial.printf("%02X | Sender: %02X Target: %02X SID: %02X PayloadLength: %3d |  ",frameType,sender,target,SID,payloadLength); 
        for (int i = 0; i < payloadLength; i++)
        {
        Serial.printf("%02X ",payload[i]);
        }
        Serial.println();
    }
    else if (frameType == flowControlFrame)
    {
        Serial.printf("FC TX: %02X RX: %02X\n",sender,target);
    }
}
