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
    CanFrame txFrame;
    txFrame.data_length_code = 8;
    txFrame.identifier = (0x600 + sender);
    for (int i = 0; i < 8; i++)
    {
        txFrame.data[i] = 0xAA;
    }
    txFrame.data[0] = target;
    Serial.printf("\t txFrame ready\n");
    Serial.printf("\t pendingFCFrame: %d RXComplete: %d TXComplete: %d Length: %d PayloadLength: %d Cursor: %d \n",pendingFCFrame,RXComplete,TXComplete,length,payloadLength,cursor);

    if(!pendingFCFrame && TXComplete) { //Not pending a FCFrame and TX is complete
        TXComplete = false;
        if (length<=6) //SingleFrame is doable
        {
            Serial.printf("\tSingleFrame");
            txFrame.data[1] = (byte)length;
            txFrame.data[2] = SID;
            for (int i = 0; i < payloadLength; i++)
            {
                txFrame.data[i+3] = payload[i];
            }
            ESP32Can.writeFrame(txFrame);
            Serial.printf("\t SENT\n");
            TXComplete = true;
        }
        else //It is a first MF
        {
            Serial.printf("\tMultiFrame first part");
            cursor = 0;
            seqNumber = 1;
            //length          = (swap_endian<uint16_t>(*((uint16_t*)&(rxFrame->data[1]))) & 0x0FFF);
            //*((uint16_t*)&(txFrame.data[1])) = swap_endian<uint16_t>(length) + 0x1000;
            txFrame.data[1] = 0x10 + ((length & 0x0F00)>>8);
            txFrame.data[2] = (length & 0xFF);
            txFrame.data[3] = SID;
            for (int i = 0; i < 4; i++)
            {
                txFrame.data[4+i]=payload[i];
                cursor++;
            }
            ESP32Can.writeFrame(txFrame);
            Serial.printf("\t SENT\n");
            pendingFCFrame = true;

        }
    }
    else //Pending FC Frame... assumes this is in reaction to a FC frame and the transfer is incomplete
    {
        Serial.printf("\tStarting after Continuation Frame");
        while(!TXComplete) {
            txFrame.data[1] = 0x20 + seqNumber;
            for (int i = 0; i < 6; i++)
            {
                if(cursor<payloadLength) {
                    txFrame.data[2+i] = payload[cursor];
                    cursor++;
                }
                else
                {
                    TXComplete = true;
                    break;
                }
            }
            ESP32Can.writeFrame(txFrame,0);
            Serial.printf("\t SENT with seq number 0x%x\n",seqNumber);
            seqNumber++;
        }  
        pendingFCFrame = false;
    }
}

void kwpFrame::printKwpFrame()
{
    
    if(RXComplete && (frameType != invalidFrameType && frameType != flowControlFrame)) {
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
