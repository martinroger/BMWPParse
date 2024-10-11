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
            if(!RXComplete) {
                #ifdef SERIAL_DEBUG
                    _debugSerial.println("ERROR: SingleFrame received while another frame is incomplete");
                #endif  
            }
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
            if(!RXComplete){
                #ifdef SERIAL_DEBUG
                    _debugSerial.println("ERROR: FirstFrame received while another frame is incomplete");
                #endif  
            } 
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
            else {
                #ifdef SERIAL_DEBUG
                    _debugSerial.println("ERROR: ContinuationFrame without FirstFrame");
                #endif
            } 
            break;
        case flowControlFrame:
            if(!pendingFCFrame) {
                #ifdef SERIAL_DEBUG
                    _debugSerial.println("ERROR: Unexpected FlowControl frame");
                #endif
            }
            break;
        default:
            #ifdef SERIAL_DEBUG
                _debugSerial.println("ERROR: Not a valid FrameType");
            #endif
            break;
    }
    return frameType;
}

void kwpFrame::sendKwpFrame(bool singleShot ,bool loopBack)
{
    CanFrame txFrame;
    txFrame.data_length_code = 8;
    txFrame.identifier = (0x600 + sender);
    txFrame.ss = (int)singleShot;
    txFrame.self = (int)loopBack;
    for (int i = 0; i < 8; i++)
    {
        txFrame.data[i] = 0xFF;
    }
    txFrame.data[0] = target;
    #ifdef SERIAL_DEBUG
        _debugSerial.printf("%d sendKWPFrame",__LINE__);
        _debugSerial.printf("\t pendingFCFrame: %d RXComplete: %d TXComplete: %d Length: %d PayloadLength: %d Cursor: %d \n",pendingFCFrame,RXComplete,TXComplete,length,payloadLength,cursor);
    #endif
    if(!pendingFCFrame && TXComplete) { //Not pending a FCFrame and TX is complete
        TXComplete = false;
        if (length<=6) //SingleFrame is doable
        {
            #ifdef SERIAL_DEBUG
                _debugSerial.printf("\tSingleFrame");
            #endif
            txFrame.data[1] = (byte)length;
            txFrame.data[2] = SID;
            for (int i = 0; i < payloadLength; i++)
            {
                txFrame.data[i+3] = payload[i];
            }
            ESP32Can.writeFrame(txFrame);
            TXComplete = true;
            #ifdef SERIAL_DEBUG
                _debugSerial.printf("\t SENT\n");
            #endif
        }
        else //It is a first MF
        {
            #ifdef SERIAL_DEBUG
                _debugSerial.printf("\tMultiFrame first part");
            #endif
            cursor = 0;
            seqNumber = 1;
            txFrame.data[1] = 0x10 + ((length & 0x0F00)>>8);
            txFrame.data[2] = (length & 0xFF);
            txFrame.data[3] = SID;
            for (int i = 0; i < 4; i++)
            {
                txFrame.data[4+i]=payload[i];
                cursor++;
            }
            ESP32Can.writeFrame(txFrame);
            pendingFCFrame = true;
            #ifdef SERIAL_DEBUG
                _debugSerial.printf("\t SENT\n");
            #endif
        }
    }
    else //Pending FC Frame... assumes this is in reaction to a FC frame and the transfer is incomplete
    {
        #ifdef SERIAL_DEBUG
            _debugSerial.printf("\tStarting after Continuation Frame\n");
        #endif
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
                    txFrame.data[2+i] = 0xFF;
                }
            }
            ESP32Can.writeFrame(txFrame,0);
            #ifdef SERIAL_DEBUG
                _debugSerial.printf("\t\tSENT with seq number 0x%02X\n",seqNumber);
            #endif
            seqNumber++;
        }  
        pendingFCFrame = false;
    }
}

void kwpFrame::printKwpFrame(Stream& targetStream)
{
    
    if(RXComplete && (frameType != invalidFrameType && frameType != flowControlFrame)) {
        targetStream.printf("%02X | Sender: %02X Target: %02X SID: %02X PayloadLength: %3d |  ",frameType,sender,target,SID,payloadLength); 
        for (int i = 0; i < payloadLength; i++)
        {
        targetStream.printf("%02X ",payload[i]);
        }
        targetStream.println();
    }
    else if (frameType == flowControlFrame)
    {
        targetStream.printf("FC TX: %02X RX: %02X\n",sender,target);
    }
}

void kwpFrame::resetFrame()
{
    target = 0x00;
    sender = 0x00;
    SID = 0x00;
    length = 2;
    payloadLength = 1;
    cursor = 0;
    RXComplete     =   true;
    pendingFCFrame =   false;
    TXComplete     =   true;
}

void kwpFrame::attachDebugSerial(Stream &targetSerial)
{
    _debugSerial = targetSerial;
}
