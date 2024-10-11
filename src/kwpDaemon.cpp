#include "kwpDaemon.h"

void kwpDaemon::sendFlowControlFrame(byte sender, byte target)
{
    CanFrame FCFrame = {0};
    FCFrame.identifier = 0x600 + sender;
    FCFrame.data_length_code = 8;
    FCFrame.self=0;
    FCFrame.ss = 0;
    FCFrame.data[0] = target;
    FCFrame.data[1] = 0x30;

    ESP32Can.writeFrame(FCFrame);
    #ifdef SERIAL_DEBUG
        _debugSerial.printf("\tFC frame sent from 0x6%02X to 0x6%02X\n",sender,target);
    #endif
}

bool kwpDaemon::processIncomingCANFrame(CanFrame rxFrame)
{
    bool ret = false;
    if(((rxFrame.identifier & 0xF00) == (0x600)) && (rxFrame.data[0]==ID)) //If this message is targeted at daemon
    {  
        rxKwpFrame.processCanFrame(&rxFrame);
        switch (rxKwpFrame.frameType) //Choose response based on the incoming frameType
        {
        case singleFrame:
        #ifdef SERIAL_DEBUG
            rxKwpFrame.printKwpFrame();
        #endif
        //Do something based on the SID
        ret = true;
        break;
        case flowControlFrame:
        #ifdef SERIAL_DEBUG
            rxKwpFrame.printKwpFrame();
        #endif
        if(txKwpFrame.pendingFCFrame) txKwpFrame.sendKwpFrame();
        ret = true;
        break;
        case continuationFrame:
        if(rxKwpFrame.RXComplete) {
            //Do something based on the SID
            #ifdef SERIAL_DEBUG
            rxKwpFrame.printKwpFrame();
            #endif
        }
        ret = true;
        break;
        case firstFrame:
        //Reply with the Flow Control frame
        sendFlowControlFrame(ID,rxKwpFrame.sender);
        ret = true;
        break;
        default:
            #ifdef SERIAL_DEBUG
            Serial.printf("%d\t Invalid FrameType\n",__LINE__);
            #endif
            ret = false;
        break;
        }
    }
    if(ret) timeOut.beginNextPeriod(); //Reset the timer
    return ret;
}

void kwpDaemon::attachDebugSerial(Stream &targetSerial)
{
    _debugSerial = targetSerial;
}

void kwpDaemon::reset()
{
    state = INIT;
    rxKwpFrame.resetFrame();
    rxKwpFrame.resetFrame();
}

void kwpDaemon::tick()
{
    //Do the laundry
}
