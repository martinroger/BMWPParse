#include "kwpDaemon.h"

/// @brief Utility sender that broadcasts a simple FlowControl frame at the attention of a target
/// @param sender Byte ID of the sender of the FlowControl frame (based on 0x600 + sender)
/// @param target Byte ID of the receiver waiting for the FlowControl frame (based on 0x600 + target)
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

/// @brief Main reaction-based processor of incoming CANFrame. Uses SID-based logic for follow-up actions
/// @param rxFrame Incoming CAN Frame to be examined and reacted to by the daemon.
/// @return True if rxFrame is successfully processed into a KWPFrame, false otherwise.
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

/// @brief Attaches a target Serial for debug messages
/// @param targetSerial Stream (usually a Serial) on which to broadcast debug messages.
void kwpDaemon::attachDebugSerial(Stream &targetSerial)
{
    _debugSerial = targetSerial;
}

/// @brief Used to completely reset the daemon and its frame to a SLEEP state
void kwpDaemon::reset()
{
    state = SLEEP;
    rxKwpFrame.resetFrame();
    rxKwpFrame.resetFrame();
}

/// @brief Ticker for possible state transitions at regular intervals
void kwpDaemon::tick()
{
    //Do the laundry
}
