#pragma once
#include <kwpDaemon2.h>

bool kwp_Daemon::begin()
{
    //Set up some frames
    
    
    return false;
}

bool kwp_Daemon::reset()
{
    return false;
}

bool kwp_Daemon::tick()
{
    return false;
}

bool kwp_Daemon::processRXCanFrame(twai_message_t* frameToProcess)
{
    return false;
}

bool kwp_Daemon::_processRXKwpFrame(kwpFrame *frameToProcess)
{
    return false;
}

bool kwp_Daemon::_pushToTxBuffer(twai_message_t frameToQueue)
{
    return false;
}

bool kwp_Daemon::_popTxBuffer()
{

    //Set a FCFrame flag if this is a firstFrame
    if((_txBuffer[0].data[1] & 0xF0) == 0x10) _waitForFCFrame = true;

    return false;
}

bool kwp_Daemon::_ReqClearDDLI()
{
}

bool kwp_Daemon::_ReqSetDDLI()
{
}

bool kwp_Daemon::_ReqReadDDLI()
{
}

bool kwp_Daemon::_sendFCFrame()
{
}

void kwp_Daemon::_twaiStatusWatchDog()
{
}
