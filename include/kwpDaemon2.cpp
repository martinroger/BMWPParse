#pragma once
#include <kwpDaemon2.h>

bool kwp_Daemon::begin()
{
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

bool kwp_Daemon::processRXCanFrame(twai_message_t frameToProcess)
{
    return false;
}

bool kwp_Daemon::_pushToTxBuffer(twai_message_t frameToQueue)
{
    return false;
}

bool kwp_Daemon::_popTxBuffer()
{
    return false;
}

void kwp_Daemon::_ReqClearDDLI()
{
}

void kwp_Daemon::_ReqSetDDLI()
{
}

void kwp_Daemon::_ReqReadDDLI()
{
}

void kwp_Daemon::_sendFCFrame()
{
}

void kwp_Daemon::_twaiStatusWatchDog()
{
}
