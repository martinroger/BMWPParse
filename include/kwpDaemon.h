#pragma once
#include <Arduino_Helpers.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include "kwpFrame.h"

/// @brief Possible Daemon states
enum KWP_DAEMON_STATE 
{
  SLEEP,        //Starting transition, CAN status unknown/disabled. Moves to INIT when CAN traffic is observed
  INIT,         //CAN enabled, transitions out to DDLI_CLEAR after the clear requests is sent
  DDLI_CLEAR,   //DDLI clear request pending, transitions to DDLI_SETUP on positive response, back to INIT on negative response
  DDLI_SETUP,   //DDLI setup request pending, transitions to DDLI_REQUEST on positive response, back to DDLI_CLEAR after negative response (and new clear request)
  DDLI_REQUEST, //DDLI read request pending, transitions to DDLI_PARSE on positive response, back to DDLI_CLEAR after negative response (and new clear request)
  DDLI_PARSE    //Moves back to DDLI_REQUEST after successful parsing of DDLI data, otherwise back to DDLI_CLEAR in case of issues (and new clear request)
};

/// @brief 
class kwpDaemon
{
    public:
        byte ID = 0xF1;
        byte target = 0x12;
        Timer<millis> timeOut = 15000;
        Timer<millis> refreshInterval = 100;
        kwpFrame rxKwpFrame;
        kwpFrame txKwpFrame;
        KWP_DAEMON_STATE state = SLEEP;
        void sendFlowControlFrame(byte sender, byte target); //Could be private
        bool processIncomingCANFrame(CanFrame rxFrame);
        void attachDebugSerial(Stream& targetSerial = Serial);
        void reset();
        void tick();
    private:
        Stream& _debugSerial = Serial;
};
