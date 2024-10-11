#pragma once
#include <Arduino_Helpers.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include "kwpFrame.h"

enum KWP_DAEMON_STATE 
{
  INIT, //Starting state, will attempt to go clear the DDLI
  DDLI_CLEAR, //Request to clear the DDLI is sent out, waiting for Positive Response
  DDLI_SETUP, //Positive clear, request to setup the DDLI and wait
  DDLI_REQUEST, //DDLI setup successfully, read DDLI
  DDLI_PARSE //Positive response and DDLI received, parse and revert to DDLI_REQUEST
};

class kwpDaemon
{
    public:
        byte ID = 0xF1;
        byte target = 0x12;
        Timer<millis> timeOut = 15000;
        Timer<millis> refreshInterval = 100;
        kwpFrame rxKwpFrame;
        kwpFrame txKwpFrame;
        KWP_DAEMON_STATE state = INIT;
        void sendFlowControlFrame(byte sender, byte target); //Could be private
        bool processIncomingCANFrame(CanFrame rxFrame);
        void attachDebugSerial(Stream& targetSerial = Serial);
        void reset();
        void tick();
    private:
        Stream& _debugSerial = Serial;
};
