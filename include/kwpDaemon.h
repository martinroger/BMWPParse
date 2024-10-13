#pragma once
#include <Arduino_Helpers.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include "kwpFrame.h"
#include "esp_log.h"

#ifndef TX_FRAME_TIMEOUT
	#define TX_FRAME_TIMEOUT 5
#endif
#ifndef RX_FRAME_TIMEOUT
	#define RX_FRAME_TIMEOUT 0
#endif


/// @brief Possible Daemon states
enum KWP_DAEMON_STATE 
{
  SLEEP,        //Starting transition, CAN status unknown/disabled. Moves to INIT when CAN traffic is observed
  INIT,         //CAN enabled, transitions out to DDLI_CLEAR after the clear requests is sent
  DDLI_CLEAR,   //DDLI clear request pending, transitions to DDLI_SETUP on positive response, back to INIT on negative response
  DDLI_SETUP,   //DDLI setup request pending, transitions to DDLI_REQUEST on positive response, back to DDLI_CLEAR after negative response (and new clear request)
  DDLI_REQUEST, //DDLI read request pending, transitions to DDLI_PARSE on positive response, back to DDLI_CLEAR after negative response (and new clear request)
  DDLI_PARSE    //After successful parsing, stays in DDLI_PARSE until data is used, otherwise back to DDLI_CLEAR in case of issues (and new clear request)
};

/// @brief 
class kwpDaemon
{
	public:
		typedef void parsedDataHandler_t();

		byte ID = 0xF1;
		byte target = 0x12;
		//Timer<millis> timeOut = 10000;
		//Timer<millis> refreshInterval = 250;
		kwpFrame rxKwpFrame;
		kwpFrame* txKwpFrame;
		KWP_DAEMON_STATE state = SLEEP;
		void sendFlowControlFrame(byte sender, byte target); //Could be private
		bool processIncomingCANFrame(CanFrame rxFrame);
		void processIncomingSID();
		void clearDDLI(byte sender, byte target);
		void setupDDLI(byte sender, byte target);
		void requestDDLI(byte sender, byte target);
		bool parseDDLI();
		void attachDebugSerial(Stream& targetSerial = Serial);
		void reset();
		void tick(bool canState);
		void attachDataHandler(parsedDataHandler_t dataHandler);
	private:
	  	Stream& _debugSerial = Serial;
		kwpFrame _clearRequest;
		kwpFrame _readDDLI;
		kwpFrame _setupDDLI;
		parsedDataHandler_t* _parsedDataHandler = nullptr;
		CanFrame _rxFrame;
};
