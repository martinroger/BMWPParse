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

	ESP32Can.writeFrame(FCFrame,5);
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tFC frame sent from 0x6%02X to 0x6%02X\n",__LINE__,sender,target);
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
		#ifdef SERIAL_DEBUG
			_debugSerial.printf("Ln%D\tFrame received for Daemon\n",__LINE__);
		#endif
		rxKwpFrame.processCanFrame(&rxFrame);
		switch (rxKwpFrame.frameType) //Choose response based on the incoming frameType
		{
			case singleFrame:
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\tType: Single Frame\n",__LINE__);
					rxKwpFrame.printKwpFrame(_debugSerial);
				#endif
				processIncomingSID();
				ret = true;
				break;
			case flowControlFrame:
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\tType: Flow Control Frame\n",__LINE__);
					rxKwpFrame.printKwpFrame(_debugSerial);
				#endif
				if(txKwpFrame->pendingFCFrame) txKwpFrame->sendKwpFrame();
				ret = true;
				break;
			case continuationFrame:
				if(rxKwpFrame.RXComplete) {
					#ifdef SERIAL_DEBUG
						_debugSerial.printf("Ln%D\tType: Continuation Frame (complete)\n",__LINE__);
						rxKwpFrame.printKwpFrame(_debugSerial);
					#endif
					processIncomingSID();
				}
				ret = true;
				break;
			case firstFrame:
				#ifdef SERIAL_DEBUG
						_debugSerial.printf("Ln%D\tType: First Frame\n",__LINE__);
				#endif
				sendFlowControlFrame(ID,rxKwpFrame.sender);
				ret = true;
				break;
			default:
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\tInvalid FrameType\n",__LINE__);
				#endif
				ret = false;
				break;
		}
	}
	//if(ret) timeOut.beginNextPeriod(); //Reset the timer
	return ret;
}

/// @brief Process incoming response SIDs and perform related state changes
void kwpDaemon::processIncomingSID()
{
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tProcessing SID\n",__LINE__);
	#endif
	switch (rxKwpFrame.SID)
	{
	case 0x6C: //dynamicallyDefineLocalIdentifier
		if(rxKwpFrame.payload[0]==0xF0) {//Positive response for DDLI F0
			#ifdef SERIAL_DEBUG
				_debugSerial.printf("Ln%D\t0x%02X\tPositive response to DDLI\n",__LINE__,rxKwpFrame.SID);
			#endif
			KWP_DAEMON_STATE prevState = state;
			switch (prevState)
			{
			case DDLI_CLEAR:
				setupDDLI(ID,target);
				state = DDLI_SETUP;
				//timeOut.beginNextPeriod();
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\t\tTransition from DDLI_CLEAR to DDLI_SETUP\n",__LINE__);
				#endif
				break;
			case DDLI_SETUP:
				requestDDLI(ID,target);
				state = DDLI_REQUEST;
				//timeOut.beginNextPeriod();
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\t\tTransition from DDLI_SETUP to DDLI_REQUEST\n",__LINE__);
				#endif
				break;
			default:
				//state = INIT;
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\t\tUnexpected Daemon state: %d\n",__LINE__,state);
				#endif
				break;
			}
		}
		break;
	case 0x61: //readDataByCommonIdentifier
		#ifdef SERIAL_DEBUG
			_debugSerial.printf("Ln%D\t0x%02X\tPositive response to readDataByCommonIdentifier\n",__LINE__,rxKwpFrame.SID);
		#endif
		if(rxKwpFrame.payload[0]==0xF0 && state == DDLI_REQUEST) {
			if(parseDDLI()) {
				state = DDLI_PARSE; //Revert back to Request when successfully parsing
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\t\tTransition from DDLI_REQUEST to DDLI_PARSE\n",__LINE__);
				#endif
			}
			else {
				clearDDLI(ID,target); //Or try to clear
				state = DDLI_CLEAR;
				#ifdef SERIAL_DEBUG
					_debugSerial.printf("Ln%D\t\tTransition from DDLI_REQUEST to DDLI_CLEAR\n",__LINE__);
				#endif
			}
			//timeOut.beginNextPeriod();
		}
		break;
	case 0x7F: //Negative Response
		#ifdef SERIAL_DEBUG
			_debugSerial.printf("Ln%D\tNegative Response to SID 0x%02X\n",__LINE__,rxKwpFrame.payload[0]);
		#endif
		//reset();
		break;
	default:
		#ifdef SERIAL_DEBUG
			_debugSerial.printf("Ln%D\tUnrecognized SID 0x%02X\n",__LINE__,rxKwpFrame.SID);
		#endif
		break;
	}
}

/// @brief Sends a Clear DDLI kwpFrame for DDLI F0
/// @param sender Sender of the request
/// @param target Intended target of the request
void kwpDaemon::clearDDLI(byte sender, byte target)
{
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tSending DDLI Clear...\n",__LINE__);    
	#endif
	_clearRequest.target = target;
	_clearRequest.sender = sender;
	_clearRequest.SID = 0x2C;
	_clearRequest.payloadLength = 2;
	_clearRequest.length = _clearRequest.payloadLength + 1;
	_clearRequest.payload[0] = 0xF0;
	_clearRequest.payload[1] = 0x04;
	_clearRequest.sendKwpFrame(); //SF expected
	txKwpFrame = &_clearRequest; //Just in case
}

/// @brief Sends a DDLI setup frame for DDLI F0
/// @param sender Sender of the request
/// @param target Intended target of the request
void kwpDaemon::setupDDLI(byte sender, byte target)
{
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tSetting up DDLI...\n",__LINE__);    
	#endif
	_setupDDLI.target = target;
	_setupDDLI.sender = sender;
	_setupDDLI.SID = 0x2C;
	_setupDDLI.payloadLength = 55;
	_setupDDLI.length = _setupDDLI.payloadLength + 1;
	byte tempPayload[/*_setupDDLI.payloadLength*/] = {
		0xF0,
		0x02, 0x01, 0x01, 0x58, 0x0C, 0x01, 
		0x02, 0x02, 0x02, 0x58, 0xF0, 0x01,
		0x02, 0x04, 0x02, 0x5A, 0xBC, 0x01,
		0x02, 0x06, 0x02, 0x58, 0xDD, 0x01,
		0x02, 0x08, 0x01, 0x58, 0x0D, 0x01,
		0x02, 0x09, 0x01, 0x44, 0x02, 0x01, 
		0x02, 0x0A, 0x01, 0x58, 0x1F, 0x01,
		0x02, 0x0B, 0x01, 0x58, 0x05, 0x01,
		0x02, 0x0C, 0x01, 0x58, 0x0F, 0x01 
	};
	for (int i = 0; i < _setupDDLI.payloadLength; i++)
	{
		_setupDDLI.payload[i] = tempPayload[i];
	}
	_setupDDLI.sendKwpFrame(); //MF expected
	txKwpFrame = &_setupDDLI; 
}

/// @brief Sends a read request for DDLI F0
/// @param sender Sender of the request
/// @param target Intended target of the request
void kwpDaemon::requestDDLI(byte sender, byte target)
{
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tRequesting DDLI Read...\n",__LINE__);     
	#endif
	_readDDLI.target = target;
	_readDDLI.sender = sender;
	_readDDLI.SID = 0x21;
	_readDDLI.payloadLength = 1;
	_readDDLI.length = _readDDLI.payloadLength + 1;
	_readDDLI.payload[0] = 0xF0;
	_readDDLI.sendKwpFrame(); //SF expected
	txKwpFrame = &_readDDLI; //Just in case
}

/// @brief Processes the DDLI frame into a DDLI structure
/// @return True in absence of error, false otherwise
bool kwpDaemon::parseDDLI()
{
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tParsing DDLI...",__LINE__);     
	#endif
	bool ret = false;
	if(rxKwpFrame.payloadLength==13) ret = true;
	#ifdef SERIAL_DEBUG
		if(ret)	_debugSerial.printf("Success\n");     
		else _debugSerial.printf("Fail\n");
	#endif
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
	#ifdef SERIAL_DEBUG
		_debugSerial.printf("Ln%D\tResetting daemon...\n",__LINE__);     
	#endif
	state = SLEEP;
	rxKwpFrame.resetFrame();
	txKwpFrame = &_clearRequest;
	_clearRequest.resetFrame();
	_readDDLI.resetFrame();
	_setupDDLI.resetFrame();
	//timeOut.beginNextPeriod();
}

/// @brief Ticker of the daemon. Can be called systematically, will operate some state changes
/// @param canState Status of the CAN connection (true is up, false is down)
void kwpDaemon::tick(bool canState)
{
		KWP_DAEMON_STATE previousState = state;
		switch (previousState)
		{
		case SLEEP:
			if(canState) state = INIT;
			#ifdef SERIAL_DEBUG
				_debugSerial.printf("Ln%D\tTICK transition from SLEEP to INIT\n",__LINE__);
			#endif
			//timeOut.beginNextPeriod();
			break;
		case INIT:
			#ifdef SERIAL_DEBUG
				_debugSerial.printf("Ln%D\tTICK transition from INIT to DDLI_CLEAR\n",__LINE__);
			#endif
			clearDDLI(ID,target);
			state = DDLI_CLEAR;
			break;
		case DDLI_CLEAR:
			// if(timeOut) {
			// 	state = INIT;
			// 	//timeOut.beginNextPeriod();
			// 	#ifdef SERIAL_DEBUG
			// 		_debugSerial.printf("Ln%D\tTICK TIMEOUT from DDLI_CLEAR to INIT\n",__LINE__);
			// 	#endif
			// }
			clearDDLI(ID,target);
			state = DDLI_CLEAR;
			break;
		case DDLI_SETUP:
			// if(timeOut) {
			// 	clearDDLI(ID,target);
			// 	state = DDLI_CLEAR;
			// 	//timeOut.beginNextPeriod();
			// 	#ifdef SERIAL_DEBUG
			// 		_debugSerial.printf("Ln%D\tTICK TIMEOUT from DDLI_SETUP to DDLI_CLEAR\n",__LINE__);
			// 	#endif
			// }
			break;
		case DDLI_REQUEST:
			// if(timeOut) {
			// 	clearDDLI(ID,target);
			// 	state = DDLI_CLEAR;
			// 	//timeOut.beginNextPeriod();
			// 	#ifdef SERIAL_DEBUG
			// 		_debugSerial.printf("Ln%D\tTICK TIMEOUT from DDLI_REQUEST to DDLI_CLEAR\n",__LINE__);
			// 	#endif
			// }
			break;
		case DDLI_PARSE:
			#ifdef SERIAL_DEBUG
				_debugSerial.printf("Ln%D\tTICK transition from DDLI_PARSE to DDLI_REQUEST\n",__LINE__);
			#endif
			//Callback to handover the data
			if(_parsedDataHandler) _parsedDataHandler();
			requestDDLI(ID,target);
			state = DDLI_REQUEST;
			break;
		}
	
	if(!canState) state = SLEEP;
}

/// @brief Callback definition to update external variables based on the DDLI array
/// @param dataHandler Pointer to the callback
void kwpDaemon::attachDataHandler(parsedDataHandler_t dataHandler)
{
	_parsedDataHandler = dataHandler;
}
