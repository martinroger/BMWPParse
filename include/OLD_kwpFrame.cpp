#include "OLD_kwpFrame.h"

/// @brief Parse essential metadata (target, sender, type) from a candidate CANFrame
/// @param rxFrame Candidate CAN Frame that might be a KWP Frame
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
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\tFrame type parsed: %X\n",__LINE__,frameType);
	#endif
}

/// @brief 
/// @param rxFrame 
/// @return 
KWP_FRAME_TYPE kwpFrame::processCanFrame(CanFrame *rxFrame)
{
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\tProcessing CAN Frame to KWP Frame\n",__LINE__);
	#endif
	parseMetaData(rxFrame);
	
	switch(frameType) {
		case singleFrame:
			if(!RXComplete) {
				#ifdef KWP_FRAME_DEBUG
					_debugSerial.printf("Ln%D\tERROR: SingleFrame received while another frame is incomplete\n",__LINE__);
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
				#ifdef KWP_FRAME_DEBUG
					_debugSerial.printf("Ln%D\tERROR: FirstFrame received while another frame is incomplete\n",__LINE__);
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
				#ifdef KWP_FRAME_DEBUG
					_debugSerial.printf("Ln%D\tERROR: ContinuationFrame without FirstFrame\n",__LINE__);
				#endif
			} 
			break;
		case flowControlFrame:
			//Nothing to do, we are in the wrong scope for that
			break;
		default:
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("Ln%D\tERROR: Not a valid FrameType\n");
			#endif
			break;
	}
	return frameType;
}

/// @brief 
/// @param singleShot When true, the frame broadcast will only be attempted once. False by default.
/// @param loopBack When true, the CAN frames that are sent will be detected on the RX side. False by default.
void kwpFrame::sendKwpFrame(bool singleShot ,bool loopBack)
{
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\tSending KWP Frame ...\n",__LINE__);
	#endif
	
	CanFrame txFrame;
	txFrame.data_length_code = 8;
	txFrame.identifier = (0x600 + sender);
	txFrame.ss = (int)singleShot;
	txFrame.self = (int)loopBack;
	for (int i = 0; i < 8; i++)
	{
		txFrame.data[i] = 0xAA;
	}
	txFrame.data[0] = target;
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\t\tSTART : pendingFCFrame: %d RXComplete: %d TXComplete: %d Length: %d PayloadLength: %d Cursor: %d \n",__LINE__,pendingFCFrame,RXComplete,TXComplete,length,payloadLength,cursor);
	#endif
	if(!pendingFCFrame && TXComplete) { //Not pending a FCFrame and TX is complete
		TXComplete = false;
		if (length<=6) //SingleFrame is doable
		{
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("Ln%D\t\tSingleFrame",__LINE__);
			#endif
			cursor = 0;
			txFrame.data[1] = (byte)length;
			txFrame.data[2] = SID;
			for (int i = 0; i < payloadLength; i++)
			{
				txFrame.data[i+3] = payload[i];
				cursor++;
			}
			ESP32Can.writeFrame(txFrame,TX_FRAME_TIMEOUT);
			TXComplete = true;
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("\t SENT\n");
			#endif
		}
		else //It is a first MF
		{
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("Ln%D\t\tMultiFrame first part",__LINE__);
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
			ESP32Can.writeFrame(txFrame,TX_FRAME_TIMEOUT);
			pendingFCFrame = true;
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("\t SENT\n");
			#endif
		}
	}
	else //Pending FC Frame... assumes this is in reaction to a FC frame and the transfer is incomplete
	{
		#ifdef KWP_FRAME_DEBUG
			_debugSerial.printf("Ln%D\t\tContinuation frames\n",__LINE__);
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
					txFrame.data[2+i] = 0xAA;
				}
			}
			ESP32Can.writeFrame(txFrame,TX_FRAME_TIMEOUT);
			#ifdef KWP_FRAME_DEBUG
				_debugSerial.printf("Ln%D\t\t\tSENT with seq number 0x%02X\n",__LINE__,seqNumber);
			#endif
			seqNumber++;
		}  
		pendingFCFrame = false;
	}
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\t\tFINAL : pendingFCFrame: %d RXComplete: %d TXComplete: %d Length: %d PayloadLength: %d Cursor: %d \n",__LINE__,pendingFCFrame,RXComplete,TXComplete,length,payloadLength,cursor);
	#endif
}

/// @brief 
/// @param targetStream 
void kwpFrame::printKwpFrame(Stream& targetStream)
{
	
	if(RXComplete && (frameType != invalidFrameType && frameType != flowControlFrame)) {
		targetStream.printf("Ln%D\t%02X | Sender: %02X Target: %02X SID: %02X PayloadLength: %3d |  ",__LINE__,frameType,sender,target,SID,payloadLength); 
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

/// @brief Used to reset Frames when in Timeout
void kwpFrame::resetFrame()
{
	#ifdef KWP_FRAME_DEBUG
		_debugSerial.printf("Ln%D\tResetting KWP Frame ...\n",__LINE__);
	#endif
	
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

/// @brief 
/// @param targetSerial 
void kwpFrame::attachDebugSerial(Stream &targetSerial)
{
	_debugSerial = targetSerial;
}
