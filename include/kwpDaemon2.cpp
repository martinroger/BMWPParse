#pragma once
#include <kwpDaemon2.h>

/// @brief 
/// @param _senderID 
/// @param _targetID 
kwp_Daemon::kwp_Daemon(byte _senderID, byte _targetID) : senderID(_senderID), targetID(_targetID)
{
}

/// @brief 
/// @param _canTx 
/// @param _canRx 
/// @return 
bool kwp_Daemon::begin(uint8_t _canTx, uint8_t _canRx)
{
    bool ret = false;

    //Twai configuration
	twai_general_config_t g_config =  {	.mode = TWAI_MODE_NORMAL, 
										.tx_io = (gpio_num_t) _canTx, 
										.rx_io = (gpio_num_t) _canRx, 
										.clkout_io = TWAI_IO_UNUSED, 
										.bus_off_io = TWAI_IO_UNUSED,      
										.tx_queue_len = 16, 
										.rx_queue_len = 16,       
										.alerts_enabled = TWAI_ALERT_NONE,  
										.clkout_divider = 0,        
										.intr_flags = ESP_INTR_FLAG_LEVEL1};
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  
  	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    //Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) 
    {
    	ret = true;
  	} 
    else 
    {
		ret = false;
  	}

    // Start TWAI driver
	if (twai_start() == ESP_OK) 
    {
		ret = true;
	} 
    else 
    {
		ret = false;
	}
    
    // Reconfigure alerts to detect TX alerts and Bus-Off errors
	uint32_t alerts_to_enable = 	TWAI_ALERT_RX_DATA 
								| 	TWAI_ALERT_TX_FAILED 
								| 	TWAI_ALERT_ERR_PASS 
								| 	TWAI_ALERT_BUS_ERROR 
								| 	TWAI_ALERT_RX_QUEUE_FULL
							//	|	TWAI_ALERT_TX_SUCCESS
								|	TWAI_ALERT_ARB_LOST
								;
	if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) 
    {
		//Serial.println("CAN Alerts reconfigured");
	} 
    else 
    {
		//Serial.println("Failed to reconfigure alerts");
	}

    //Some housekeeping
 
    
    return ret;
}

/// @brief 
/// @return 
bool kwp_Daemon::reset()
{
    //Reset state
    status = KWP_DAEMON_INIT_ST;
    _waitForFCFrame = false;
    _lastKWPReceived_ts = millis();
    _lastTick_ts    =   millis();
    _txBufferLen = 0;
    _rxFrame.reset();
    
    return true;
}

/// @brief Ticker for the daemon. Defines state-related actions, and timer-related actions
/// @param inhibit External inhibit parameter. Will cause periodical resets and stop communication as long as it is active
/// @return TBD
bool kwp_Daemon::tick(bool inhibit)
{
    bool ret = true; //Inverted logic, faults set false
    unsigned long currentMillis = millis();
    if(!inhibit) //KWP DAEMON is not inhibited externally
    {
        //Reset regardless of state if no KWP message has been received ina a while
        if(currentMillis - _lastKWPReceived_ts > KWP_DAEMON_NO_RESP_TIMEOUT_MS)
        {
            reset();
            ret = false;
        }
        //Run through the status if the tick interval has been met
        if(currentMillis - _lastTick_ts > KWP_DAEMON_TICK_INTERVAL_MS)
        {
            KWP_DAEMON_STATE prevStatus = status;
            switch (prevStatus)
            {
            case KWP_DAEMON_INIT_ST: //Starting state, or reset consequence
                if(_ReqClearDDLI()) //Move to KWP_DAEMON_CLEAR_REQ_ST and reset timeout
                {
                    status = KWP_DAEMON_CLEAR_REQ_ST;
                    _lastKWPReceived_ts = currentMillis;
                }
                else    //Throw an issue
                {
                    ret = false;
                }        
                break;
            case KWP_DAEMON_CLEAR_REQ_ST:
                //Do nothing, wait either for a reset or for a response - 0x6C
                break;
            case KWP_DAEMON_SETUP_REQ_ST:
                //Do nothing, wait either for a reset or for a response - 0x6C
                break;
            case KWP_DAEMON_READ_REQ_ST:
                //Do nothing, wait either for a reset or for a response - 0x61
                break;
            case KWP_DAEMON_PARSED_ST:
                //If read interval is met, move to KWP_DAEMON_READ_REQ_SET
                if(currentMillis - _lastRead_ts > KWP_DAEMON_READ_INTERVAL_MS)
                {
                    if(_ReqReadDDLI())
                    {
                        status = KWP_DAEMON_READ_REQ_ST;
                        _lastRead_ts = currentMillis;
                        ret = true;
                    }
                    else
                    {
                        if(_ReqClearDDLI()) //Move to KWP_DAEMON_CLEAR_REQ_ST and reset timeout
                        {
                            status = KWP_DAEMON_CLEAR_REQ_ST;
                            _lastKWPReceived_ts = currentMillis;
                        }
                        else    //Throw an issue and reset
                        {
                            reset();
                            ret = false;
                        }   
                    }
                }
                break;
            default:
                //Not sure what to do in case of ENUM corruption
                break;
            }
            //Reset timestamp
            _lastTick_ts = currentMillis;
        }
        //Send anything that is ready to send
        if(currentMillis - _lastTx_ts > KWP_DAEMON_TX_INTERVAL_MS)
        {
            if(_popTxBuffer())
            {
                _lastTx_ts = currentMillis;
                
            }
            else
            {
                //Do something ?
            }
            //ret something ?
        }
    }
    else //If the external inhibit is live, periodically reset
    {
        if(currentMillis - _lastTick_ts > KWP_DAEMON_TICK_INTERVAL_MS) reset();
    }
    
    //Final return
    return ret;
}

/// @brief Process a candidate CAN Frame for KWP telegram, called from loop() when a twai_message_t is received. Does not reset KWP timeout !!
/// @param frameToProcess Candidate CAN Frame that has to be processed. Target/sender filtration happens in the function
/// @return False if processing failed, wrong target, wrong sender, failed subprocessing, unexpected frame. True if subprocessing went OK
bool kwp_Daemon::processRXCanFrame(twai_message_t* frameToProcess)
{
    bool ret = false;
    
    //Exclude frames that don't match the targetID and SenderID conditions
    if(((frameToProcess->identifier) == (0x600 + targetID)) && (frameToProcess->data[0] == senderID))
    {
        byte typeNibble = frameToProcess->data[1] % 0xF0;       //Get frame type nibble
        KWP_CANFRAME_TYPE frameType = INVALID_FRAME;            //Get frame type
        if(!(typeNibble>0x30))
        {
            frameType = (KWP_CANFRAME_TYPE) typeNibble;
        }

        //Switch on the frameType
        switch (frameType)
        {
        case SINGLE_FRAME:
            //TODO : introduce additional checks here : unexpected frame type etc excessive length...
            _rxFrame.setMetadaData(frameToProcess); //Sets target, sender and multiframe flag. Expected Multiframe flag is false
            _rxFrame.length         =   frameToProcess->data[1];
            _rxFrame.bufferLength   =   _rxFrame.length - 1;
            _rxFrame.SID            =   frameToProcess->data[2];
            _rxFrame.cursor         =   0;
            if(_rxFrame.appendCanFrameBuffer(frameToProcess, 3 , 3 + _rxFrame.bufferLength - 1))
            {
                ret = true;
            }
            else
            {
                ret = false;
                break;
            }
            ret = _processRXKwpFrame(&_rxFrame);
            break;
        
        case FIRST_FRAME:
            //TODO : introduce additional checks here : unexpected frame type etc excessive length...
            _rxFrame.setMetadaData(frameToProcess); //Sets target, sender and multiframe flag. Expected Multiframe flag is true
            _rxFrame.length         =   (swap_endian<uint16_t>(*((uint16_t*)&(frameToProcess->data[1]))) & 0x0FFF);
            _rxFrame.bufferLength   =   _rxFrame.length - 1;
            _rxFrame.SID            =   frameToProcess->data[3];
            _rxFrame.cursor         =   0;
            if(_rxFrame.appendCanFrameBuffer(frameToProcess, 4))
            {
                ret = true;
            }
            else
            {
                ret = false;
                break;
            }
            //Send the flow control frame, don't process anything
            ret = _sendFCFrame();
            break;

        case CONTINUATION_FRAME:
            //_rxFrame metadata is expected to already be set
            //TODO : introduce additional checks here : unexpected frame type etc excessive length...
            //Copy one by one the frameToProcess to the buffer, until it breaks (i.e cursor == bufferLength)
            for (int i = 2; i < 8; i++)
            {
                if(!_rxFrame.appendCanFrameBuffer(frameToProcess,i,i)) break;
            }
            //Double check and then process
            if(_rxFrame.cursor == _rxFrame.bufferLength)
            {
                _rxFrame.rxComplete = true;
                ret = _processRXKwpFrame(&_rxFrame);
            }
            break;
        
        case FLOWCONTROL_FRAME:
            //No need to use _rxFrame
            //TODO : specific checks on Flow Control frame content
            if(_waitForFCFrame) //FlowControl frame was expected
            {
                _waitForFCFrame = false;
                ret = true;
            }
            else //FlowControl frame was not expected
            {
                ret = false;
            }
            break;

        default: //Only invalid frames get here
            ret = false;
            break;
        }
    }

    //Final return. False if couldn't process or somehow the frame is not targetted at the daemon
    return ret;
}

/// @brief In spirit an SID processor. Called in line with the processRXCanFrame in loop()
/// @param frameToProcess 
/// @return True in case of error-free process, including if there are successful status resets. False if there is a TX error or anything leading to a fall back to INIT
bool kwp_Daemon::_processRXKwpFrame(kwpFrame *frameToProcess)
{
    bool ret = false;
    KWP_DAEMON_STATE prevStatus = status;
    switch (frameToProcess->SID)
    {
    case 0x6C:  //Response SID to either a clear or a set request
        switch (prevStatus)
        {
        case KWP_DAEMON_CLEAR_REQ_ST:   //Daemon had sent a clear request, can progress towards setup request
            if(_ReqSetDDLI())   //Queue the set up frames, move to KWP_DAEMON_SETUP_REQ_ST, reset timeout
            {
                status = KWP_DAEMON_SETUP_REQ_ST;
                ret = true;
            }
            else    //Somehow failed to process, return to KWP_DAEMON_INIT_ST and do not reset timeout
            {
                status = KWP_DAEMON_INIT_ST;
                ret = false;
            }
            break;
        
        case KWP_DAEMON_SETUP_REQ_ST:   //Daemon had sent a setup request, can progress towards the reading loop
            if(_ReqReadDDLI())  //Starts reading the DDLI, setup was successful, move to KWP_DAEMON_READ_REQ_ST, reset timeout
            {
                status = KWP_DAEMON_READ_REQ_ST;
                ret = true;
            }
            else    //Somehow failed to process, attempt to return to KWP_DAEMON_CLEAR_REQ_ST and reset timeout, else drop to KWP_DAEMON_INIT_ST
            {
                if(_ReqClearDDLI()) //Attempt to re-clear
                {
                    status = KWP_DAEMON_CLEAR_REQ_ST;
                    ret = true;
                }
                else    //Revert to Init and wait for next loop
                {
                    status = KWP_DAEMON_INIT_ST;
                    ret = false;
                }
            }
            break;      
        
        default:    //The Daemon is not in any normal state to receive this request.
            if(_ReqClearDDLI()) //Attempt to re-clear, revert to KWP_DAEMON_CLEAR_REQ_ST and reset timeout
            {
                status = KWP_DAEMON_CLEAR_REQ_ST;
                ret = true;
            }
            else    //Revert to KWP_DAEMON_INIT_ST and wait for next loop, do not reset timeout
            {
                status = KWP_DAEMON_INIT_ST;
                ret = false;
            }
            break;
        }
        break;
    case 0x61:  //Response SID to a read DDLI request
        switch (prevStatus)
        {
        case KWP_DAEMON_READ_REQ_ST:    //Daemon had sent a read request, all is good
            if(_parseDDLI())    //Parsing was successful, reset timeout after moving to KWP_DAEMON_PARSED_ST. Tick() will send the next read request
            {
                status = KWP_DAEMON_PARSED_ST;
                ret = true;
            }
            else    //Somehow failed to process, attempt to return to KWP_DAEMON_CLEAR_REQ_ST and reset timeout, else drop to KWP_DAEMON_INIT_ST
            {
                if(_ReqClearDDLI()) //Attempt to re-clear
                {
                    status = KWP_DAEMON_CLEAR_REQ_ST;
                    ret = true;
                }
                else    //Revert to Init and wait for next loop
                {
                    status = KWP_DAEMON_INIT_ST;
                    ret = false;
                }
            }
            break;
        
        default:    //Daemon was not in expected state
            if(_ReqClearDDLI()) //Attempt to re-clear
            {
                status = KWP_DAEMON_CLEAR_REQ_ST;
                ret = true;
            }
            else    //Revert to Init and wait for next loop
            {
                status = KWP_DAEMON_INIT_ST;
                ret = false;
            }
            break;
        }
        break;
    
    case 0x7F:  //Target returned an error, drop to KWP_DAEMON_CLEAR_REQ_ST
        if(_ReqClearDDLI()) //Attempt to re-clear
        {
            status = KWP_DAEMON_CLEAR_REQ_ST;
            ret = true;
        }
        else    //Revert to Init and wait for next loop
        {
            status = KWP_DAEMON_INIT_ST;
            ret = false;
        }
        break;

    default:
        //Do nothing for now
        break;
    }

    //Reset the timeout timer if the processing was OK
    if(ret) _lastKWPReceived_ts = millis();

    //Final catch-all return
    return ret;
}

/// @brief Appends a twai_message_t to the daemon buffer in a FIFO style
/// @param frameToQueue twai_message_t to append at the end of the queue
/// @return false if the queue is full
bool kwp_Daemon::_pushToTxBuffer(twai_message_t frameToQueue)
{
    bool ret = false;
    
    if(_txBufferLen == KWP_DAEMON_TX_BUFFER_LEN) //We are at the end of the buffer, no element can be appended
    {
        ret = false;
    }
    else //We can append and increase the counter
    {
        _txBuffer[_txBufferLen] = frameToQueue;
        _txBufferLen++;
        ret = true;
    }

    //Final status return
    return ret;
}

/// @brief Attempts to transmit the oldest twai_message_t in the queue. Called from tick.
/// @return True if the queue is empty, or if there is a FC Frame hold, or if the transmit was successful. False otherwise (usually transmit errors)
bool kwp_Daemon::_popTxBuffer()
{
    bool ret = false;
    //Immediately quit if the tx buffer is empty or there is pending incoming FC Frame
    if((_txBufferLen<1) || _waitForFCFrame)
    { 
        ret = true;
        return ret;
    }
    else //There is something to transmit and it is allowed
    {
        if(twai_transmit( &(_txBuffer[0]) , pdMS_TO_TICKS(KWP_DAEMON_TX_TIMEOUT_MS) ) == ESP_OK )
        {
            //Set a FCFrame flag if this is a firstFrame
            if((_txBuffer[0].data[1] & 0xF0) == 0x10) _waitForFCFrame = true;
            //Shift the tx buffer left
            _txBufferLen = _txBufferLen - 1;
            for (int i = 0; i < _txBufferLen; i++)
            {
                _txBuffer[i] = _txBuffer[i+1];
            }
            ret = true;
        }
        else    //Some transmit error, return false
        {
            ret = false;
        }
    }

    //Final return
    return ret;
}

/// @brief Converts a healthy KWPFrame into a series of twai_message_t all sent to the Tx Buffer
/// @param frameToPackage Target KWP Frame to slice and dice
/// @return True if all OK, false if there is an issue pushing to the Buffer, typically overrun
bool kwp_Daemon::_packageKWPFrame(kwpFrame *frameToPackage)
{
    bool ret = false;

    //Do some homework first
    frameToPackage->calculateMetaData();
    frameToPackage->cursor = 0;

    //Prepare template twai_message_t
    twai_message_t canFrame;
    canFrame.identifier         =   0x600 + senderID;
    canFrame.data_length_code   =   8;
    canFrame.ss                 =   0;
    canFrame.extd               =   0;
    canFrame.rtr                =   0;
    canFrame.data[0]            =   targetID;

    //Bifurcate pased on calculated length
    if(frameToPackage->multiFrame) //If it is a multiframe
    {
        canFrame.data[1]    =   0x10 + ((frameToPackage->length & 0x0F00) >> 8);
        canFrame.data[2]    =   (frameToPackage->length & 0xFF);
        canFrame.data[3]    =   frameToPackage->SID;
        for (int i = 0; i < 4; i++)
        {
            canFrame.data[4+i] = frameToPackage->buffer[frameToPackage->cursor];
            frameToPackage->cursor = frameToPackage->cursor + 1;
        }
        //Write the FF to the buffer
        if(_pushToTxBuffer(canFrame)) ret = true;
        
        //Now package the continuation frames
        uint8_t seqNumber = 1;
        while(frameToPackage->cursor < frameToPackage->bufferLength)
        {
            canFrame.data[1] = 0x20 + seqNumber;
            for (int i = 0; i < 6; i++)
            {
                if (frameToPackage->cursor < frameToPackage->bufferLength)
                {
                    canFrame.data[2+i]      =   frameToPackage->buffer[frameToPackage->cursor];
                    frameToPackage->cursor  =   frameToPackage->cursor + 1;
                }
                else //Padding
                {
                    canFrame.data[2+i]      =   0xAA; 
                }
            }
            if(_pushToTxBuffer(canFrame)) ret = true;
            else
            {
                ret = false;
                break;
            }
            seqNumber++;
        }
        //Problems caught !
        if(!ret) return ret;
    }
    else    //If it is achievable in a single frame
    {
        canFrame.data[1]    =   (byte)(frameToPackage->length);
        canFrame.data[2]    =   frameToPackage->SID;
        for (int i = 3; i < 8; i++)
        {
            if (frameToPackage->cursor < frameToPackage->bufferLength)
            {
                canFrame.data[i]        =   frameToPackage->buffer[frameToPackage->cursor];
                frameToPackage->cursor  = frameToPackage->cursor + 1;
            }
            else //Some padding
            {
                canFrame.data[i]        =   0xAA;
            }
            
        }
        //Write the SF to the buffer
        if(_pushToTxBuffer(canFrame)) ret = true;
    }

    //Final return
    return ret;
}

/// @brief 
/// @return 
bool kwp_Daemon::_ReqClearDDLI()
{
    bool ret = false;
    const byte buf[2] = {0xF0,0x04};
    kwpFrame frame( targetID , senderID , 0x2C , 2 , buf );
    if(_packageKWPFrame(&frame)) ret = true;
    return ret;
}

/// @brief 
/// @return 
bool kwp_Daemon::_ReqSetDDLI()
{
    bool ret = false;
    const byte buf[55] = {
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
    kwpFrame frame( targetID , senderID , 0x2C , 55 , buf );
    if(_packageKWPFrame(&frame)) ret = true;
    return ret;
}

bool kwp_Daemon::_ReqReadDDLI()
{
    bool ret = false;
    const byte buf[1] = {0xF0};
    kwpFrame frame( targetID , senderID , 0x21 , 1 , buf );
    if(_packageKWPFrame(&frame)) ret = true;
    return ret;
}

bool kwp_Daemon::_sendFCFrame()
{
    bool ret = false;
    twai_message_t canFrame;
    canFrame.identifier         =   0x600 + senderID;
    canFrame.data_length_code   =   8;
    canFrame.ss                 =   0;
    canFrame.extd               =   0;
    canFrame.rtr                =   0;
    canFrame.data[0]            =   targetID;
    canFrame.data[1]            =   0x30;
    canFrame.data[2]            =   0x00;
    canFrame.data[3]            =   0x00;
    for (int i = 4; i < 8; i++)
    {
        canFrame.data[i] = 0xFF;
    }
    
    if(_pushToTxBuffer(canFrame)) ret = true;

    return ret;
}

void kwp_Daemon::_twaiStatusWatchDog()
{
    static twai_status_info_t twaistatus;
	twai_get_status_info(&twaistatus);
	Serial.printf("%s\t",__func__);
	Serial.printf("Bus errors:%lu\t",twaistatus.bus_error_count);
	Serial.printf("TX queue:%lu\t",twaistatus.msgs_to_tx);
	Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
	Serial.printf("TX failed: %lu\t", twaistatus.tx_failed_count);
	Serial.printf("ARB lost: %lu\n", twaistatus.arb_lost_count);
}
