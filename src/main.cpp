#include <Arduino.h>
//#include "kwpDaemon.h"
//#include "esp_log.h"
#include "driver/twai.h"

#ifndef CAN_TX
	#define CAN_TX D2
#endif
#ifndef CAN_RX
	#define CAN_RX D1
#endif

#define CYCLIC_TRANSMIT_RATE_MS 5 //50 necessary, works down to 10
#define WAIT_FOR_ALERTS_MS 0
#define TX_TIME_OUT 0
#define MAX_REQS 30000
unsigned long requests = 0;

enum KWP_FRAME_TYPE : byte
{
	singleFrame         =   0x00,
	firstFrame          =   0x10,
	continuationFrame   =   0x20,
	flowControlFrame    =   0x30,
	invalidFrameType    =   0xFF
};

template <typename T>
T swap_endian(T u)
{
	static_assert (CHAR_BIT == 8, "CHAR_BIT != 8");

	union
	{
		T u;
		unsigned char u8[sizeof(T)];
	} source, dest;

	source.u = u;

	for (size_t k = 0; k < sizeof(T); k++)
		dest.u8[k] = source.u8[sizeof(T) - k - 1];

	return dest.u;
}

static bool driver_installed = false;
unsigned long previousMillis = 0;  // will store last time a message was sent
unsigned long previousDebug = 0;	//Will store last time the alerts were sent over Serial

uint32_t alerts_triggered;
unsigned long currentMillis;
twai_message_t rxMessage;
twai_message_t txBuffer[16];
int txBufferLen = 0;
unsigned long txED;
unsigned long rxED;

bool DDLICleared = false;
bool DDLISet = false;
bool waitForResp = false;
bool DONE = false;


//Setup function
void setup() 
{
	
	//Serial Debug
	pinMode(LED_BUILTIN,OUTPUT);
	digitalWrite(LED_BUILTIN,!LOW);
	Serial.begin(115200);
	while(Serial.available()>0) {
		Serial.read();
		digitalWrite(LED_BUILTIN,!HIGH);
	}
	while(Serial.available()==0) {
		delay(10);
		digitalWrite(LED_BUILTIN,!HIGH);
	}
	digitalWrite(LED_BUILTIN,!LOW);

	//Twai configuration
	twai_general_config_t g_config =  {	.mode = TWAI_MODE_NORMAL, 
										.tx_io = (gpio_num_t) CAN_TX, 
										.rx_io = (gpio_num_t) CAN_RX, 
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
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    	Serial.println("Driver installed");
  	} else {
		Serial.println("Failed to install driver");
		return;
  	}

	// Start TWAI driver
	if (twai_start() == ESP_OK) {
		Serial.println("Driver started");
	} else {
		Serial.println("Failed to start driver");
		return;
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
	if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
		Serial.println("CAN Alerts reconfigured");
	} else {
		Serial.println("Failed to reconfigure alerts");
		return;
	}

	// TWAI driver is now successfully installed and started
  	driver_installed = true;

	Serial.println(sizeof(rxMessage));
	Serial.println(sizeof(txBuffer));

}

//Twai status watchdog
void twaiStatusWatchdog()
{
	static twai_status_info_t twaistatus;
	twai_get_status_info(&twaistatus);
	Serial.printf("%s\t",__func__);
	Serial.printf("Bus errors:%lu\t",twaistatus.bus_error_count);
	Serial.printf("TX queue:%lu\t",twaistatus.msgs_to_tx);
	Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
	Serial.printf("TX failed: %lu\t", twaistatus.tx_failed_count);
	Serial.printf("ARB lost: %lu\t", twaistatus.arb_lost_count);
	Serial.printf("TXed:%lu\t",txED);
	Serial.printf("RXed:%lu\t",rxED);
	Serial.printf("REQs:%lu\n",requests);
}


static void queue_in_txBuffer(twai_message_t txBuf[], int* bufLen, twai_message_t qFrame)
{
	txBuf[*bufLen] = qFrame;
	*bufLen = *bufLen +1;
}

static bool pop_txBuffer(twai_message_t txBuf[], int* bufLen)
{
	bool ret = false;
	if(*bufLen<1) return ret;
	if(twai_transmit(&(txBuf[0]),pdMS_TO_TICKS(TX_TIME_OUT))==ESP_OK)
	{
		*bufLen = *bufLen -1;
		for (int i = 0; i < *bufLen; i++)
		{
			txBuf[i] = txBuf[i+1];
		}
		txED++;
		ret = true;
	}
	//Serial.println(*bufLen);
	return ret;
}


//Transmit handler
static void send_generic_message() 
{
	// Send message
	unsigned long snapMillis = millis();

	// Configure message to transmit
	twai_message_t outFrame;
	outFrame.identifier = 0x6F1;
	outFrame.ss = 0;
	outFrame.data_length_code = 8;
	outFrame.extd = 0;
	outFrame.rtr = 0;
	// for (int i = 0; i < 8; i++) 
	// {
	// 	outFrame.data[i] = ((millis()>>i) & 0xFF);
	// }
	*((uint64_t*)&(outFrame.data[0])) = (swap_endian<uint64_t>(rxED)|0xAAAAAAAAAAAAAAAA);
	// Queue message for transmission
	if (twai_transmit(&outFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	{
		//printf("Message queued for transmission\n");
		txED++;
	} 
	else 
	{
		//printf("Failed to queue message for transmission\n");
	}
}

static void send_pos_response(byte reqSID)
{
	twai_message_t posRespFrame;
	posRespFrame.identifier = 0x6F1;
	posRespFrame.data_length_code = 8;
	posRespFrame.ss = 0;
	posRespFrame.extd = 0;
	posRespFrame.rtr = 0;
	posRespFrame.data[0] = 0x12;
	posRespFrame.data[1] = 0x01;
	posRespFrame.data[2] = reqSID + 0x40;
	// if (twai_transmit(&posRespFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,posRespFrame);
}

static void send_clear_request()
{
	twai_message_t clearReqFrame;
	clearReqFrame.identifier = 0x6F1;
	clearReqFrame.data_length_code = 8;
	clearReqFrame.ss = 0;
	clearReqFrame.extd = 0;
	clearReqFrame.rtr = 0;
	clearReqFrame.data[0] = 0x12;
	clearReqFrame.data[1] = 0x03;
	clearReqFrame.data[2] = 0x2C;
	clearReqFrame.data[3] = 0xF0;
	clearReqFrame.data[4] = 0x04;
	// if (twai_transmit(&clearReqFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// 	waitForResp=true;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,clearReqFrame);
	waitForResp=true;
}

static void send_flow_control(byte targetID)
{
	twai_message_t flowControlFrame;
	flowControlFrame.identifier = 0x6F1;
	flowControlFrame.data_length_code = 8;
	flowControlFrame.ss = 0;
	flowControlFrame.extd = 0;
	flowControlFrame.rtr = 0;
	flowControlFrame.data[0] = targetID;
	flowControlFrame.data[1] = 0x30;
	flowControlFrame.data[2] = 0x00;
	flowControlFrame.data[3] = 0x00;
	flowControlFrame.data[4] = 0x00;
	// if (twai_transmit(&flowControlFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// 	waitForResp = true;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,flowControlFrame);
	waitForResp=true;
}

static void send_DDLISet_FF()
{
	byte tempPayload[] = {
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
	twai_message_t DDLISet_firstFrame;
	DDLISet_firstFrame.identifier = 0x6F1;
	DDLISet_firstFrame.data_length_code = 8;
	DDLISet_firstFrame.ss = 0;
	DDLISet_firstFrame.extd = 0;
	DDLISet_firstFrame.rtr = 0;
	DDLISet_firstFrame.data[0] = 0x12;
	DDLISet_firstFrame.data[1] = 0x10;
	DDLISet_firstFrame.data[2] = 56;
	DDLISet_firstFrame.data[3] = 0x2C;
	for (int i = 0; i < 4; i++)
	{
		DDLISet_firstFrame.data[i+4] = tempPayload[i];
	}

	// if (twai_transmit(&DDLISet_firstFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// 	waitForResp = true;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,DDLISet_firstFrame);
	waitForResp=true;

}

static void send_DDLISet_CF(int seqNumber)
{
byte tempPayload[] = {
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
	twai_message_t DDLISet_contFrame;
	DDLISet_contFrame.identifier = 0x6F1;
	DDLISet_contFrame.data_length_code = 8;
	DDLISet_contFrame.ss = 0;
	DDLISet_contFrame.extd = 0;
	DDLISet_contFrame.rtr = 0;
	DDLISet_contFrame.data[0] = 0x12;
	DDLISet_contFrame.data[1] = 0x20 + seqNumber;
	for (int i = 0; i < 6; i++)
	{
		if(4+i+(seqNumber-1)*6<56)
		{
			DDLISet_contFrame.data[2+i] = tempPayload[4+i+(seqNumber-1)*6]; //ugly
		}
		else
		{
			DDLISet_contFrame.data[2+i] = 0xAA;
		}
	}

	// if (twai_transmit(&DDLISet_contFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// 	waitForResp = true;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,DDLISet_contFrame);
	waitForResp=true;
}

static void send_read_request()
{
	twai_message_t readReqFrame;
	readReqFrame.identifier = 0x6F1;
	readReqFrame.data_length_code = 8;
	readReqFrame.ss = 0;
	readReqFrame.extd = 0;
	readReqFrame.rtr = 0;
	readReqFrame.data[0] = 0x12;
	readReqFrame.data[1] = 0x02;
	readReqFrame.data[2] = 0x21;
	readReqFrame.data[3] = 0xF0;

	// if (twai_transmit(&readReqFrame, pdMS_TO_TICKS(TX_TIME_OUT)) == ESP_OK) 
	// {
	// 	//printf("Message queued for transmission\n");
	// 	txED++;
	// 	waitForResp = true;
	// } 
	// else 
	// {
	// 	//printf("Failed to queue message for transmission\n");
	// }
	queue_in_txBuffer(txBuffer,&txBufferLen,readReqFrame);
	waitForResp=true;
	requests++;
}

//Incoming handler
static void handle_rx_message(twai_message_t inFrame) 
{
  	// Process received message
	static uint16_t length;
	static uint16_t payLoadLength;
	static byte payLoadBuffer[255];
	static byte SID;
	static byte cursor;
	static byte MultiFrame;
	
	
	if((inFrame.identifier == 0x612) && (inFrame.data[0] == 0xF1)) 
	{
		//Serial.print(__func__);
		byte typeNibble = inFrame.data[1] & 0xF0;
		KWP_FRAME_TYPE frameType = invalidFrameType;
		if (!(typeNibble>0x30))
		{
			frameType = (KWP_FRAME_TYPE)typeNibble;		
		}

		switch (frameType)
		{
		//Single Frames. Overrides any other type that might be unfinished
		case singleFrame:
			MultiFrame 		= 	false;
			length 			= 	inFrame.data[1];
			payLoadLength 	= 	length - 1;
			SID 			= 	inFrame.data[2];
			cursor 			= 	0;
			for (int i = 3; i < 3+payLoadLength; i++)
			{
				payLoadBuffer[cursor] = inFrame.data[i];
				cursor++;
			}

			//ProcessSID
			if(!DDLICleared && (SID==0x6C))
			{
				DDLICleared = true;
				waitForResp = false;
			}
			else if(DDLICleared && (SID==0x6C))
			{
				DDLISet = true;
				waitForResp = false;
			}
			
			//Payload debug output
			// Serial.printf("%s\tSingleFrame\tSID:%02X\tPayloadLen:%d\tPayload:",__func__,SID,payLoadLength);
			// for (int i = 0; i < payLoadLength; i++)
			// {
			// 	Serial.printf(" %02X",payLoadBuffer[i]);
			// }
			// Serial.println();
			break;
		//First frame. Sets up a MultiFrame counter, assumes data is a bit static between the loops
		case firstFrame:
			MultiFrame 		= 	true;
			length 			= 	(swap_endian<uint16_t>(*((uint16_t*)&(inFrame.data[1]))) & 0x0FFF);
			payLoadLength 	= 	length - 1;
			SID 			= 	inFrame.data[3];
			cursor 			= 	0;
			for (int i = 4; i < 8; i++)
			{
				payLoadBuffer[cursor] = inFrame.data[i];
				cursor++;
			}

			//ProcessSID
			if(DDLISet && (SID==0x61))
			{
				send_flow_control(0x12);
			}

			//Debug output
			// Serial.printf("%s\tFirstFrame\tSID:%02X\tPayloadLen:%d",__func__,SID,payLoadLength);
			// Serial.println();
			break;
		//ContinuationFrame. Will throw an error if MultiFrame is not set up accordingly
		case continuationFrame:
			if(MultiFrame)
			{
				for (int i = 2; i < 8; i++)
				{
					if(cursor<payLoadLength)
					{
						payLoadBuffer[cursor] = inFrame.data[i];
						cursor++;
					}
					else
					{
						MultiFrame = false;
						break;
					}
				}
				if(!MultiFrame) //If there has been a finished continuation frame
				{
					//ProcessSID
					waitForResp = false;
					//Payload debug output
					// Serial.printf("%s\tMultiFrame\tSID:%02X\tPayloadLen:%d\tPayload:",__func__,SID,payLoadLength);
					// for (int i = 0; i < payLoadLength; i++)
					// {
					// 	Serial.printf(" %02X",payLoadBuffer[i]);
					// }
					// Serial.println();
				}
				else
				{
					// Serial.printf("%s\tContinuationFrame\tSID:%02X\tPayloadLen:%d\tSequence:%d\n",__func__,SID,payLoadLength,(inFrame.data[1] & 0x0F));
				}
				
			}
			else //Unannounced ContinuationFrame
			{
				Serial.printf("%s\tInvalid ContinuationFrame\n",__func__);
			}
			break;
		//FlowControlFrame	
		case flowControlFrame:
			// Serial.printf("%s\tFlowControlFrame\n",__func__);
			//send_generic_message();
			for (int i = 1; i < 10; i++)
			{
				send_DDLISet_CF(i);
			}
			
			

			break;

		default:
			break;
		}
	}
	//For another selected frame template
	// else if (/* condition */)
	// {
	// 	/* code */
	// }
	//Any other frame
	else {
		return;
	}
}



void loop() {
	//Eject if driver is uninstalled
	if (!driver_installed) {
    	// Driver not installed
    	delay(1000);
    	return;
  	}
	
	// Check if alert happened, if yes react
	if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(WAIT_FOR_ALERTS_MS))==ESP_OK)
	{
		twai_status_info_t twaistatus;
		twai_get_status_info(&twaistatus);	
		// Handle alerts
		if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
			Serial.println("Alert: TWAI controller has become error passive.");
		}
		if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
			Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
			Serial.printf("Bus error count: %lu\n", twaistatus.bus_error_count);
		}
		if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
			Serial.println("Alert: The Transmission failed.");
			Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
			Serial.printf("TX error: %lu\t", twaistatus.tx_error_counter);
			Serial.printf("TX failed: %lu\n", twaistatus.tx_failed_count);
		}
		if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
			Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
			Serial.printf("RX buffered: %lu\t", twaistatus.msgs_to_rx);
			Serial.printf("RX missed: %lu\t", twaistatus.rx_missed_count);
			Serial.printf("RX overrun %lu\n", twaistatus.rx_overrun_count);
  		}
		if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
			Serial.println("Alert: The Transmission was successful.");
			Serial.printf("TX buffered: %lu\t", twaistatus.msgs_to_tx);
		}
		if (alerts_triggered & TWAI_ALERT_ARB_LOST) {
			Serial.println("Alert: Arbitration lost.");
			Serial.printf("ARB lost: %lu\n", twaistatus.arb_lost_count);
		}
	}

	//Handle any message in the buffer until it is empty
	while (twai_receive(&rxMessage, 0) == ESP_OK) {
		rxED++;
		handle_rx_message(rxMessage);
	}
	


	// Send message
	currentMillis = millis();
	if (currentMillis - previousMillis >= CYCLIC_TRANSMIT_RATE_MS) {
		previousMillis = currentMillis;
		//send_generic_message();
		if(!waitForResp)
		{
			if(!DDLICleared)
			{
				send_clear_request();
			}
			if (DDLICleared && !DDLISet)
			{
				send_DDLISet_FF();
			}
			if(DDLICleared && DDLISet && (requests<MAX_REQS))
			{
				send_read_request();
				requests++;
			}
		}
		
		if(txBufferLen>0) 
		{
			if(!pop_txBuffer(txBuffer,&txBufferLen)) twaiStatusWatchdog();
		}
		
	}

	if(requests>=MAX_REQS && !DONE)
	{
		twaiStatusWatchdog();
		Serial.println("Done.");
		DONE = true;
	}
}