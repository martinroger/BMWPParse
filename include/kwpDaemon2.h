#pragma once
#include <Arduino.h>
#include <kwpFrame2.h>

#pragma region DEFINES
#ifndef KWP_DAEMON_ECU_ID
    #define KWP_DAEMON_ECU_ID 0xF1
#endif
#ifndef KWP_DAEMON_TARGET_ID
    #define KWP_DAEMON_TARGET_ID 0x12
#endif
#ifndef KWP_DAEMON_TX_INTERVAL_MS
    #define KWP_DAEMON_TX_INTERVAL_MS 20
#endif
#ifndef KWP_DAEMON_TX_TIMEOUT_MS
    #define KWP_DAEMON_TX_TIMEOUT_MS 0
#endif
#ifndef KWP_DAEMON_RX_TIMEOUT_MS
    #define KWP_DAEMON_RX_TIMEOUT_MS 0
#endif
#ifndef KWP_DAEMON_TX_BUFFER_LEN
    #define KWP_DAEMON_TX_BUFFER_LEN 16
#endif
#ifndef KWP_DAEMON_TWAI_TXQUEUE_LEN
    #define KWP_DAEMON_TWAI_TXQUEUE_LEN 16
#endif
#ifndef KWP_DAEMON_TWAI_RXQUEUE_LEN
    #define KWP_DAEMON_TWAI_RXQUEUE_LEN 16
#endif
#ifndef KWP_DAEMON_NO_RESP_TIMEOUT_MS
    #define KWP_DAEMON_NO_RESP_TIMEOUT_MS 1000
#endif
#pragma endregion


#pragma region UTILS
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
#pragma endregion


#pragma region ENUMS
enum KWP_CANFRAME_TYPE : byte
{
	SINGLE_FRAME        =   0x00,
	FIRST_FRAME         =   0x10,
	CONTINUATION_FRAME  =   0x20,
	FLOWCONTROL_FRAME   =   0x30,
	INVALID_FRAME       =   0xFF
};

enum KWP_DAEMON_STATE
{
    KWP_DAEMON_INIT_ST,
    KWP_DAEMON_CLEAR_REQ_ST,
    KWP_DAEMON_SETUP_REQ_ST,
    KWP_DAEMON_READ_REQ_ST,
    KWP_DAEMON_PARSE_ST
};
#pragma endregion


#pragma region CLASS DEFINITION
class kwp_Daemon
{
    public:
        
        KWP_DAEMON_STATE status = KWP_DAEMON_INIT_ST;

        kwp_Daemon(byte _senderID, byte _targetID);

        bool begin();
        bool reset();
        bool tick();
        bool processRXCanFrame(twai_message_t* frameToProcess); //Takes a pointed CAN Frame and processes it

        byte targetID = KWP_DAEMON_TARGET_ID;
        byte senderID = KWP_DAEMON_ECU_ID;

    private:
        bool _waitForFCFrame = false;                           //Set up by the txQueue popper, down on reception

        unsigned long _lastKWPReceived_ts = 0;                  //Timeout variable for last KWP received

        twai_message_t _txBuffer[KWP_DAEMON_TX_BUFFER_LEN];
        uint16_t _txBufferLen = 0;

        kwpFrame _rxFrame;

        bool _processRXKwpFrame(kwpFrame* frameToProcess);      //Takes a pointed frame and processes it

        bool _pushToTxBuffer(twai_message_t frameToQueue);      //Used internally in the other methods
        bool _popTxBuffer();                                    //Can modify internally _waitForFCFrame after popping
        
        bool _ReqClearDDLI();                                   //Very basic sending of clear request
        bool _ReqSetDDLI();                                     //Calculates and send the DDLI request
        bool _ReqReadDDLI();                                    //Very basic sending of read request

        bool _sendFCFrame();                                    //For flow control
        void _twaiStatusWatchDog();                             //For debug
};
#pragma endregion


#pragma region PRESET OBJECTS

#pragma endregion