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
        twai_message_t txBuffer[KWP_DAEMON_TX_BUFFER_LEN];

        bool begin();
        bool reset();
        bool tick();
        bool processRXCanFrame(twai_message_t frameToProcess);
    private:
        bool _waitForFCFrame = false;
        unsigned long lastKWPReceived_ts = 0;

        bool _pushToTxBuffer(twai_message_t frameToQueue);
        bool _popTxBuffer(); //Can modify internally _waitForFCFrame
        
        void _ReqClearDDLI();
        void _ReqSetDDLI();
        void _ReqReadDDLI();

        void _sendFCFrame();
        void _twaiStatusWatchDog();
};
#pragma endregion


#pragma region PRESET OBJECTS

#pragma endregion