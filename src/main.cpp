//Define before anyone else
#ifndef CAN_TX
	#define CAN_TX D2
#endif
#ifndef CAN_RX
	#define CAN_RX D1
#endif


#include <Arduino.h>
#include "kwpDaemon.h"
//#include "esp_log.h"
#include "driver/twai.h"

kwpDaemon moloch(0xF1,0x12);
twai_message_t rxMessage;


//Setup function
void setup() 
{
	
	//Serial Debug
	pinMode(LED_BUILTIN,OUTPUT);
	digitalWrite(LED_BUILTIN,!LOW);
	Serial.begin(115200);
/* 	while(millis()<2000 || Serial.available()) {
		Serial.read();
		digitalWrite(LED_BUILTIN,!HIGH);
	}
	while(Serial.available()==0 && millis()<60000) {
		delay(10);
		digitalWrite(LED_BUILTIN,!HIGH);
	} */
	digitalWrite(LED_BUILTIN,!LOW);

	//Daemon is summoned
	moloch.begin(CAN_TX,CAN_RX);

}

void loop() {
	
	//Handle any message in the buffer until it is empty
	while (twai_receive(&rxMessage, 0) == ESP_OK) {
		moloch.processRXCanFrame(&rxMessage);
	}
	
	//Tickle the daemon
	moloch.tick(false);

}