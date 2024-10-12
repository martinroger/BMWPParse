#include <Arduino.h>
#include "kwpDaemon.h"

#ifndef CAN_TX
	#define CAN_TX D2
#endif
#ifndef CAN_RX
	#define CAN_RX D1
#endif

CanFrame rxFrame = {0};
bool canState;

kwpDaemon moloch;

void parseHandler() {
	Serial.printf("Ln%D\tParseHandler called\n",__LINE__);
	moloch.rxKwpFrame.printKwpFrame();
}

void setup() {

	Serial.begin(115200);
	#ifdef LED_BUILTIN
		pinMode(LED_BUILTIN,OUTPUT);
		digitalWrite(LED_BUILTIN,HIGH);
	#endif

	ESP32Can.setPins(CAN_TX,CAN_RX);
	ESP32Can.setRxQueueSize(64);
	ESP32Can.setTxQueueSize(64);
	ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
	canState = ESP32Can.begin();

	moloch.attachDataHandler(parseHandler);

	delay(15000);

}

void loop() {

	if(ESP32Can.readFrame(rxFrame,0)) //Check incoming rxFrame
	{
		#ifdef LED_BUILTIN
			digitalWrite(LED_BUILTIN,LOW);
		#endif
		if(moloch.processIncomingCANFrame(rxFrame)) {

		}
		else //Not processed because not valid demonic frame
		{ 
		}
		#ifdef LED_BUILTIN
				digitalWrite(LED_BUILTIN,HIGH);
		#endif
	}

	moloch.tick(canState);
}



