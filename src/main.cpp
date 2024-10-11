#include <Arduino.h>
#include "kwpDaemon.h"

#ifndef CAN_TX
  #define CAN_TX D2
#endif
#ifndef CAN_RX
  #define CAN_RX D1
#endif

CanFrame rxFrame = {0};

kwpDaemon moloch;

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
  ESP32Can.begin();

}

void loop() {

  if(ESP32Can.readFrame(rxFrame)) //Check incoming rxFrame
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
}



