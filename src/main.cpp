#include <Arduino.h>
#include <Arduino_Helpers.h>
#include <AH/Timing/MillisMicrosTimer.hpp>
#include "kwpFrame.h"

#define CAN_TX D2
#define CAN_RX D1

CanFrame rxFrame = {0};

enum KWP_DAEMON_STATE 
{
  INIT, //Starting state, will attempt to go clear the DDLI
  DDLI_CLEAR, //Request to clear the DDLI is sent out, waiting for Positive Response
  DDLI_SETUP, //Positive clear, request to setup the DDLI and wait
  DDLI_REQUEST, //DDLI setup successfully, read DDLI
  DDLI_PARSE //Positive response and DDLI received, parse and revert to DDLI_REQUEST
};

byte daemonID = 0xF1; //Daemon receiver address
byte targetID = 0x12; //ECU ID targetted by daemon
Timer<millis> daemonTimeout = 15000; //If no POS response is received by that time, daemon resets to INIT
kwpFrame rxKwpFrame;
kwpFrame txKwpFrame;

void sendFCFrame(byte sender, byte target) {
  CanFrame FCFrame = {0};
  FCFrame.identifier = 0x600 + sender;
  FCFrame.data_length_code = 8;
  FCFrame.self=0;
  FCFrame.ss = 0;
  FCFrame.data[0] = target;
  FCFrame.data[1] = 0x30;

  ESP32Can.writeFrame(FCFrame);
  #ifdef SERIAL_DEBUG
    Serial.printf("\tFC frame sent from 0x6%02X to 0x6%02X\n",sender,target);
  #endif
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
  ESP32Can.begin();

}

void loop() {

  if(ESP32Can.readFrame(rxFrame)) //Check incoming rxFrame
  {
    #ifdef LED_BUILTIN
      digitalWrite(LED_BUILTIN,LOW);
    #endif
    if(((rxFrame.identifier & 0xF00) == (0x600)) && (rxFrame.data[0]==daemonID)) //If this message is targeted at daemon
    {  
      rxKwpFrame.processCanFrame(&rxFrame);
      switch (rxKwpFrame.frameType) //Choose response based on the incoming frameType
      {
      case singleFrame:
        #ifdef SERIAL_DEBUG
          rxKwpFrame.printKwpFrame();
        #endif
        //Do something based on the SID
        break;
      case flowControlFrame:
        #ifdef SERIAL_DEBUG
            rxKwpFrame.printKwpFrame();
        #endif
        if(txKwpFrame.pendingFCFrame) txKwpFrame.sendKwpFrame();
        break;
      case continuationFrame:
        if(rxKwpFrame.RXComplete) {
          //Do something based on the SID
          #ifdef SERIAL_DEBUG
            rxKwpFrame.printKwpFrame();
          #endif
        }
        break;
      case firstFrame:
        //Reply with the Flow Control frame
        sendFCFrame(daemonID,rxKwpFrame.sender);
        break;
      default:
          #ifdef SERIAL_DEBUG
            Serial.printf("%d\t Invalid FrameType\n",__LINE__);
          #endif
        break;
      }
    }
    else //Not a daemon recipient or not a KWP Frame
    { 

    }
    #ifdef LED_BUILTIN
      digitalWrite(LED_BUILTIN,HIGH);
    #endif

  }
}



