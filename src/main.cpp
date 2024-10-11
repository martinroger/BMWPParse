#include <Arduino.h>
#include "kwpFrame.h"

#define CAN_TX D2
#define CAN_RX D1



/*Usage : 
tempTrackIndex = swap_endian<uint32_t>(*((uint32_t*)&byteArray[3]));
*/

CanFrame rxFrame = {0};

byte target;
byte sender;
byte SID;
uint16_t payloadLen;
uint16_t otherLen;
byte cursor;
byte kwpBuf[255];
bool frameComplete = true;

KWP_FRAME_TYPE rxFrameType;

kwpFrame rxKwpFrame;
kwpFrame exampleFrame;

void buildExampleFrame() {
    exampleFrame.target = 0x12;
    exampleFrame.sender = 0xF1;
    exampleFrame.SID    = 0x2C;
    exampleFrame.length = 56;
    exampleFrame.payloadLength = 55;
    exampleFrame.cursor = 0;
    byte tempPayload[] = { 0xF0, 0x02, 0x01, 0x01, 0x58, 0x0C, 0x01, 0x02, 0x02, 0x02, 0x58, 0xF0, 0x01, 0x02, 0x04, 0x02, 0x5A, 0xBC, 0x01, 0x02, 0x06, 0x02, 0x58, 0xDD, 0x01, 0x02, 0x08, 0x01, 0x58, 0x0D, 0x01, 0x02, 0x09, 0x01, 0x44, 0x02, 0x01, 0x02, 0x0A, 0x01, 0x58, 0x1F, 0x01, 0x02, 0x0B, 0x01, 0x58, 0x05, 0x01, 0x02, 0x0C, 0x01, 0x58, 0x0F, 0x01 };
    for (int i = 0; i < 55; i++)
    {
        exampleFrame.payload[i] = tempPayload[i];
    }
    
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ESP32Can.setPins(CAN_TX,CAN_RX);
  ESP32Can.setRxQueueSize(64);
  ESP32Can.setTxQueueSize(64);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  ESP32Can.begin();

  buildExampleFrame();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(ESP32Can.readFrame(rxFrame,0))
  {
    if((rxFrame.identifier & 0xF00) == 0x600) {
      KWP_FRAME_TYPE tempType = rxKwpFrame.processCanFrame(&rxFrame);
      switch(tempType)
      {
        case flowControlFrame:
          rxKwpFrame.printKwpFrame();
          exampleFrame.sendKwpFrame();
          break;
        default:
          rxKwpFrame.printKwpFrame();
          break;
      }
    }
    else { 
      //Do nothing
    }
  }
  if(Serial.available()>0) {
    Serial.read();
    exampleFrame.sendKwpFrame();
  }
}



