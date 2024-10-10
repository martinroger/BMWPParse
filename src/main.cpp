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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ESP32Can.setPins(CAN_TX,CAN_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  ESP32Can.begin();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(ESP32Can.readFrame(rxFrame,0))
  {
    if((rxFrame.identifier & 0xF00) == 0x600) {
      rxKwpFrame.processCanFrame(&rxFrame);
      rxKwpFrame.printKwpFrame();
    }
    else { 
      //Do nothing
    }
  }
}

