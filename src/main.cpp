#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>

#define CAN_TX D2
#define CAN_RX D1

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

/*Usage : 
tempTrackIndex = swap_endian<uint32_t>(*((uint32_t*)&byteArray[3]));
*/

CanFrame rxFrame = {0};

enum frameType : byte
{
    singleFrame         =   0x00,
    firstFrame          =   0x10,
    continuationFrame   =   0x20,
    flowControlFrame    =   0x30
};

byte target;
byte sender;
byte SID;
uint16_t payloadLen;
uint16_t otherLen;
byte cursor;
byte kwpBuf[255];
bool frameComplete = true;

frameType rxFrameType = singleFrame;

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
    //Serial.println("Frame received");
    //Serial.printf("ID: 0x%03x",rxFrame.identifier);
    //Serial.printf("\tDLC: %d",rxFrame.data_length_code);
    //Serial.print("\tPayload:");
    // for (int i = 0; i < rxFrame.data_length_code; i++)
    // {
    //   Serial.printf("\t%02x",rxFrame.data[i]);
    // }
    // Serial.println();

    if((rxFrame.identifier & 0xF00) == 0x600) {
      //Sender and target are systematically sent
      sender = rxFrame.identifier & 0xFF;
      target = rxFrame.data[0];

      rxFrameType = (frameType)(rxFrame.data[1] & 0xF0); //Dangerous

      switch (rxFrameType)
      {
      case singleFrame:
        if(!frameComplete) {
          Serial.println("ERROR: SingleFrame received while another frame is not complete");
        }
        else {
          payloadLen = (rxFrame.data[1] & 0x0F) - 1;
          SID = rxFrame.data[2];
          cursor = 0;
          for (int i = 3; i < 3 + payloadLen; i++) //There are three (RX LEN SID) bytes before the actual payload
          {
            kwpBuf[cursor] = rxFrame.data[i];
            cursor++;
          }
          //Print the frame to Serial
          Serial.printf("SF TX: %02X RX: %02X SID: %02X LEN: %3d |  ",sender,target,SID,payloadLen); //Would be better to push it in a KWP Frame
          for (int i = 0; i < payloadLen; i++)
          {
            Serial.printf("%02X ",kwpBuf[i]);
          }
          Serial.println();
        }
        break;
      
      case firstFrame:
        if(!frameComplete) {
          Serial.println("ERROR: SingleFrame received while another frame is not complete");
        }
        else {
          payloadLen = (swap_endian<uint16_t>(*((uint16_t*)&(rxFrame.data[1]))) & 0x0FFF) -1; //Remove the byte used by the SID
          SID = rxFrame.data[3];
          Serial.printf("FF TX: %02X RX: %02X SID: %02X LEN: %3d ...\n ",sender,target,SID,payloadLen);
          frameComplete = false;
          cursor = 0;
          for (int i = 4; i < 8; i++)
          {
            kwpBuf[cursor] = rxFrame.data[i];
            cursor++;
          }        
        }
        break;
      case continuationFrame: //Multiframes
        if(!frameComplete) { //Expected case
          for (int i = 2; i < 8; i++)
          {
            if(cursor<payloadLen) {
              kwpBuf[cursor] = rxFrame.data[i];
              cursor++;
            }
            else { //All the transferring has happened
              frameComplete = true;
              Serial.printf("MF TX: %02X RX: %02X SID: %02X LEN: %3d |  ",sender,target,SID,payloadLen); //Would be better to push it in a KWP Frame
              for (int i = 0; i < payloadLen; i++)
              {
                Serial.printf("%02X ",kwpBuf[i]);
              }
              Serial.println(); 
              break; //Don't forget to eject from the for-loop
            }
          }
        }
        else { //Somehow receive a ContinuationFrame without FirstFrame
          Serial.println("ERROR: ContinuationFrame without FirstFrame");
        }
        break;

      case flowControlFrame: 
        Serial.printf("FC TX: %02X RX: %02X\n",sender,target); 
        break;
      default:
        break;
      }

    }
    else { //Not a 6xx Frame

    }
  }
}

