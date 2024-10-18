#include "kwpFrame2.h"

/// @brief 
/// @param _target 
/// @param _sender 
/// @param _SID 
/// @param _length 
/// @param _bufferLength 
/// @param _rxComplete 
/// @param _multiFrame 
kwpFrame::kwpFrame(byte _target, byte _sender, byte _SID, uint16_t _length, uint16_t _bufferLength, bool _rxComplete, bool _multiFrame)
{
    target = _target;
    sender = _sender;
    SID = _SID;
    length = _length;
    bufferLength = _bufferLength;
    rxComplete = _rxComplete;
    multiFrame = _multiFrame;
}

/// @brief 
/// @param _target 
/// @param _sender 
/// @param _SID 
/// @param _bufferLength 
kwpFrame::kwpFrame(byte _target, byte _sender, byte _SID, uint16_t _bufferLength)
{
    kwpFrame(_target,_sender,_SID,_bufferLength+1,_bufferLength);
}

/// @brief Parses Metadata on first reception frame when in an RX position. Unused in TX
/// @param canMetaFrame FirstFrame or SingleFrame (no Flow Control) to get the metadata from
void kwpFrame::setMetadaData(twai_message_t *canMetaFrame)
{
    target = canMetaFrame->data[0];
    sender = canMetaFrame->identifier & 0xFF;
    switch ((canMetaFrame->data[1]) & 0xF0)
    {
    case 0x00: //SingleFrame
        multiFrame = false;
        break;
    case 0x10: //FirstFrame
        multiFrame = true;
        break;
    default:
        multiFrame = false;
        break;
    }
}

/// @brief After constructing a KWP Frame, calculates all metadata variants
void kwpFrame::calculateMetaData()
{
    if(length>6) multiFrame = true;
    else multiFrame = false;
}

/// @brief Appends a segment of a CAN Frame buffer to an existing kwpFrame payload buffer
/// @param canFrame Donor CAN Frame, referenced by pointer
/// @param startPos Start position, by default 2
/// @param endPos End position, by default 7
/// @return True if the append is possible, false if sanity checks are failed
bool kwpFrame::appendCanFrameBuffer(twai_message_t *canFrame, uint8_t startPos, uint8_t endPos)
{
    //Some sanity check
    if( (startPos>endPos)   || 
        (endPos>7)          || 
        (canFrame==nullptr) || 
        ((cursor + (endPos-startPos+1)) > bufferLength)) return false;
    else
    {
        for (int i = startPos; i < endPos+1; i++)
        {
            buffer[cursor] = canFrame->data[i];
            cursor++;
        }
        return true;
    }
}

/// @brief Resets frame to a fresh state
void kwpFrame::reset(byte _target, byte _sender)
{
    target          =   _target;
    sender          =   _sender;
    SID             =   0x00;
    length          =   2;
    bufferLength   =   1;
    cursor          =   0;
    rxComplete      =   true;
    for (int i = 0; i < 255; i++)
    {
        buffer[i] = 0x00;
    }
}
