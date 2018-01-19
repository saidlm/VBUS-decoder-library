/*
 * Resol VBUS decoder library v1.0
 * Created by Martin Saidl January, 2018
 */
#include "Arduino.h"
#include "vbusdecoder.h"

VBUSDecoder::VBUSDecoder(Stream* serial):
  _stream(serial),
  _temp{0},
  _relay{0},
  _pump{0},
  _tempNum(0),
  _relayNum(0),
  _pumpNum(0),
  _errorFlag(false),
  _readyFlag(false),
  _rcvBuffer{0},
  _rcvBufferIdx(0),
  _state(SYNC)
  {}  

VBUSDecoder::~VBUSDecoder()
  {}
  
void VBUSDecoder::begin() {
  _lastMillis = millis();
  _state = SYNC;
}

;

void VBUSDecoder::loop() {
    switch (_state) {
    case SYNC:
      _syncHandler();
      break;
    case RECEIVE:
      _receiveHandler();
      break;
    case DECODE:
      _decodeHandler();
      break;
    case ERROR:
      _errorHandler();
      break;
    default:
      _syncHandler();
      break;
  } 
}

const float VBUSDecoder::getTemp(uint8_t idx) const {
  return _temp[idx];
}

const uint8_t VBUSDecoder::getPump(uint8_t idx) const {
  return _pump[idx];  
}

const bool VBUSDecoder::getRelay(uint8_t idx) const {
  return _relay[idx];
}

uint8_t const VBUSDecoder::getTempNum() const {
  return _tempNum;
}

uint8_t const VBUSDecoder::getPumpNum() const {
  return _pumpNum;
}

uint8_t const VBUSDecoder::getRelayNum() const {
  return _relayNum;
}

const bool VBUSDecoder::getVbusStat() const {
  return !_errorFlag;
}

const bool VBUSDecoder::isReady() const {
  return _readyFlag;
}

// CRC calculator - comming from: http://danielwippermann.github.io/resol-vbus/vbus-specification.html
uint8_t VBUSDecoder::_calcCRC(const uint8_t *Buffer, uint8_t Offset, uint8_t Length) {
    uint8_t Crc;
    uint8_t i;
    Crc = 0x7F;
    for (i = 0; i < Length; i++) {
        Crc = (Crc - Buffer [Offset + i]) & 0x7F; 
    }
    return Crc;
}

// Septed injection - comming from: http://danielwippermann.github.io/resol-vbus/vbus-specification.html
void VBUSDecoder::_septetInject(uint8_t *Buffer, uint8_t Offset, uint8_t Length) {
    uint8_t Septett;
    uint8_t i;

    Septett = Buffer [Offset + Length];
    for (i = 0; i < Length; i++) {
        if (Septett & (1 << i)) {
            Buffer [Offset + i] |= 0x80;
        }
    }
}

// Temperature calculation - comming from: https://github.com/bbqkees/vbus-arduino-domoticz/blob/master/ArduinoVBusDecoder.ino
// It converts 2 data bytes to temperature
float VBUSDecoder::_calcTemp(uint8_t Byte1, uint8_t Byte2) {
  int16_t v;
  v = Byte1 << 8 | Byte2;

  return ((float)v * 0.1);
}

// Header decoder
// Grab header data from frame and stores them in to frame structure
void VBUSDecoder::_headerDecoder() {
  _dstAddr = (_rcvBuffer[1] << 8) | _rcvBuffer[0];
  _srcAddr = (_rcvBuffer[3] << 8) | _rcvBuffer[2];
  _protocol = (_rcvBuffer[4] >> 4) + (_rcvBuffer[4] & (1<<15)); 
  _cmd = (_rcvBuffer[6] << 8) | _rcvBuffer[5];
  _frameCnt = _rcvBuffer[7];
  _frameLen = _rcvBuffer[7] * 6 + 10;  
}

//Sync handler
void VBUSDecoder::_syncHandler() {
  if (millis() - _lastMillis > 20 * 1000UL) // if no packet arrived in last 20 sec go to error state
    _state = ERROR;
  if (_stream->available() > 0)
    if (_stream->read() == 0xaa) { // Sync byte has been received
      _rcvBufferIdx = 0;
      _dstAddr = 0;
      _srcAddr = 0;
      _protocol = 0;
      _cmd = 0;
      _frameCnt = 0;
      _frameLen = 0;
      _state = RECEIVE;
    }  
}

// Receiving handler
void VBUSDecoder::_receiveHandler() {
  uint8_t crc;

  while (_stream->available() > 0) {
    uint8_t rcvByte = _stream->read();

    // MSB is set - according to protocol description the receiving has to be stopped
    if (rcvByte >= 0x80) { 
      _state = ERROR;
      return;
    }
    
    _rcvBuffer[_rcvBufferIdx] = rcvByte;
    _rcvBufferIdx++;  
  }

  // Test if there is frame header stored in receive buffer
  if ((_rcvBufferIdx > 10) && (_frameCnt == 0)) {  
    _headerDecoder();

    crc = _calcCRC(_rcvBuffer, 0, 9);

    // Only protocol 1.0 will be decoded
    if (_protocol != 1) {
      _state = SYNC;
      return; 
    }
      
    // if CRC fails go to ERROR state
    if (crc != 0) {
      _state = ERROR;
      return;
    }
    _errorFlag = false;
  }


  // Test if whole frame has been already received
  if ((_rcvBufferIdx == _frameLen - 1)) {
    for (uint8_t i=0: i < _frameCnt) {
      crc = _calcCRC(_rcvBuffer, (i * 6) + 10, 6);
      
      // Go to error state if CRC fails
      if  (crc != 0) {
        _state = ERROR;
        return;
      }   
    }
    _lastMillis = millis();
    _state = DECODE;  
  }
}

// Decoder handler
void VBUSDecoder::_decodeHandler() {

  // Only packets carring comman 0x0100 - Master to slave are in focuse
  if (_cmd == 0x0100) {
	Serial.println("cmd 100");

    switch (_srcAddr) {
      case 0x1060:  // Vitosolic 200
      _vitosolic200Decoder();
      break;
      defalut: // General RESOL device
      _defaultDecoder();
    }
    _readyFlag =  true;
    _state = SYNC;  
  }
}

// Error handler
void VBUSDecoder::_errorHandler() {
  _errorFlag = true;
  _readyFlag = false;
  _state = SYNC;
}


// Frame decoders for unique devices
// thank to Bbqkees - https://github.com/bbqkees/vbus-arduino-domoticz/blob/master/ArduinoVBusDecoder.ino

// Default decoder for commn RESOL devices
void VBUSDecoder::_defaultDecoder() {

  // Default temp 1-4 extraction
  // For most Resol controllers temp 1-4 are always available, so
  // even if you do not know the datagram format you can still see
  // these temps 1 to 4.

  //Offset  Size    Mask    Name                    Factor  Unit
  // Frame 1
  //0       2               Temperature sensor 1    0.1     °C
  //2       2               Temperature sensor 2    0.1     °C
  // Frame 2
  //4       2               Temperature sensor 3    0.1     °C
  //6       2               Temperature sensor 4    0.1     °C
  //
  // Each frame has 6 bytes
  // byte 1 to 4 are data bytes -> MSB of each bytes
  // byte 5 is a septet and contains MSB of bytes 1 to 4
  // byte 6 is a checksum
  //
  //*******************  Frame 1  *******************
  _tempNum = 4;
  
  _rcvBufferIdx = 9;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  // Temperature Sensor 1, 15 bits, factor 0.1 in C
  _temp[0] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  // Temperature sensor 2, 15 bits, factor 0.1 in C
  _temp[1] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  //*******************  Frame 2  *******************
  _rcvBufferIdx = 15;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[2] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[3] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  ///******************* End of frames ****************
}

// Vitosolic 200
void VBUSDecoder::_vitosolic200Decoder() {

  //Offset  Mask        Name                Factor      Unit
  //0                   Temperature S1      1.0         °C
  //1                   Temperature S1      256.0       °C
  //2                   Temperature S2      1.0         °C
  //3                   Temperature S2      256.0       °C
  //4                   Temperature S3      1.0         °C
  //5                   Temperature S3      256.0       °C
  //6                   Temperature S4      1.0         °C
  //7                   Temperature S4      256.0       °C
  //8                   Temperature S5      1.0         °C
  //9                   Temperature S5      256.0       °C
  //10                  Temperature S6      1.0         °C
  //11                  Temperature S6      256.0       °C
  //12                  Temperature S7      1.0         °C
  //13                  Temperature S7      256.0       °C
  //14                  Temperature S8      1.0         °C
  //15                  Temperature S8      256.0       °C
  //16                  Temperature S9      1.0         °C
  //17                  Temperature S9      256.0       °C
  //18                  Temperature S10     1.0         °C
  //19                  Temperature S10     256.0       °C
  //20                  Temperature S11     1.0         °C  
  //21                  Temperature S11     256.0       °C
  //22                  Temperature S12     1.0         °C
  //23                  Temperature S11     256.0       °C

  //44                  Relay 1             1           %
  //45                  Relay 2             1           %
  //46                  Relay 3             1           %
  //47                  Relay 4             1           %
  //48                  Relay 5             1           %
  //49                  Relay 6             1           %
  //50                  Relay 7             1           %
  
  // Each frame has 6 bytes
  // byte 1 to 4 are data bytes -> MSB of each bytes
  // byte 5 is a septet and contains MSB of bytes 1 to 4
  // byte 6 is a checksum
  //
  //**************************************************
  
  _tempNum = 12;
  _relayNum = 7;
  _pumpNum = 4;
  
  _rcvBufferIdx = 9;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[0] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[1] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 15;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[2] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[3] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 21;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[4] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[5] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 27;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[6] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[7] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 33;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[8] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[9] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 39;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);

  _temp[10] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 1], _rcvBuffer[_rcvBufferIdx]);
  _temp[11] = _calcTemp(_rcvBuffer[_rcvBufferIdx + 3], _rcvBuffer[_rcvBufferIdx + 2]);

  _rcvBufferIdx = 75;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4); 
  
  _pump[0] =  _rcvBuffer[_rcvBufferIdx] & 0X7F;
  _pump[1] =  _rcvBuffer[_rcvBufferIdx + 1] & 0X7F;
  _pump[2] =  _rcvBuffer[_rcvBufferIdx + 2] & 0X7F;
  _pump[3] =  _rcvBuffer[_rcvBufferIdx + 3] & 0X7F;

  _rcvBufferIdx = 81;
  _septetInject(_rcvBuffer, _rcvBufferIdx, 4);
  
  _pump[4] =  _rcvBuffer[_rcvBufferIdx] & 0X7F;
  _pump[5] =  _rcvBuffer[_rcvBufferIdx + 1] & 0X7F;
  _pump[6] =  _rcvBuffer[_rcvBufferIdx + 2] & 0X7F;

  for (uint8_t i = 0; i < 7; i++)
    _relay[i] = (_pump[i] == 0x64)?true:false;
  ///******************* End of frames ****************
}

