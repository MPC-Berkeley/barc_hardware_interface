#include "QuadratureEncoder2.h"

// initialize all instance of encoder to null.
Encoders *Encoders::_instances[MAX_NUM_ENCODERS] = {NULL, NULL,NULL, NULL};
uint8_t Encoders::_whichEncoder = 0;

Encoders::Encoders(byte pinA, byte pinB){
   _encoderPINA = pinA;
   _encoderPINB = pinB;
   pinMode(_encoderPINA, INPUT_PULLUP);  
   pinMode(_encoderPINB, INPUT_PULLUP);
   
   // _whichEncoder++;
   // switch(_whichEncoder){
   //  case 1:
        attachInterrupt(_encoderPINA, interruptEncoder1A, RISING);
        attachInterrupt(_encoderPINB,  interruptEncoder1B, RISING);  
        _instances[0] = this;
   //      break;
   //   case 2:
   //      attachInterrupt(_encoderPINA, interruptEncoder2A, RISING);
   //      attachInterrupt(_encoderPINB,  interruptEncoder2B, CHANGE);  
   //      _instances[1] = this;
   //      break;
   //   case 3:
   //      attachInterrupt(_encoderPINA, interruptEncoder3A, RISING);
   //      attachInterrupt(_encoderPINB,  interruptEncoder3B, CHANGE); 
   //      _instances[2] = this; 
   //      break;
   //   case 4:
   //      attachInterrupt(_encoderPINA, interruptEncoder4A, RISING);
   //      attachInterrupt(_encoderPINB,  interruptEncoder4B, CHANGE);  
   //      _instances[3] = this;
   //      break;
   // }
}

void Encoders::encoderCountA() {
    long _prevTime = this->_AcurrTime;
    _AcurrTime = micros();
    this->_lastRisingATime = _AcurrTime;
    long _50dutyTime = _prevTime + (_AcurrTime - _prevTime)*0.5;
    if (_50dutyTime > _lastRisingBTime) {
      this->_encoderCount++;
    } else {
      this->_encoderCount--;
    }
}

void Encoders::encoderCountB() {
  long _prevTime = this->_BcurrTime;
  _BcurrTime = micros();
  this->_lastRisingBTime = micros();
  long _50dutyTime = _prevTime + (_BcurrTime - _prevTime)*0.5;
  if (_50dutyTime > _lastRisingATime) {
    this->_encoderCount--;
  }
  else{
    this->_encoderCount++;
  }
}

long Encoders::getEncoderCount(){
  return this->_encoderCount;
}

double Encoders::getSpeed(){
  double speed = _encoderCount;
  this->_encoderCount = 0;
  return speed;
}

