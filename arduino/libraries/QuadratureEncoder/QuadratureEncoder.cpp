// #include <EnableInterrupt.h>
#include "QuadratureEncoder.h"

//arduino BLE isn't compatible with EnableInterrupts library

// initialize all instance of encoder to null.

Encoders *Encoders::_instances[MAX_NUM_ENCODERS] = {NULL, NULL,NULL, NULL};
uint8_t Encoders::_whichEncoder = 0;
Encoders::Encoders(byte pinA, byte pinB){
   _encoderPINA = pinA;
   _encoderPINB = pinB;
   pinMode(_encoderPINA, INPUT_PULLUP);  
   pinMode(_encoderPINB, INPUT_PULLUP);
   _whichEncoder++;
   switch(_whichEncoder){
    case 1:
        attachInterrupt(_encoderPINB, interruptEncoder1, CHANGE);
        attachInterrupt(_encoderPINA,  interruptEncoder1, CHANGE);  
        _instances[0] = this;
        break;
     case 2:
        attachInterrupt(_encoderPINB, interruptEncoder2, CHANGE);
        attachInterrupt(_encoderPINA,  interruptEncoder2, CHANGE);  
        _instances[1] = this;
        break;
     case 3:
        attachInterrupt(_encoderPINB, interruptEncoder3, CHANGE);
        attachInterrupt(_encoderPINA,  interruptEncoder3, CHANGE); 
        _instances[2] = this; 
        break;
     case 4:
        attachInterrupt(_encoderPINB, interruptEncoder4, CHANGE);
        attachInterrupt(_encoderPINA,  interruptEncoder4, CHANGE);  
        _instances[3] = this;
        break;
   }
}

void Encoders::encoderCount(){
  int EncoderPhaseA = digitalRead(this->_encoderPINA);  // MSB
  int EncoderPhaseB = digitalRead(this->_encoderPINB);  // LSB
  
  _prevTime = _currTime ; 
  _currTime = micros() ; 
  _prevCount = _encoderCount ; 
  _timeBuffer[_index] = _currTime ; 

  int currentEncoded = (EncoderPhaseA << 1) | EncoderPhaseB;
  int sum = (this->_lastEncoded << 2) | currentEncoded;
  switch(sum){
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      this->_encoderCount--;
      this->_countBuffer[this->_index] = _encoderCount ; 
      break;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      this->_encoderCount++;
      this->_countBuffer[this->_index] = _encoderCount ; 
      break;
    default:
      this->_encoderErrors++;
      break;
  }
  this->_lastEncoded = currentEncoded;
  this->_index++ ;
}


long Encoders::getEncoderCount(){
  return _encoderCount;
}
void Encoders::setEncoderCount(long setEncoderVal){
  this->_encoderCount = setEncoderVal;
}

long Encoders::getEncoderErrorCount(){
  return _encoderErrors;
}

long Encoders::getEncoderTime(){
  return _currTime ;
}

double Encoders::getEncoderdT(){
  return (double) (_currTime - _prevTime)  ; /// 1E6
}

double Encoders::getSpeed(){
  return (double) (_encoderCount - _prevCount) / ((double) (_currTime - _prevTime) / 1E6) * _resolution * _radius / 4  ;
}

double Encoders::getSpeedAvg(){ 
  double v = 0, dx = 0, dt = 0 ; 
  uint8_t t0 = _index - 12 ;
  uint8_t tf = _index - 2 ; 
  uint8_t aux = 0; 
  for (uint8_t i = 1; i < 10 ; i++) {
    aux = t0 + i ; 
    dx += _countBuffer[aux] - _countBuffer[aux-1] ;
    dt += _timeBuffer[aux] - _timeBuffer[aux-1] ;

  }

  v = (double) dx / (double) dt * 10E5 * _resolution * _radius / 4; 
  v = (isnan(v)) ? 0 : v ; 
  return  v; 
}
