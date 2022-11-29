#include "QuadratureEncoder2.h"

//Timer
#include "NRF52_MBED_TimerInterrupt.h"
#include "NRF52_MBED_ISR_Timer.h"
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     3
#define TIMER_INTERVAL_S        1

#define ENCODER_RESOLUTION 0.1 //10 counts per revolution == 0.1 Revolutions per count
#define WHEEL_RADIUS 0.0325 //meters
#define PI 3.14159265358979323846

NRF52_MBED_Timer ITimer(NRF_TIMER_3);
bool attachInterruptSuccess = ITimer.attachInterruptInterval(TIMER_INTERVAL_S * 1000000, Encoders::timerISR); //Not currently used: add later for debugging

//Make speed multiplier so we don't need to do this calculation everytime
static double _speed_multiplier = (ENCODER_RESOLUTION * 2*PI*WHEEL_RADIUS)/TIMER_INTERVAL_S; //[meters/(seconds*counts)]
static double _rpm_multiplier = (ENCODER_RESOLUTION/TIMER_INTERVAL_S) * 60;

// initialize all instance of encoder to null.
Encoders *Encoders::_instances[MAX_NUM_ENCODERS] = {NULL, NULL,NULL, NULL};
uint8_t Encoders::_whichEncoder = 0;

Encoders::Encoders(byte pinA, byte pinB, bool reverse){
  if (reverse) {
   _encoderPINA = pinB;
   _encoderPINB = pinA;
  } else {
   _encoderPINA = pinA;
   _encoderPINB = pinB;
  }
   pinMode(_encoderPINA, INPUT_PULLUP);  
   pinMode(_encoderPINB, INPUT_PULLUP);
   switch(_whichEncoder){
     case 0:                                                        
        attachInterrupt(_encoderPINA, interruptEncoder1A, RISING);
        attachInterrupt(_encoderPINB,  interruptEncoder1B, RISING);  
        break;
     case 1:
        attachInterrupt(_encoderPINA, interruptEncoder2A, RISING);
        attachInterrupt(_encoderPINB,  interruptEncoder2B, RISING);  
        break;
     case 2:
        attachInterrupt(_encoderPINA, interruptEncoder3A, RISING);
        attachInterrupt(_encoderPINB,  interruptEncoder3B, RISING); 
        break;
     case 3:
        attachInterrupt(_encoderPINA, interruptEncoder4A, RISING);
        attachInterrupt(_encoderPINB,  interruptEncoder4B, RISING);  
        break;
   }
   _instances[_whichEncoder] = this;
  _whichEncoder++;
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

double Encoders::getEncoderCount(){
  return this->_encoderCount;
}

double Encoders::getSpeed(){
  return _speed * _speed_multiplier;
}

void Encoders::calculateSpeed(){
  _speed = _encoderCount; //[Counts] * ([Revs/Count] * 2pi*R [m/Rev] / Interval [s]) = [m/s]
  this->_encoderCountBuffer.add(_speed);
  this->_encoderCount = 0;
}

double Encoders::getSpeedAvg() {
  return _speed * _speed_multiplier;//_encoderCountBuffer.average_value() * _speed_multiplier;
}

double Encoders::getRPMAvg() {
  return _speed *_rpm_multiplier; //[Counts] * ([Revs/Count*Minutes])
}

