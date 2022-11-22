#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

#define MAX_NUM_ENCODERS 4

class Encoders{
  public:  
    Encoders(byte pinA, byte pinB);

    static void interruptEncoder1A(){
      if(Encoders::_instances[0] != NULL)
      Encoders::_instances[0]->encoderCountA();
    }
    static void interruptEncoder1B(){
      if(Encoders::_instances[0] != NULL)
      Encoders::_instances[0]->encoderCountB();
    }
    // static void interruptEncoder2A(){
    //   if(Encoders::_instances[1] != NULL)
    //   Encoders::_instances[1]->encoderCountA();
    // }
    // static void interruptEncoder2B(){
    //   if(Encoders::_instances[1] != NULL)
    //   Encoders::_instances[1]->encoderCountB();
    // }
    // static void interruptEncoder3A(){
    //   if(Encoders::_instances[2] != NULL)
    //   Encoders::_instances[2]->encoderCountA();
    // }
    // static void interruptEncoder3B(){
    //   if(Encoders::_instances[2] != NULL)

    //   Encoders::_instances[2]->encoderCountB();
    // }
    // static void interruptEncoder4A(){
    //   if(Encoders::_instances[3] != NULL)
    //   Encoders::_instances[3]->encoderCountA();
    // }
    // static void interruptEncoder4B(){
    //   if(Encoders::_instances[3] != NULL)
    //   Encoders::_instances[3]->encoderCountB();
    // }
    
    void encoderCountA();
    void encoderCountB();
    long getEncoderCount();
    double getSpeed();
    static Encoders *_instances[MAX_NUM_ENCODERS];
    
  private:
    static uint8_t _whichEncoder;
    uint8_t _encoderPINA;
    uint8_t _encoderPINB; 
    double _ticks_per_rev = 5;
    double _radius = 0.0325; 
    volatile long _encoderCount = 0;
    volatile long _AcurrTime;
    volatile long _BcurrTime;
    volatile long _lastRisingBTime = 0;
    volatile long _lastRisingATime = 0;
};

#endif
