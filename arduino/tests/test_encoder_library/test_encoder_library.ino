#include <QuadratureEncoder2.h>

// Serial Port Settings
#define BAUD_RATE         115200    // Serial Port Baud Rate
#define NUM_FLOATING_POINT_DECIMALS 3 // Number of decimal places when printing floating point numbers

#define ENC_RR_A D6
#define ENC_RR_B D7
#define ENC_RL_A A6
#define ENC_RL_B A5
#define ENC_FR_A D10
#define ENC_FR_B D9
#define ENC_FL_A A1
#define ENC_FL_B A2

#define DIR D5
#define PWM D4
#define SLP D3

Encoders fl(ENC_FL_A, ENC_FL_B);

//Timer Settings
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     3
#include "NRF52_MBED_TimerInterrupt.h"
#define TIMER0_INTERVAL_MS        1000
NRF52_MBED_Timer ITimer0(NRF_TIMER_3);

double currentSpeed = 0;

void TimerHandler0() {
  currentSpeed = fl.getSpeed();
}
 
void setup() {
  Serial.begin(BAUD_RATE);
  if (!ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)) {
    Serial.println("Can't set ITimer0. Select another freq. or timer");
  }
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
}

void loop() {
  Serial.println(currentSpeed);
  drive_backward(20);
//  drive_forward(20);
//Serial.println(fl.getEncoderCount());
}

void drive_forward(int PWM_value) {
  digitalWrite(DIR, HIGH);
  analogWrite(PWM,PWM_value);
}

void drive_backward(int PWM_value) {
  digitalWrite(DIR, LOW);
  analogWrite(PWM,PWM_value);
}
