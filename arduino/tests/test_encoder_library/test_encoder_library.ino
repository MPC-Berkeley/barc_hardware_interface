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

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
}

void loop() {
//  drive_backward(20);
//  drive_forward(20);
Serial.println(fl.getSpeedAvg());
}

void drive_forward(int PWM_value) {
  digitalWrite(DIR, HIGH);
  analogWrite(PWM,PWM_value);
}

void drive_backward(int PWM_value) {
  digitalWrite(DIR, LOW);
  analogWrite(PWM,PWM_value);
}
