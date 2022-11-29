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

Encoders fl(ENC_FL_A, ENC_FL_B, false);
Encoders fr(ENC_FR_A, ENC_FR_B, true);
Encoders rl(ENC_RL_A, ENC_RL_B, false);
Encoders rr(ENC_RR_A, ENC_RR_B, true);

void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
}

void loop() {
  drive_forward(60);
  Serial.print("v_fl: ");
  Serial.print(fl.getRPMAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_fr: ");
  Serial.print(fr.getRPMAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_rl: ");
  Serial.print(rl.getRPMAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_rr: ");
  Serial.println(rr.getRPMAvg(), NUM_FLOATING_POINT_DECIMALS);
}

void getSpeedAvg() {
  Serial.print("v_fl: ");
  Serial.print(fl.getSpeedAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_fr: ");
  Serial.print(fr.getSpeedAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_rl: ");
  Serial.print(rl.getSpeedAvg(), NUM_FLOATING_POINT_DECIMALS);
  Serial.print("v_rr: ");
  Serial.println(rr.getSpeedAvg(), NUM_FLOATING_POINT_DECIMALS);
}
void drive_forward(int PWM_value) {
  digitalWrite(DIR, LOW);
  analogWrite(PWM,PWM_value);
}

void drive_backward(int PWM_value) {
  digitalWrite(DIR, HIGH);
  analogWrite(PWM,PWM_value);
}
