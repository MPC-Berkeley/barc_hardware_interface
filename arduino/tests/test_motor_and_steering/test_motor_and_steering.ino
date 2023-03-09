#include <Servo.h>
#define SteeringPin D12

#define STEERING_MAX 1900
#define STEERING_MIN 1100
#define STEERING_OFF 1500

#define DIR D5
#define PWM D4
#define SLP D3
#define FLT D2
#define CS A7

Servo steering;
int pos = 0;

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(FLT, INPUT_PULLUP);
  pinMode(CS, INPUT);
  Serial.begin(115200);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
  steering.attach(SteeringPin);
}

void loop() {
  //Drive Forward
  analogWrite(PWM, 0);
  digitalWrite(DIR, LOW);
//  delay(2000);
//
//
//  //Drive backward
//  analogWrite(PWM, );
//  digitalWrite(DIR, LOW);
//  delay(2000);

     steering.writeMicroseconds(1500);
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    steering.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    steering.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
