#define DIR D5
#define PWM D4
#define SLP D3
#define FLT D2
#define CS A7

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(FLT, INPUT_PULLUP);
  pinMode(CS, INPUT);
  Serial.begin(115200);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
}

void loop() {
  //Drive Forward
  analogWrite(PWM, 30);
  digitalWrite(DIR, LOW);
//  delay(2000);
//
//
//  //Drive backward
//  analogWrite(PWM, );
//  digitalWrite(DIR, LOW);
//  delay(2000);
}
