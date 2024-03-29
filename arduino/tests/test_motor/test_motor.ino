#define DIR D6
#define PWM D7
#define SLP D8
#define FLT D9
#define CS A0

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
  analogWrite(PWM, 50);
  digitalWrite(DIR, HIGH);
  delay(2000);


  //Drive backward
//  analogWrite(PWM, 50);
//  digitalWrite(DIR, LOW);
//  delay(2000);
}
