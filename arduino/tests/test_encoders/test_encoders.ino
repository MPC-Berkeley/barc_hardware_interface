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

void setup() {
  // put your setup code here, to run once:
  pinMode(ENC_RR_A, INPUT_PULLUP);
  pinMode(ENC_RR_B, INPUT_PULLUP);
  pinMode(ENC_RL_A, INPUT_PULLUP);
  pinMode(ENC_RL_B, INPUT_PULLUP);
  pinMode(ENC_FR_A, INPUT_PULLUP);
  pinMode(ENC_FR_B, INPUT_PULLUP);
  pinMode(ENC_FL_A, INPUT_PULLUP);
  pinMode(ENC_FL_B, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
  Serial.begin(115200);
}
void loop() {
  drive_backward(25);
  log_data();
}

void log_data() {
  Serial.print(digitalRead(ENC_FR_A));
  Serial.print(",");
  Serial.print(digitalRead(ENC_FR_B));
  Serial.print(",");
  Serial.println(millis());
}

void print_readings() {
  Serial.println("Front Left:");
  Serial.print(digitalRead(ENC_FL_A));
  Serial.print(",");
  Serial.println(digitalRead(ENC_FL_B));
  
  Serial.println("Front Right:");
  Serial.print(digitalRead(ENC_FR_A));
  Serial.print(",");
  Serial.println(digitalRead(ENC_FR_B));

  Serial.println("Rear Left:");
  Serial.print(digitalRead(ENC_RL_A));
  Serial.print(",");
  Serial.println(digitalRead(ENC_RL_B));

  Serial.println("Rear Right:");
  Serial.print(digitalRead(ENC_RR_A));
  Serial.print(",");
  Serial.println(digitalRead(ENC_RR_B));
}

void drive_forward(int PWM_value) {
  digitalWrite(DIR, LOW);
  analogWrite(PWM,PWM_value);
}

void drive_backward(int PWM_value) {
  digitalWrite(DIR, HIGH);
  analogWrite(PWM,PWM_value);
}
