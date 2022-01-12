#define ENC_RR_A D12
#define ENC_RR_B D11
#define ENC_RL_A D4
#define ENC_RL_B D3
#define ENC_FR_A A1
#define ENC_FR_B A2
#define ENC_FL_A A5
#define ENC_FL_B A6

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
  Serial.begin(115200);
}
void loop() {
  // put your main code here, to run repeatedly:
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
  
  delay(10);
}
