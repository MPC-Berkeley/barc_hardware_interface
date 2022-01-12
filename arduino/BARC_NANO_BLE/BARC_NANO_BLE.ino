#include "Servo.h"
#include <QuadratureEncoder.h>

#define DEBUG        0

#define THROTTLE_PIN D7
#define DIRECTION_PIN D6
#define MOTOR_SLEEP D8
#define MOTOR_FAULT D9
#define MOTOR_CURRENT A0
#define STEERING_PIN D5

Servo steering_servo;

#define THROTTLE_MAX 255
#define THROTTLE_MIN 0
#define THROTTLE_OFF 0


#define STEERING_MAX 1900
#define STEERING_MIN 1100
#define STEERING_OFF 1500


// Serial Port Settings
#define BAUD_RATE         115200    //Serial Port Baud Rate
#define NEWLINE           '\n'      //Serial newline char
#define byteCount         5         //Serial bytes per command
#define SERIAL_TIMEOUT    500      //Serial timeout in ms

boolean output_enabled = true;
boolean output_remote  = false;
unsigned int throttle_v = THROTTLE_OFF;
boolean throttle_dir = true;
unsigned int steering_us = STEERING_OFF;
unsigned int remote_throttle_v = THROTTLE_OFF;
boolean remote_throttle_dir = true;
unsigned int remote_steering_us = STEERING_OFF;

#define ENC_FL_A A5
#define ENC_FL_B A6
#define ENC_FR_A A2
#define ENC_FR_B A1 // ! swapped order
#define ENC_RL_A D4
#define ENC_RL_B D3
#define ENC_RR_A D11 // ! swapped order
#define ENC_RR_B D12

Encoders fr(ENC_FR_A, ENC_FR_B);
Encoders fl(ENC_FL_A, ENC_FL_B);
Encoders rl(ENC_RL_A, ENC_RL_B);
Encoders rr(ENC_RR_A, ENC_RR_B);
 
void setup() {
  Serial.begin(BAUD_RATE);
  setup_actuators();
  reset_actuators();
//  setup_tachs();
//  reset_tachs();
  
}

void loop() {
  check_serial();
  update_actuators();
  //print_tachs();
}



/* Serial communication protocol.
 *  See provided text document for details
 *  
 *  Will disable output if no input is recieved for SERIAL_TIMEOUT milliseconds
 */

void check_serial(){
  static int last_read = millis();
  static char cmdIn[byteCount];
  static boolean badRead = false;
  static byte readLength = 0;
  
  if (Serial.available() > 0){
    readLength = Serial.readBytesUntil(NEWLINE, cmdIn, byteCount+1);

    if (readLength != byteCount){
      if (DEBUG){ 
        Serial.println("Improper Message Length");
      }
      badRead = true;
    }
    else if (DEBUG){
      for (int i = 0; i<sizeof(cmdIn);i++){
        Serial.print(cmdIn[i]);
      }
      Serial.println();
    }

    last_read = millis();

    if (cmdIn[0] == 'A'){
      if (DEBUG){
        Serial.println("Detected Write Command");
      }

      unsigned int payload = (unsigned int)strtol(((char*)cmdIn)+2, 0, 10)+1000;

      if (DEBUG){
        Serial.print("Write Payload: ");
        Serial.println(payload);
      }

      switch (cmdIn[1]){
        case 'A': // enable output
          output_enabled = true;
          if (DEBUG){
            Serial.println("Output Enabled");
          }
          break;
          
        case 'B': // disable output
          output_enabled = false;
          if (DEBUG){
            Serial.println("Output Disabled");
          }
          break;
          
        case '0': // write to throttle output
          write_throttle(payload);
          break;
          
        case '1': // write to steering output
          write_steering(payload);
          break;
          
        default: 
          if (DEBUG){
            Serial.println("Unknown Write Command");
          }
      }
    }
    else if (cmdIn[0] == 'B'){
      if (DEBUG){
        Serial.println("Detected Read Command");
      }

      switch (cmdIn[1]){
        case 'A': // check if output enabled
          if (output_enabled){
            Serial.println("BA001");
          }
          else{
            Serial.println("BA000");
          }
          break;
          
        case 'B': // check if output disabled
          if (output_enabled){
            Serial.println("BB000");
          }
          else{
            Serial.println("BB001");
          }
          break;
          
        case '0': // read throttle output
          if (output_enabled){
            Serial.print("B0");
            Serial.println(throttle_v, HEX);
          }
          else{
            Serial.print("B0");
            Serial.println(THROTTLE_OFF, HEX);
          }
          break;
          
        case '1': // read steering output
          Serial.println("Read Steering output");
          if (output_enabled){
            Serial.print("B1");
            Serial.println(steering_us, HEX);
          }
          else{
            Serial.print("B1");
            Serial.println(STEERING_OFF, HEX);
          }
          break;
          
        default:
          if (DEBUG){
            Serial.println("Uknown Read Command");
          }
      }
    }
  }
  else{
    if (millis() - last_read > SERIAL_TIMEOUT){
      if (DEBUG){
        if (output_enabled){
          Serial.println("Serial Timeout, Disabling Output");
        }
      }
      
      output_enabled = false;
    }
  }
}

/* Servo control code.
 * It is intended that write_throttle and write_steering are used to change the throttle and steering high time
 * These are then applied with update_servos
 * 
 * The reason for the extra step is so that the servos can be disabled and enabled while remembering their previous command
 * to disable without remembering, send a neutral command while they are disabled.
 */


void setup_actuators(){
  steering_servo.attach(STEERING_PIN);
  pinMode(THROTTLE_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_SLEEP, OUTPUT);
  pinMode(MOTOR_FAULT, INPUT_PULLUP);
  pinMode(MOTOR_CURRENT, INPUT);
}

void reset_actuators(){
  steering_servo.writeMicroseconds(STEERING_OFF);
  digitalWrite(MOTOR_SLEEP, HIGH); // Motor driver on
  digitalWrite(DIRECTION_PIN, HIGH); // Forward by default
  analogWrite(THROTTLE_PIN, THROTTLE_OFF);
  delay(3000);
  update_actuators();
}

void update_actuators(){
  if (output_enabled){
    if (output_remote) {
      steering_servo.writeMicroseconds(remote_steering_us);
      digitalWrite(DIRECTION_PIN, remote_throttle_dir);
      analogWrite(THROTTLE_PIN, remote_throttle_v);
    }
    else {
      steering_servo.writeMicroseconds(steering_us);
      digitalWrite(DIRECTION_PIN, throttle_dir);
      analogWrite(THROTTLE_PIN, throttle_v);
    }
  }
  else{
    steering_servo.writeMicroseconds(STEERING_OFF);
    analogWrite(THROTTLE_PIN, THROTTLE_OFF);
  }
}

void write_throttle(unsigned int v){
  throttle_v = min(max(v, THROTTLE_MIN), THROTTLE_MAX);
  if (DEBUG) {
    Serial.print("Wrote Throttle ");
    Serial.println(throttle_v);
  }
}

void write_steering(unsigned int us){
  steering_us = min(max(us, STEERING_MIN), STEERING_MAX);
  if (DEBUG) {
    Serial.print("Wrote Steering ");
    Serial.println(steering_us);
  }
}
