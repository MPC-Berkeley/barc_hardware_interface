#include <Servo.h>
#include <QuadratureEncoder.h>
#include <Arduino_LSM9DS1.h>

#define DEBUG        0

#define THROTTLE_PIN D7
#define DIRECTION_PIN D6
#define MOTOR_SLEEP D8
#define MOTOR_FAULT D9
#define MOTOR_CURRENT A0
#define STEERING_PIN D5

Servo steering_servo;

#define THROTTLE_MSG_MAX 1900
#define THROTTLE_MSG_MIN 1100
#define THROTTLE_MSG_OFF 1500
#define THROTTLE_MAX 255
#define THROTTLE_MIN 0
#define THROTTLE_OFF 0
#define THROTTLE_FORWARD LOW
#define THROTTLE_BACKWARD HIGH

#define STEERING_MAX 1999
#define STEERING_MIN 1001
#define STEERING_OFF 1500

// Serial Port Settings
#define BAUD_RATE         115200    // Serial Port Baud Rate
#define NEWLINE           '\n'      // Serial newline char
#define byteCount         5         // Serial bytes per command
#define SERIAL_TIMEOUT    500      // Serial timeout in ms
#define NUM_FLOATING_POINT_DECIMALS 3 // Number of decimal places when printing floating point numbers

boolean output_enabled = true;
unsigned int throttle_v = THROTTLE_OFF;
boolean throttle_dir = THROTTLE_FORWARD;
unsigned int steering_us = STEERING_OFF;

#define ENC_FL_A A5
#define ENC_FL_B A6
#define ENC_FR_A A2
#define ENC_FR_B A1 // ! swapped order
#define ENC_RL_A D4
#define ENC_RL_B D3
#define ENC_RR_A D11 // ! swapped order
#define ENC_RR_B D12

float alpha = 0.2;
float ax, ay, az;
float ax_smooth = 0.0;
float ay_smooth = 0.0;
float az_smooth = 0.0;

double v_fr, v_fl, v_rr, v_rl;
Encoders fr(ENC_FR_A, ENC_FR_B);
Encoders fl(ENC_FL_A, ENC_FL_B);
Encoders rl(ENC_RL_A, ENC_RL_B);
Encoders rr(ENC_RR_A, ENC_RR_B);
 
void setup() {
  Serial.begin(BAUD_RATE);
  IMU.begin();
  setup_actuators();
  reset_actuators();
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
          if (output_enabled){
            Serial.print("B1");
            Serial.println(steering_us, HEX);
          }
          else{
            Serial.print("B1");
            Serial.println(STEERING_OFF, HEX);
          }
          break;

        case '2': // read accelerometer
          if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(ax, ay, az);
            ax_smooth = (1-alpha)*ax_smooth + alpha*ax;
            ay_smooth = (1-alpha)*ay_smooth + alpha*ay;
            az_smooth = (1-alpha)*az_smooth + alpha*az;
          }
          Serial.print("B2x");
          Serial.print(ax_smooth, NUM_FLOATING_POINT_DECIMALS);
          Serial.print("y");
          Serial.print(ay_smooth, NUM_FLOATING_POINT_DECIMALS);
          Serial.print("z");
          Serial.print(az_smooth, NUM_FLOATING_POINT_DECIMALS);
          break;
          
        case '3': // read encoder
          v_fr = fr.getSpeedAvg();
          v_fl = fl.getSpeedAvg();
          v_rr = rr.getSpeedAvg();
          v_rl = rl.getSpeedAvg(); 
          Serial.print("B3a");
          Serial.print(v_fl, NUM_FLOATING_POINT_DECIMALS);
          Serial.print("b");
          Serial.print(v_fr, NUM_FLOATING_POINT_DECIMALS);
          Serial.print("c");
          Serial.print(v_rl, NUM_FLOATING_POINT_DECIMALS);
          Serial.print("d");
          Serial.println(v_rr, NUM_FLOATING_POINT_DECIMALS);
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
  digitalWrite(DIRECTION_PIN, THROTTLE_FORWARD); // Forward by default
  analogWrite(THROTTLE_PIN, THROTTLE_OFF);
  delay(3000);
  update_actuators();
}

void update_actuators(){
  if (DEBUG){
    Serial.print("Output enabled: ");
    Serial.println(output_enabled);
  }
  if (output_enabled){
    steering_servo.writeMicroseconds(steering_us);
    digitalWrite(DIRECTION_PIN, throttle_dir);
    analogWrite(THROTTLE_PIN, throttle_v);
  }
  else{
    steering_servo.writeMicroseconds(STEERING_OFF);
    analogWrite(THROTTLE_PIN, THROTTLE_OFF);
  }
}

void write_throttle(unsigned int m){
  int throttle_msg = min(max(m, THROTTLE_MSG_MIN), THROTTLE_MSG_MAX);
  if (throttle_msg > THROTTLE_MSG_OFF){
    throttle_dir = THROTTLE_FORWARD;
    throttle_v = round(THROTTLE_MAX*float(throttle_msg-THROTTLE_MSG_OFF)/float(THROTTLE_MSG_MAX-THROTTLE_MSG_OFF));
  }
  else if (throttle_msg < THROTTLE_MSG_OFF){
    throttle_dir = THROTTLE_BACKWARD;
    throttle_v = round(THROTTLE_MAX*float(THROTTLE_MSG_OFF-throttle_msg)/float(THROTTLE_MSG_OFF-THROTTLE_MSG_MIN));
  }
  else {
    throttle_dir = THROTTLE_FORWARD;
    throttle_v = THROTTLE_OFF;
  }
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
