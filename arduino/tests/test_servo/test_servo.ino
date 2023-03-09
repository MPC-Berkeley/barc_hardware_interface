#include <Servo.h>
#define SteeringPin D12

#define STEERING_MAX 1900
#define STEERING_MIN 1100
#define STEERING_OFF 1500

Servo steering;
int pos = 0;

void setup() {
  // put your setup code here, to run once:
  steering.attach(SteeringPin);
}

void loop() {
   steering.writeMicroseconds(1500);
//  steering.writeMicroseconds(STEERING_OFF);
//   put your main code here, to run repeatedly:
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    steering.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    steering.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

//  for (pos = STEERING_MIN; pos <= STEERING_MAX; pos += 1) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    steering.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
//  for (pos = STEERING_MAX; pos >= STEERING_MIN; pos -= 1) { // goes from 180 degrees to 0 degrees
//    steering.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(15);                       // waits 15ms for the servo to reach the position
//  }
}
