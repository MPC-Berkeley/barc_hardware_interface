#include <QuadratureEncoder.h>

#define BUFFER_SIZE 256
/***** encoder *******/
#define ENC_FL_A A5
#define ENC_FL_B A6
#define ENC_FR_A A2
#define ENC_FR_B A1 // ! swapped order
#define ENC_RL_A D4
#define ENC_RL_B D3
#define ENC_RR_A D11 // ! swapped order
#define ENC_RR_B D12

#define SAMPLING_TIME 100
#define TIMEOUT 300

/***** motor *******/
#define DIR D6
#define PWM D7
#define SLP D8
#define FLT D9
#define CS A0

#define STEERING_PIN D5

/*** voltage divider ***/
#define VBATT_pin D1 // ! change me 
#define NUM_SAMPLES 10
const double R1 = 680e3, R2 = 330e3 ;
double V_batt[256] ; 
byte v_bat_idx ; 
/***********************/
/******* variables definitions *******/

unsigned long lastMilli = 0;

//encoder
double v_fr, v_fl, v_rr, v_rl, v_avg ; 
Encoders fr(ENC_FR_A, ENC_FR_B);
Encoders fl(ENC_FL_A, ENC_FL_B);
Encoders rl(ENC_RL_A, ENC_RL_B);
Encoders rr(ENC_RR_A, ENC_RR_B);


// PID Controller
/************** PID Class ***************/
class PID{
    private: 

      double _kp, _ki, _kd ; 
      double _u, _u_max, _u_min;
      long _previousTime, _currentTime ; 
      double _last_e = 0, _cum_e = 0, _der_e = 0; 
      double _ref = 0 ; 

      double _anti_windup(double y){
        double new_e ; 
        double ur = _cum_e * _ki;
        double e = _ref - y;

        if ( (ur > _u_max && _ki * e > 0) 
        || ( ur < _u_min && _ki * e < 0) ){
          new_e = 0 ; 
        }
        else{
          new_e = e ;
        }
        
        return new_e;
      }
      
      double _coerce_limits(double u){
        return min(max(u, _u_min), _u_max) ;
      }

    public: 

      PID(double kp, double ki, double kd, double u_min, double u_max){
          
          _kp = kp ; _ki = ki ; _kd = kd ; 
          _u_min = u_min ; _u_max = u_max ; 
          _cum_e = 0 ; 
          _previousTime = micros() ; 
          _currentTime = micros() ; 

      }

      void set_ref(double ref){
        _ref = ref ; 
      }

      double step(double y){
        double e = _ref - y ; 
        double dt = (double)(micros() - _previousTime)/1E6 ;

        _cum_e += _anti_windup(y) * dt ; 
        _cum_e = (isnan(_cum_e)) ? 0 : _cum_e ; 

        
        
        // _der_e = (_last_e - e) / dt ; 
    
        double u = _kp * e + _ki * _cum_e ; // _kd * _der_e
        u = _coerce_limits(u) ; 

        
        _last_e = e ;
        _previousTime = _currentTime ; 
        _currentTime = micros() ; 
        
        return u ; 
      }

}; 

/****************************************/

PID pid_throttle(10, 10, 0, 0, 100) ;
PID pid_steer(0, 0, 0, 0, 255) ; // ! change me 

//ref data type 
typedef struct ref_t{
  double v = 0 ; 
  double y = 0; 
}ref_t ; 

ref_t ref ; 

//input data type
typedef struct u_t{
  int throttle = 0 ; 
  int steer = 0 ; 
}u_t;

u_t u ; 

//receiver variables
char receivedData[100]; //creates variable to store data from jetson (100 is byte size)
char handshake = '&';
int last_received = 0;
bool serial_started = false;

/*************************************/
 
void setup() {
  pid_throttle.set_ref(0.5); 
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(FLT, INPUT_PULLUP);
  pinMode(CS, INPUT);
  digitalWrite(SLP, HIGH); //Keep motor driver ON
  Serial.begin(115200);
  
}


void loop() {

  /********* motor ***********/
   analogWrite(PWM, u.throttle);
   digitalWrite(DIR, LOW);
  /**************************/
  
  /********* encoder ***********/
  v_fr = fr.getSpeedAvg() ;
  v_fl = fl.getSpeedAvg() ;
  v_rr = rr.getSpeedAvg() ;
  v_rl = rl.getSpeedAvg() ; 

  v_avg = (v_fr + v_rr + v_rl) / 3 ;

  u.throttle = pid_throttle.step(v_avg) ; 

  Serial.print("V_avg: ");
  Serial.print(v_avg);
  Serial.print(", V_fr: ");
  Serial.print(v_fr);
  Serial.print(", V_fl: ");
  Serial.print(v_fl);
  Serial.print(", V_rr: ");
  Serial.print(v_rr);
  Serial.print(", V_rl: ");
  Serial.print(v_rl);
  Serial.print(" , u_throttle: ");
  Serial.println(u.throttle);
//   u.steer = pid_steer.step(v_avg) ; 
  /*****************************/

  /*** serial communication  ***/
//  ref = parse_command() ; 

//  pid_throttle.set_ref(ref.v) ; 
  // pid_steer.set_ref(ref.y) ; 
//  Serial.println(ref.v) ; 
  /*****************************/

}


/*****************************/
/*     helper functions      */
/*****************************/



/********* serial communication *********/
ref_t parse_command(){
    ref_t ref ;  

    // FORMAT: "HANDSHAKE" "SPEED*100" "STEERANGLE*100"
    // (neutral formatting): & 0 0
    if (SerialUSB.available()) {
      byte size = SerialUSB.readBytesUntil('\r', receivedData, 50); //reads serial data into buffer and times out after 100ms
      receivedData[size] = 0; //end of the string can be specified with a 0.
      char *s = strtok(receivedData, " "); //allows string to be broken into tokens by " ".
      if (s[0] == handshake) {
        s = strtok(NULL, " ");
        if (s != NULL) ref.v = (double) atoi(s) / 100 ; //sets variable to received data and converts ASCII to integer if message is not empty
        s = strtok(NULL, " ");
        if (s != NULL) ref.y = (double) atoi(s) / 100; //sets variable to received data and converts ASCII to integer if message is not empty
      }
      last_received = millis();
      serial_started = true;
    }
    else if(millis() - last_received > TIMEOUT && serial_started){
      ref.v = 0;
      ref.y = 0;
    }

    return ref ; 

}

double 

/****************************************/
