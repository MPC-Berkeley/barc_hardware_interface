//*************************************************************
//IF THIS DOES NOT COMPILE, SET BOARD TO ARDUINO DUE!!!!!!!!!!! 
//*************************************************************

// Setup: 
// Step 1: Connect ground on the arduino to ground on the encoder
// Step 2: Connect the "3,3V" arduino pin to 5V on the encoder (it can handle the low voltage)
// Step 3: For the rear right encoder, connect A and B to pins 6 and 7 on the arduino (order will just flip sign)
// Step 4: For the rear left encoder, connect A and B to pins 9 and 8 on the arduino (order will just flip sign)
// The program will print to the serial console (115200 baud)

#define ENCODER_RESOLUTION 2000//counts per revolution (Setting 0 1 0 1 on the encoder)
#define WHEEL_RADIUS 0.0325 //meters
#define PI 3.14159265358979323846
#define TIMER_FREQUENCY 2625000.0 //per second
#define SAMPLE_SEPARATION 200

static long _speed_multiplier = (2*PI*WHEEL_RADIUS*TIMER_FREQUENCY)/ENCODER_RESOLUTION; //[meters/(counts*second)]
static long _rpm_multiplier = (TIMER_FREQUENCY/ENCODER_RESOLUTION) * 60; //[rev/counts*minute]

void setup() {
  // put your setup code here, to run once:
  create_timer(); // creates a 2.625MHz timer (internal, no pin output)
  create_left_enc_interrupt();
  create_right_enc_interrupt();
  create_front_left_enc_interrupt();
  create_front_right_enc_interrupt();

  Serial.begin(115200);
}

void loop() {
  //Serial.println(get_enc_count());
  Serial.print("&");
  Serial.print("RL");
  Serial.print(get_left_enc_rpm());
//  Serial.print(",");
  Serial.print("RR");
  Serial.print(get_right_enc_rpm());
//  Serial.print(",");
  Serial.print("FL");
  Serial.print(get_front_left_enc_rpm());
//  Serial.print(",");
  Serial.print("FR");
  Serial.println(get_front_right_enc_rpm());
}

//****************code for making and reading a 2.625 internal timer that can be read way faster than micros() and is much much more stable*******

//which timer you use is irrelevant, I used #0 but you may want one with no user-facing pins (if you want all the user-facing pins available to choose from)

unsigned long read_timer(){
  //this is only here as a demo, put register inline where needed - it's faster
  unsigned long us_time = TC0->TC_CHANNEL[0].TC_CV;
  return us_time;
}

void create_timer(){
  /* Counts up at 2.626 MHz (use this freq for calculating time in s or freq in hz)
   * This resets every 2^32 counts = ~ 27 minutes
   * The time delays between successive measurements (uint32 - uint32) will work fine 
   * unless they are more than 27 minutes apart 
   * Adapted code from second post at https://forum.arduino.cc/index.php?topic=556678.0 
   * There are other ways to access registers but...
   * this does a great job at minimizing future guesswork 
   * (e.g. debugging what went wrong and even what does what) */

  // Tell the power management controller to turn on the clock for TC0
  // Connections are found in datasheet on page 38 (#27 is TC0)
  PMC->PMC_PCER0 |= PMC_PCER0_PID27;                      // Timer Counter 0 channel 0 IS TC0

  // uncomment to connect to physical pin #2 as a means to verify 1MHz operation.
  // pin assignments for this are on page 41 of the datasheet and on page 858 (you may need to use both)
  // pin input output is split across PIOA, PIOB, and PIOC. 
  // Due uses 144 pin version
  // I recommend referring to this image for physical layout: https://www.pinterest.fr/pin/484770347367609215/
  // it is mostly comprehensive and makes it easy to find, for instance, TIOA and TIOB (hardware pin names are in white)
  // note - some of the pins, e.g. analog, they only report one of the multiplexed peripherals
  // for instance PIOC controls analog pins A1 to A9, which can be TIO or PWM pins as well (not shown in picture)
  // in that case you will have to read through the table on page 41 for PIOA, B, and C
  //PIOB->PIO_PDR |= PIO_PDR_P25 | PIO_PDR_P27;  // remove pin from PIO so it can go to peripheral
  //PIOB->PIO_ABSR |= PIO_ABSR_P25 | PIO_ABSR_P27; // attach to peripheral B function

  // we set it to wave mode rather than capture mode to generate a time signal
  // instead of measuring one
  // note that there are nine TC channels that can be turned on or off (e.g. see page 38)
  // and they are grouped in batches of three
  // in the code below, TC2->TC_CHANNEL[2] would access group 3, channel 3 since zero indexed (TC9)
  // this makes for somewhat confusing code 
  TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK3 // use MCLK / 32
                            | TC_CMR_WAVE                // generate output
                            | TC_CMR_WAVSEL_UP//_RC           // count up, restart on overflow (2^32-1)... add _RC to reset on RC match for sample output signal on pin 2
                            | TC_CMR_ACPA_CLEAR          // Clear TIOA0 on RA compare match
                            | TC_CMR_ACPC_SET;           // Set TIOA0 on RC compare match
                            
  //uncomment these to create a square wave output (this ruins the timer since it wraps around too often)
  //TC0->TC_CHANNEL[0].TC_RC = 10;
  //TC0->TC_CHANNEL[0].TC_RA = 5;

  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG | TC_CCR_CLKEN; // enable and start the clock
}

/** quadrature encoder **/
volatile unsigned long left_enc_circular_buffer[256];
volatile unsigned long left_enc_circular_buffer_count[256];
volatile long left_enc_counter = 0;
volatile byte left_enc_index = 0;

volatile unsigned long right_enc_circular_buffer[256];
volatile unsigned long right_enc_circular_buffer_count[256];
volatile long right_enc_counter = 0;
volatile byte right_enc_index = 0;

volatile unsigned long front_left_enc_circular_buffer[256];
volatile unsigned long front_left_enc_circular_buffer_count[256];
volatile long front_left_enc_counter = 0;
volatile byte front_left_enc_index = 0;

volatile unsigned long front_right_enc_circular_buffer[256];
volatile unsigned long front_right_enc_circular_buffer_count[256];
volatile long front_right_enc_counter = 0;
volatile byte front_right_enc_index = 0;

void create_left_enc_interrupt(){
  for (int i = 0;i<256;i++){
    left_enc_circular_buffer[i] = 0;
    left_enc_circular_buffer_count[i] = 0;
  }
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  attachInterrupt(digitalPinToInterrupt(6),left_enc_a_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(7),left_enc_b_ISR,CHANGE);
}

void create_front_left_enc_interrupt(){
  for (int i = 0;i<256;i++){
    front_left_enc_circular_buffer[i] = 0;
    front_left_enc_circular_buffer_count[i] = 0;
  }
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(3),front_left_enc_a_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(2),front_left_enc_b_ISR,CHANGE);
}

void create_right_enc_interrupt(){
  for (int i = 0;i<256;i++){
    right_enc_circular_buffer[i] = 0;
    right_enc_circular_buffer_count[i] = 0;
  }
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  attachInterrupt(digitalPinToInterrupt(8),right_enc_a_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(9),right_enc_b_ISR,CHANGE);
}


void create_front_right_enc_interrupt(){
  for (int i = 0;i<256;i++){
    right_enc_circular_buffer[i] = 0;
    right_enc_circular_buffer_count[i] = 0;
  }
  pinMode(5, INPUT);
  pinMode(4, INPUT);
  attachInterrupt(digitalPinToInterrupt(5),front_right_enc_a_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(4),front_right_enc_b_ISR,CHANGE);
}

long get_left_enc_count(){
  return left_enc_counter;
}

long get_front_left_enc_count(){
  return front_left_enc_counter;
}

long get_right_enc_count(){
  return right_enc_counter;
}

long get_front_right_enc_count(){
  return front_right_enc_counter;
}

float get_left_enc_rpm(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = left_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = left_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - left_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = left_enc_counter - left_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _rpm_multiplier * c_diff / t_diff;
}

float get_front_left_enc_rpm(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = front_left_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = front_left_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - front_left_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = front_left_enc_counter - front_left_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _rpm_multiplier * c_diff / t_diff;
}

float get_left_enc_speed(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = left_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = left_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - left_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = left_enc_counter - left_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _speed_multiplier * c_diff / t_diff;
}

float get_front_left_enc_speed(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = front_left_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = front_left_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - front_left_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = front_left_enc_counter - front_left_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _speed_multiplier * c_diff / t_diff;
}

float get_right_enc_rpm(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = right_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = right_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - right_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = right_enc_counter - right_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _rpm_multiplier * c_diff / t_diff;
}

float get_front_right_enc_rpm(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = front_right_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = front_right_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - front_right_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = front_right_enc_counter - front_right_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _rpm_multiplier * c_diff / t_diff;
}

float get_right_enc_speed(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = right_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = right_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - right_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = right_enc_counter - right_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _speed_multiplier * c_diff / t_diff;
}

float get_front_right_enc_speed(){
  /* speed is computed on demand to avoid float arithmetic in interrupts */
  // returns speed of the encoder in pulse Hz
  
  const byte sample_sep = SAMPLE_SEPARATION; // take two points 50 measurements apart... this is ok for a fast signal, maybe not for a slow one
  byte upper_index = front_right_enc_index-1;  // avoid the current buffer position, 
  byte lower_index = front_right_enc_index-1 - sample_sep; // this can wrap around safely
  long t_diff = TC0->TC_CHANNEL[0].TC_CV - front_right_enc_circular_buffer[lower_index];
  if (t_diff > 1300000)
    return 0;
  long c_diff = front_right_enc_counter - front_right_enc_circular_buffer_count[lower_index];
//  return 2625000.0 * c_diff / t_diff; //Counts/sec
  return _speed_multiplier * c_diff / t_diff;
}

void left_enc_a_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC24) == PIO_PC24){ // is pin 6 high?
    if ((PIOC->PIO_PDSR & PIO_PC23) == PIO_PC23){ // is pin 7 high?
      left_enc_counter++;
    }
    else{
      left_enc_counter--;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC23) == PIO_PC23){ // is pin 7 high?
      left_enc_counter--;
    }
    else{
      left_enc_counter++;
    }
    
  }

  left_enc_circular_buffer[left_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  left_enc_circular_buffer_count[left_enc_index] = left_enc_counter;
  left_enc_index++;
  
}

void front_left_enc_a_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC28) == PIO_PC28){ // is pin 3 high?
    if ((PIOB->PIO_PDSR & PIO_PB25) == PIO_PB25){ // is pin 2 high?
      front_left_enc_counter++;
    }
    else{
      front_left_enc_counter--;
    }
  }
  else{
    if ((PIOB->PIO_PDSR & PIO_PB25) == PIO_PB25){ // is pin 2 high?
      front_left_enc_counter--;
    }
    else{
      front_left_enc_counter++;
    }
    
  }

  front_left_enc_circular_buffer[front_left_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  front_left_enc_circular_buffer_count[front_left_enc_index] = front_left_enc_counter;
  front_left_enc_index++;
}

void left_enc_b_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC24) == PIO_PC24){ // is pin 6 high?
    if ((PIOC->PIO_PDSR & PIO_PC23) == PIO_PC23){ // is pin 7 high?
      left_enc_counter--;
    }
    else{
      left_enc_counter++;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC23) == PIO_PC23){ // is pin 7 high?
      left_enc_counter++;
    }
    else{
      left_enc_counter--;
    }
  }
    
  left_enc_circular_buffer[left_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  left_enc_circular_buffer_count[left_enc_index] = left_enc_counter;
  left_enc_index++;
}


void front_left_enc_b_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC28) == PIO_PC28){ // is pin 3 high?
    if ((PIOB->PIO_PDSR & PIO_PB25) == PIO_PB25){ // is pin 2 high?
      front_left_enc_counter--;
    }
    else{
      front_left_enc_counter++;
    }
  }
  else{
    if ((PIOB->PIO_PDSR & PIO_PB25) == PIO_PB25){ // is pin 2 high?
      front_left_enc_counter++;
    }
    else{
      front_left_enc_counter--;
    }
  }
    
  front_left_enc_circular_buffer[front_left_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  front_left_enc_circular_buffer_count[front_left_enc_index] = front_left_enc_counter;
  front_left_enc_index++;
}

void right_enc_a_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC22) == PIO_PC22){ // is pin 8 high?
    if ((PIOC->PIO_PDSR & PIO_PC21) == PIO_PC21){ // is pin 9 high?
      right_enc_counter++;
    }
    else{
      right_enc_counter--;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC21) == PIO_PC21){ // is pin 9 high?
      right_enc_counter--;
    }
    else{
      right_enc_counter++;
    }
    
  }

  right_enc_circular_buffer[right_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  right_enc_circular_buffer_count[right_enc_index] = right_enc_counter;
  right_enc_index++;
  
}

void front_right_enc_a_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC25) == PIO_PC25){ // is pin 5 high?
    if ((PIOC->PIO_PDSR & PIO_PC26) == PIO_PC26){ // is pin 4 high?
      front_right_enc_counter++;
    }
    else{
      front_right_enc_counter--;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC26) == PIO_PC26){ // is pin 4 high?
      front_right_enc_counter--;
    }
    else{
      front_right_enc_counter++;
    }
    
  }

  front_right_enc_circular_buffer[front_right_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  front_right_enc_circular_buffer_count[front_right_enc_index] = front_right_enc_counter;
  front_right_enc_index++;
  
}

void right_enc_b_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC22) == PIO_PC22){ // is pin 8 high?
    if ((PIOC->PIO_PDSR & PIO_PC21) == PIO_PC21){ // is pin 9 high?
      right_enc_counter--;
    }
    else{
      right_enc_counter++;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC21) == PIO_PC21){ // is pin 9 high?
      right_enc_counter++;
    }
    else{
      right_enc_counter--;
    }
  }
    
  right_enc_circular_buffer[right_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  right_enc_circular_buffer_count[right_enc_index] = right_enc_counter;
  right_enc_index++;
}

void front_right_enc_b_ISR(){
  if ((PIOC->PIO_PDSR & PIO_PC25) == PIO_PC25){ // is pin 5 high?
    if ((PIOC->PIO_PDSR & PIO_PC26) == PIO_PC26){ // is pin 4 high?
      front_right_enc_counter--;
    }
    else{
      front_right_enc_counter++;
    }
  }
  else{
    if ((PIOC->PIO_PDSR & PIO_PC26) == PIO_PC26){ // is pin 4 high?
      front_right_enc_counter++;
    }
    else{
      front_right_enc_counter--;
    }
  }
    
  front_right_enc_circular_buffer[front_right_enc_index] = TC0->TC_CHANNEL[0].TC_CV;
  front_right_enc_circular_buffer_count[front_right_enc_index] = front_right_enc_counter;
  front_right_enc_index++;
}
