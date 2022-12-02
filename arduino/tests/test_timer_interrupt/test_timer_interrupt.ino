// These define's must be placed at the beginning before #include "NRF52TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// For Nano33-BLE, don't use Serial.print() in ISR as system will definitely hang.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     3

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "NRF52_MBED_TimerInterrupt.h"

#define TIMER0_INTERVAL_MS        50
#define TIMER1_INTERVAL_MS        500

static bool toggle0 = false;
static bool toggle1 = false;
#define LED_BLUE_PIN LEDB

// Init NRF52 timers
NRF52_MBED_Timer ITimer0(NRF_TIMER_3);
NRF52_MBED_Timer ITimer1(NRF_TIMER_4);

void TimerHandler0() {
  digitalWrite(LED_BUILTIN, toggle0);
  toggle0 = !toggle0;
}

void TimerHandler1() {
  digitalWrite(LED_BLUE_PIN, toggle1);
  toggle1 = !toggle1;
}

void setup()
{
  pinMode(LED_BUILTIN,  OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  Serial.begin(115200);
  if (!ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0)) {
    Serial.println("Can't set ITimer0. Select another freq. or timer");
  }
  if (!ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1)) {
    Serial.println("Can't set ITimer0. Select another freq. or timer");
  }
}

void loop()
{
}
