/****************************************************************************************************************************
  NRF52_MBED_ISR_Timer-Impl.h
  For NRF52 boards using mbed-RTOS such as Nano-33-BLE
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/NRF52_MBED_TimerInterrupt
  Licensed under MIT license

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one NRF52 timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Based on SimpleTimer - A timer library for Arduino.
  Author: mromani@ottotecnica.com
  Copyright (c) 2010 OTTOTECNICA Italy

  Based on BlynkTimer.h
  Author: Volodymyr Shymanskyy

  Version: 1.4.1

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.1   K Hoang      22/11/2020 Initial coding and sync with NRF52_TimerInterrupt
  1.0.2   K Hoang      23/11/2020 Add and optimize examples
  1.1.1   K.Hoang      06/12/2020 Add Change_Interval example. Bump up version to sync with other TimerInterrupt Libraries
  1.2.0   K.Hoang      11/01/2021 Add better debug feature. Optimize code and examples to reduce RAM usage
  1.2.1   K.Hoang      04/05/2021 Add mbed_nano to list of compatible architectures
  1.3.0   K.Hoang      09/09/2021 Don't use NRF_TIMER_1 because of mbed_nano core v2.0.0+
  1.4.0   K.Hoang      22/01/2022 Fix `multiple-definitions` linker error
  1.4.0   K.Hoang      26/10/2022 Add support to SEEED_XIAO_NRF52840 and SEEED_XIAO_NRF52840_SENSE using mbed
*****************************************************************************************************************************/

#pragma once

#ifndef ISR_TIMER_GENERIC_IMPL_H
#define ISR_TIMER_GENERIC_IMPL_H

//#include "NRF52_MBED_ISR_Timer.h"
#include <string.h>

NRF52_MBED_ISR_Timer::NRF52_MBED_ISR_Timer()
  : numTimers (-1)
{
}

void NRF52_MBED_ISR_Timer::init()
{
  unsigned long current_millis = millis();   //elapsed();

  for (uint8_t i = 0; i < MAX_NUMBER_TIMERS; i++)
  {
    memset((void*) &timer[i], 0, sizeof (timer_t));
    timer[i].prev_millis = current_millis;
  }

  numTimers = 0;
}

void NRF52_MBED_ISR_Timer::run()
{
  uint8_t i;
  unsigned long current_millis;

  // get current time
  current_millis = millis();   //elapsed();

  for (i = 0; i < MAX_NUMBER_TIMERS; i++)
  {

    timer[i].toBeCalled = TIMER_DEFCALL_DONTRUN;

    // no callback == no timer, i.e. jump over empty slots
    if (timer[i].callback != NULL)
    {

      // is it time to process this timer ?
      // see http://arduino.cc/forum/index.php/topic,124048.msg932592.html#msg932592

      if ((current_millis - timer[i].prev_millis) >= timer[i].delay)
      {
        unsigned long skipTimes = (current_millis - timer[i].prev_millis) / timer[i].delay;

        // update time
        timer[i].prev_millis += timer[i].delay * skipTimes;

        // check if the timer callback has to be executed
        if (timer[i].enabled)
        {

          // "run forever" timers must always be executed
          if (timer[i].maxNumRuns == TIMER_RUN_FOREVER)
          {
            timer[i].toBeCalled = TIMER_DEFCALL_RUNONLY;
          }
          // other timers get executed the specified number of times
          else if (timer[i].numRuns < timer[i].maxNumRuns)
          {
            timer[i].toBeCalled = TIMER_DEFCALL_RUNONLY;
            timer[i].numRuns++;

            // after the last run, delete the timer
            if (timer[i].numRuns >= timer[i].maxNumRuns)
            {
              timer[i].toBeCalled = TIMER_DEFCALL_RUNANDDEL;
            }
          }
        }
      }
    }
  }

  for (i = 0; i < MAX_NUMBER_TIMERS; i++)
  {
    if (timer[i].toBeCalled == TIMER_DEFCALL_DONTRUN)
      continue;

    if (timer[i].hasParam)
      (*(timerCallback_p)timer[i].callback)(timer[i].param);
    else
      (*(timerCallback)timer[i].callback)();

    if (timer[i].toBeCalled == TIMER_DEFCALL_RUNANDDEL)
      deleteTimer(i);
  }
}


// find the first available slot
// return -1 if none found
int NRF52_MBED_ISR_Timer::findFirstFreeSlot()
{
  // all slots are used
  if (numTimers >= MAX_NUMBER_TIMERS)
  {
    return -1;
  }

  // return the first slot with no callback (i.e. free)
  for (uint8_t i = 0; i < MAX_NUMBER_TIMERS; i++)
  {
    if (timer[i].callback == NULL)
    {
      return i;
    }
  }

  // no free slots found
  return -1;
}


int NRF52_MBED_ISR_Timer::setupTimer(const unsigned long& d, void* f, void* p, bool h, const unsigned& n)
{
  int freeTimer;

  if (numTimers < 0)
  {
    init();
  }

  freeTimer = findFirstFreeSlot();

  if (freeTimer < 0)
  {
    return -1;
  }

  if (f == NULL)
  {
    return -1;
  }

  timer[freeTimer].delay = d;
  timer[freeTimer].callback = f;
  timer[freeTimer].param = p;
  timer[freeTimer].hasParam = h;
  timer[freeTimer].maxNumRuns = n;
  timer[freeTimer].enabled = true;
  timer[freeTimer].prev_millis = millis();

  numTimers++;

  return freeTimer;
}


int NRF52_MBED_ISR_Timer::setTimer(const unsigned long& d, timerCallback f, const unsigned& n)
{
  return setupTimer(d, (void *)f, NULL, false, n);
}

int NRF52_MBED_ISR_Timer::setTimer(const unsigned long& d, timerCallback_p f, void* p, const unsigned& n)
{
  return setupTimer(d, (void *)f, p, true, n);
}

int NRF52_MBED_ISR_Timer::setInterval(const unsigned long& d, timerCallback f)
{
  return setupTimer(d, (void *)f, NULL, false, TIMER_RUN_FOREVER);
}

int NRF52_MBED_ISR_Timer::setInterval(const unsigned long& d, timerCallback_p f, void* p)
{
  return setupTimer(d, (void *)f, p, true, TIMER_RUN_FOREVER);
}

int NRF52_MBED_ISR_Timer::setTimeout(const unsigned long& d, timerCallback f)
{
  return setupTimer(d, (void *)f, NULL, false, TIMER_RUN_ONCE);
}

int NRF52_MBED_ISR_Timer::setTimeout(const unsigned long& d, timerCallback_p f, void* p)
{
  return setupTimer(d, (void *)f, p, true, TIMER_RUN_ONCE);
}

bool NRF52_MBED_ISR_Timer::changeInterval(const unsigned& numTimer, const unsigned long& d)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return false;
  }

  // Updates interval of existing specified timer
  if (timer[numTimer].callback != NULL)
  {
    timer[numTimer].delay = d;
    timer[numTimer].prev_millis = millis();

    return true;
  }

  // false return for non-used numTimer, no callback
  return false;
}

void NRF52_MBED_ISR_Timer::deleteTimer(const unsigned& timerId)
{
  if (timerId >= MAX_NUMBER_TIMERS)
  {
    return;
  }

  // nothing to delete if no timers are in use
  if (numTimers == 0)
  {
    return;
  }

  // don't decrease the number of timers if the specified slot is already empty
  if (timer[timerId].callback != NULL)
  {
    memset((void*) &timer[timerId], 0, sizeof (timer_t));
    timer[timerId].prev_millis = millis();

    // update number of timers
    numTimers--;
  }
}

// function contributed by code@rowansimms.com
void NRF52_MBED_ISR_Timer::restartTimer(const unsigned& numTimer)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return;
  }

  timer[numTimer].prev_millis = millis();
}


bool NRF52_MBED_ISR_Timer::isEnabled(const unsigned& numTimer)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return false;
  }

  return timer[numTimer].enabled;
}


void NRF52_MBED_ISR_Timer::enable(const unsigned& numTimer)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return;
  }

  timer[numTimer].enabled = true;
}


void NRF52_MBED_ISR_Timer::disable(const unsigned& numTimer)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return;
  }

  timer[numTimer].enabled = false;
}

void NRF52_MBED_ISR_Timer::enableAll()
{
  // Enable all timers with a callback assigned (used)

  for (uint8_t i = 0; i < MAX_NUMBER_TIMERS; i++)
  {
    if (timer[i].callback != NULL && timer[i].numRuns == TIMER_RUN_FOREVER)
    {
      timer[i].enabled = true;
    }
  }
}

void NRF52_MBED_ISR_Timer::disableAll()
{
  // Disable all timers with a callback assigned (used)

  for (uint8_t i = 0; i < MAX_NUMBER_TIMERS; i++)
  {
    if (timer[i].callback != NULL && timer[i].numRuns == TIMER_RUN_FOREVER)
    {
      timer[i].enabled = false;
    }
  }
}

void NRF52_MBED_ISR_Timer::toggle(const unsigned& numTimer)
{
  if (numTimer >= MAX_NUMBER_TIMERS)
  {
    return;
  }

  timer[numTimer].enabled = !timer[numTimer].enabled;
}


unsigned NRF52_MBED_ISR_Timer::getNumTimers()
{
  return numTimers;
}

#endif    // ISR_TIMER_GENERIC_IMPL_H
