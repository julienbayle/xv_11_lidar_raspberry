#include <time.h>
#include <string.h>
#include <malloc.h>
#include <iostream>
#include <fstream>

#include "xv_11_lidar_raspberry/motor.h"

namespace xv_11_lidar_raspberry {

  void digitalWrite (bool value)
  {
    std::ofstream file;
    file.open ("/sys/class/gpio/gpio18/value", std::ios::out | std::ios::trunc);
    file << value ? "1" : "0";
    file.close();
  }

  int setThreadPriority (const int pri)
  {
    struct sched_param sched ;
    memset (&sched, 0, sizeof(sched)) ;

    if (pri > sched_get_priority_max (SCHED_RR))
      sched.sched_priority = sched_get_priority_max (SCHED_RR) ;
    else
      sched.sched_priority = pri ;

    return sched_setscheduler (0, SCHED_RR, &sched) ;
  }

  void delayMicroseconds (unsigned int howLong)
  {
    struct timespec sleeper ;
    sleeper.tv_sec  = howLong / 1000000 ;
    sleeper.tv_nsec = (long)(howLong % 1000000 * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }

  static void *softPwmThread (void *)
  {
    int space ;

    struct sched_param param ;
    param.sched_priority = sched_get_priority_max (SCHED_RR) ;
    pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;
    setThreadPriority (90) ;

    for (;;)
    {
      if (pwm_ != 0)
        digitalWrite (true) ;
      
      delayMicroseconds (pwm_ * 100) ;

      if (pwmRange_ - pwm_ != 0)
        digitalWrite (false) ;
      
      delayMicroseconds ((pwmRange_ - pwm_) * 100) ;
    }
  }

  void startPwm (int value)
  {
    // Not already running
    if (pwmRange_ <= 0) 
    {
      digitalWrite(false);
      pwmRange_ = 1000;
      setPwm(value);
      pthread_create (&thread_, NULL, softPwmThread, NULL);
    }
  }

  void setPwm (int value)
  {
    if (value < 0)
      pwm_ = 0;
    else if (value > pwmRange_)
      pwm_ = 1000 ;
    else
      pwm_ = value ;
  }

  void stopPwm ()
  {
      if (pwmRange_ != 0)
      {
        pthread_cancel (thread_) ;
        pthread_join   (thread_, NULL) ;
        pwmRange_ = 0 ;
        digitalWrite (false) ;
      }
  }

}