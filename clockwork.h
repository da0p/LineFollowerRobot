#ifndef __CLOCKWORK_H__ // this is done to avoid multiple inclusions
#define __CLOCKWORK_H__

#include "Arduino.h"

typedef unsigned long int micros_t;

typedef void (*CallbackType)(unsigned long);

class Clockwork {
  public:
    Clockwork(unsigned int period_ms);
    // overloading
    Clockwork(unsigned int period_ms, CallbackType cb);
    void start();
    bool stop();

  private:
    micros_t _period_ms;
    micros_t _last_start;
    micros_t _last_stop;
    micros_t _tet; // time execution task
    CallbackType _cb;

};

#endif
