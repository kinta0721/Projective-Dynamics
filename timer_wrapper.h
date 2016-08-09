#ifndef _TIMER_WRAPPER_H_
#define _TIMER_WRAPPER_H_

// TODO: change to a more accurate timer lib instead of standard C lib
#include <time.h>

#include "global_headers.h"

class TimerWrapper
{
public:
    inline TimerWrapper() {m_sec_per_clock = 1.0/(ScalarType)CLOCKS_PER_SEC;}
    inline virtual ~TimerWrapper() {}

    inline void Tic() {m_start = clock();m_pause_time=0;}
    inline void Toc() {m_end = clock();}
    inline void Pause() {m_pause_start=clock();}
    inline void Resume() {m_pause_time += (clock()-m_pause_start);}
    inline ScalarType Duration() {return (m_end-m_start-m_pause_time)*m_sec_per_clock;}

protected:
    clock_t m_start;
    clock_t m_end;
    clock_t m_pause_start;
    clock_t m_pause_time;
    ScalarType m_sec_per_clock;
};

#endif