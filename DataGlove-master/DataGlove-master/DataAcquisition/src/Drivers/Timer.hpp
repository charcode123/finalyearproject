#pragma once
#include <stdint.h>
#include <Arduino.h>

class Timer
{
public:
    Timer()
    {
        start();
    }

    inline void start()
    {
        started = micros();
    }

    inline uint32_t elapsed()
    {
        return stopped - started;
    }

    inline uint32_t elapsed_now()
    {
        return micros() - started;
    }

    inline uint32_t stop()
    {
        stopped = micros();
        return elapsed();
    }

    static inline float seconds(uint32_t micros)
    {
        return micros / 1000.0f;
    }
    
private:
    uint32_t started;
    uint32_t stopped = 0;
};