#pragma once
#include <stdint.h>
#include "Timer.hpp"

template <bool activationState>
class Button
{
public:
    Button(uint8_t pin, uint32_t debounceTime=50000)
        : pin(pin), debounceTime(debounceTime)
    {
        pinMode(pin, INPUT);
    }

    bool clicked()
    {
        const bool reading = (bool)digitalRead(pin);
        if(reading != lastReading)
            time.start();

        if(time.elapsed_now() > debounceTime)
        {
            if(reading != currentState)
            {
                currentState = reading;
                if(currentState == State::ON)
                {
                    lastReading = reading;
                    return true;
                }
            }
        }
        lastReading = reading;
        return false;
    }

    bool pressed()
    {
        return (bool)digitalRead(pin);
    }

private:
    enum State : bool
    {
        OFF = !activationState,
        ON = activationState
    };

private:
    const uint8_t pin;
    const uint32_t debounceTime;
    bool lastReading = !activationState;
    bool currentState = !activationState;
    Timer time;
};