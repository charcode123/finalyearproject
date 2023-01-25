#pragma once

#include <stdint.h>
#include <type_traits>

template<typename T>
class AnalogSensor
{
    static_assert(std::is_arithmetic<T>::value, "Analog sensor output must be an arithmetic type.");

public:
    AnalogSensor(uint8_t analog_pin, uint16_t in_min, uint16_t in_max, T out_min, T out_max)
        : pin(analog_pin), in_min(in_min), in_max(in_max), out_min(out_min), out_max(out_max)
    {
    }

    inline T read()
    {
        int in = analogRead(pin);
        return (T)(out_min + (out_max - out_min) / (double)(in_max - in_min) * (in - in_min));
    }

private:
    uint8_t pin;
    uint16_t in_min, in_max;
    T out_min, out_max;
};
