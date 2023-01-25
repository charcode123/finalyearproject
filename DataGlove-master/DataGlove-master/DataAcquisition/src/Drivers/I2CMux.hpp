#pragma once

#include <Wire.h>
#include <stdint.h>

class I2CMux
{
public:
    I2CMux(uint8_t i2c_addr);
    void setChannel(uint8_t channel);
    uint8_t getCurrent();

private:
    uint8_t i2c_addr;
    uint8_t current_channel = 255;      // invalid before initialization
};

I2CMux::I2CMux(uint8_t i2c_addr)
    : i2c_addr(i2c_addr)
{
}

void I2CMux::setChannel(uint8_t channel)
{
    if(current_channel == channel)
        return;
    
    Wire.beginTransmission(i2c_addr);
    Wire.write(1 << channel);
    Wire.endTransmission();

    current_channel = channel;
}

uint8_t I2CMux::getCurrent()
{
    return current_channel;
}
