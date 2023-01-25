#include <Arduino.h>
#include "Utils/Utils.h"
#include "Drivers/Imu.h"
#include "Drivers/I2CMux.hpp"

#define NUMIMUS 12
#define CALIBRATION_SAMPLES 10000

void offline_calibration(uint8_t i);

extern "C" uint32_t set_arm_clock(uint32_t frequency);

// Mpu9250 object
Imu imus[NUMIMUS] = {
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC),
    Imu(&Wire, Imu::I2C_ADDR_PRIM),
    Imu(&Wire, Imu::I2C_ADDR_SEC)
};
uint8_t mux_map[NUMIMUS] = { 1,1,2,2,3,3,4,4,5,5,0,0 };
uint8_t joint_map[NUMIMUS] = { 0,1,2,3,4,5,7,8,10,11,13,14 };

I2CMux tca9548a(0x70);

void setup()
{
    // Set clock speed (https://forum.pjrc.com/threads/58688-Teensy-4-0-Clock-speed-influences-delay-and-SPI)
    set_arm_clock(600000000);
    // ############# I2C #############
    // Start the I2C bus
    Wire.begin();
    Wire.setClock(400000);
    // Serial to display data
    Serial.begin(115200);
    while(!Serial) {};
    // Initialize and configure IMU
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        tca9548a.setChannel(mux_map[i]);
        while(!imus[i].init())
        {
            Serial.print("Error initializing communication with IMU");
            Serial.print(i);
            Serial.println(". Retrying...");
            delay(3000);
        }
    }

    // Offline calibration
    for(uint8_t i=0; i < NUMIMUS; i++)
    {
        offline_calibration(i);
    }
    Serial.println('{');
}

void loop()
{  
}

void offline_calibration(uint8_t i)
{
    tca9548a.setChannel(mux_map[i]);
    for(uint32_t j=0; j < CALIBRATION_SAMPLES; j++)
    {
        while(!imus[i].read());

        static constexpr int df = 15;
        Serial.print(i);
        Serial.print(",");
        Serial.print(j);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_x(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_y(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_gyro_z(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_accel_x(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_accel_y(), df);
        Serial.print(",");
        Serial.print(imus[i].raw_accel_z(), df);
        Serial.print(",");
        Serial.println(imus[i].temperature());
    }
}
