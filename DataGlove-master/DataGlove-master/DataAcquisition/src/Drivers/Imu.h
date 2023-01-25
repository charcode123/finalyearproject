#pragma once
#include <stdint.h>
#include "Timer.hpp"
#include "mpu9250.h"

// Wrapper class for IMU
class Imu
{
public:
    enum class Protocol : uint8_t
    {
        SPI,
        I2C
    };

    Imu(SPIClass *spi, const uint8_t cs, float gx_offset=0, float gy_offset=0, float gz_offset=0);
    Imu(TwoWire *i2c, const uint8_t addr, float gx_offset=0, float gy_offset=0, float gz_offset=0);

    bool init();
    void calibrate();
    bool read();

    double accel_x()    { return accel_mps2[0];      }
    double accel_y()    { return accel_mps2[1];      }
    double accel_z()    { return accel_mps2[2];      }
    double gyro_x()     { return gyro_drad[0];       }
    double gyro_y()     { return gyro_drad[1];       }
    double gyro_z()     { return gyro_drad[2];       }
    float temperature() { return imu.die_temp_c();   }
    float raw_accel_x() { return imu.accel_x_mps2(); }
    float raw_accel_y() { return imu.accel_y_mps2(); }
    float raw_accel_z() { return imu.accel_z_mps2(); }
    float raw_gyro_x()  { return imu.gyro_x_radps(); }
    float raw_gyro_y()  { return imu.gyro_y_radps(); }
    float raw_gyro_z()  { return imu.gyro_z_radps(); }
    bool new_data()     { return recv_new;           }

private:
    void push_accel_buffer(float ax, float ay, float az);
    
public:
    static constexpr uint8_t I2C_ADDR_PRIM = 0x68;
    static constexpr uint8_t I2C_ADDR_SEC = 0x69;
    static constexpr uint8_t accel_buffer_len = 12;
    static constexpr uint16_t calib_cycles = 50;

private:
    bfs::Mpu9250 imu;
    Protocol protocol;
    double accel_mps2[3];
    double gyro_drad[3];
    float accel_offset[3] = {};
    float gyro_offset[3];
    float accel_buffer[3][accel_buffer_len];
    uint8_t accel_buffer_index = 0;
    bool recv_new = false;
    Timer timer;
};