#include "Imu.h"
#include <algorithm>
#include "Utils/Utils.h"

static constexpr float GRAVITY = 9.807f;

Imu::Imu(SPIClass *spi, const uint8_t cs, float gx_offset, float gy_offset, float gz_offset)
    : imu(spi, cs), protocol(Imu::Protocol::SPI), gyro_offset{gx_offset, gy_offset, gz_offset}
{
}

Imu::Imu(TwoWire *i2c, const uint8_t addr, float gx_offset, float gy_offset, float gz_offset)
    : imu(i2c, (bfs::Mpu9250::I2cAddr)addr), protocol(Imu::Protocol::I2C), gyro_offset{gx_offset, gy_offset, gz_offset}
{
}

bool Imu::init()
{
    if(!imu.Begin())
        return false;
    /* Set the sample rate divider */
    if(!imu.ConfigSrd(19))
        return false;

    timer.start();
    return true;
}

void Imu::calibrate()
{
    constexpr uint16_t cycles = calib_cycles;
    float gx[cycles], gy[cycles], gz[cycles];
    float ax[cycles], ay[cycles], az[cycles];

    // Initialize acceleration buffer
    while(!imu.Read());
    for(uint8_t i=0; i < accel_buffer_len; i++)
    {
        accel_buffer[0][i] = imu.accel_x_mps2();
        accel_buffer[1][i] = imu.accel_y_mps2();
        accel_buffer[2][i] = imu.accel_z_mps2();
    }

    // Sample readings and find offsets
    for(uint16_t i=0; i < cycles;)
    {
        if(!imu.Read())
            continue;
        ax[i] = imu.accel_x_mps2();
        ay[i] = imu.accel_y_mps2();
        az[i] = imu.accel_z_mps2();
        gx[i] = imu.gyro_x_radps();
        gy[i] = imu.gyro_y_radps();
        gz[i] = imu.gyro_z_radps();
        push_accel_buffer(ax[i], ay[i], az[i]);
        i++;
    }

    // double accel_mean[3] = {
    //     mean(ax, cycles),
    //     mean(ay, cycles),
    //     mean(az, cycles)
    // };
    // const double accel_ratio = norm(accel_mean, 3) / GRAVITY;
    // accel_offset[0] = (accel_mean[0] * accel_ratio) - accel_mean[0];
    // accel_offset[1] = (accel_mean[1] * accel_ratio) - accel_mean[1];
    // accel_offset[2] = (accel_mean[2] * accel_ratio) - accel_mean[2];

    // accel_offset[0] = mean(ax, cycles);
    // accel_offset[1] = mean(ay, cycles);
    // accel_offset[2] = mean(az, cycles) + GRAVITY;
    
    gyro_offset[0] = mean(gx, cycles);
    gyro_offset[1] = mean(gy, cycles);
    gyro_offset[2] = mean(gz, cycles);
}

bool Imu::read() // https://www.youtube.com/watch?v=CHSYgLfhwUo
{
    uint32_t delta_us = timer.stop();
    recv_new = imu.Read();
    if(!recv_new)
        return recv_new;

    push_accel_buffer(imu.accel_x_mps2(), imu.accel_y_mps2(), imu.accel_z_mps2());
    float ax_filt = median(accel_buffer[0], Imu::accel_buffer_len);
    float ay_filt = median(accel_buffer[1], Imu::accel_buffer_len);
    float az_filt = median(accel_buffer[2], Imu::accel_buffer_len);
    accel_mps2[0] = (ax_filt - accel_offset[0]);
    accel_mps2[1] = (ay_filt - accel_offset[1]);
    accel_mps2[2] = (az_filt - accel_offset[2]);
    gyro_drad[0]  = (imu.gyro_x_radps() - gyro_offset[0]) * delta_us / 1000000.0;
    gyro_drad[1]  = (imu.gyro_y_radps() - gyro_offset[1]) * delta_us / 1000000.0;
    gyro_drad[2]  = (imu.gyro_z_radps() - gyro_offset[2]) * delta_us / 1000000.0;

    timer.start();
    return recv_new;
}

void Imu::push_accel_buffer(float ax, float ay, float az)
{
    accel_buffer[0][accel_buffer_index] = ax;
    accel_buffer[1][accel_buffer_index] = ay;
    accel_buffer[2][accel_buffer_index] = az;
    accel_buffer_index = (accel_buffer_index + 1) % Imu::accel_buffer_len;
}
