/* 
 * The code in this file "MPU9250.h" is referenced and modified 
 * from Hideaki Tai (hideakitai)'s GitHub project:
 * https://github.com/hideakitai/MPU9250
 * License: MIT License
 */


#pragma once
#ifndef MPU9250_H
#define MPU9250_H

#include <iostream>
#include "MPU9250RegisterMap.h"
#include "QuaternionFilter.h"
#include "i2c_master.h"

enum class ACCEL_FS_SEL {
    A2G,
    A4G,
    A8G,
    A16G
};
enum class GYRO_FS_SEL {
    G250DPS,
    G500DPS,
    G1000DPS,
    G2000DPS
};
enum class MAG_OUTPUT_BITS {
    M14BITS,
    M16BITS
};

enum class FIFO_SAMPLE_RATE : uint8_t {
    SMPL_1000HZ,
    SMPL_500HZ,
    SMPL_333HZ,
    SMPL_250HZ,
    SMPL_200HZ,
    SMPL_167HZ,
    SMPL_143HZ,
    SMPL_125HZ,
};

enum class GYRO_DLPF_CFG : uint8_t {
    DLPF_250HZ,
    DLPF_184HZ,
    DLPF_92HZ,
    DLPF_41HZ,
    DLPF_20HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_3600HZ,
};

enum class ACCEL_DLPF_CFG : uint8_t {
    DLPF_218HZ_0,
    DLPF_218HZ_1,
    DLPF_99HZ,
    DLPF_45HZ,
    DLPF_21HZ,
    DLPF_10HZ,
    DLPF_5HZ,
    DLPF_420HZ,
};

static constexpr uint8_t MPU9250_WHOAMI_DEFAULT_VALUE {0x71};
static constexpr uint8_t MPU9255_WHOAMI_DEFAULT_VALUE {0x73};
static constexpr uint8_t MPU6500_WHOAMI_DEFAULT_VALUE {0x70};

struct MPU9250Setting {
    ACCEL_FS_SEL accel_fs_sel {ACCEL_FS_SEL::A16G};
    GYRO_FS_SEL gyro_fs_sel {GYRO_FS_SEL::G2000DPS};
    MAG_OUTPUT_BITS mag_output_bits {MAG_OUTPUT_BITS::M16BITS};
    FIFO_SAMPLE_RATE fifo_sample_rate {FIFO_SAMPLE_RATE::SMPL_125HZ};
    uint8_t gyro_fchoice {0x03};
    GYRO_DLPF_CFG gyro_dlpf_cfg {GYRO_DLPF_CFG::DLPF_5HZ};
    uint8_t accel_fchoice {0x01};
    ACCEL_DLPF_CFG accel_dlpf_cfg {ACCEL_DLPF_CFG::DLPF_45HZ};
};

class MPU9250 {
    static constexpr uint8_t MPU9250_DEFAULT_ADDRESS {0x68};  // Device address when ADO = 0
    static constexpr uint8_t AK8963_ADDRESS {0x0C};           //  Address of magnetometer
    static constexpr uint8_t AK8963_WHOAMI_DEFAULT_VALUE {0x48};
    uint8_t mpu_i2c_addr {MPU9250_DEFAULT_ADDRESS};

    // settings
    MPU9250Setting setting;
    // TODO: this should be configured!!
    static constexpr uint8_t MAG_MODE {0x06};  // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    float acc_resolution {0.f};                // scale resolutions per LSB for the sensors
    float gyro_resolution {0.f};               // scale resolutions per LSB for the sensors
    float mag_resolution {0.f};                // scale resolutions per LSB for the sensors

    // Calibration Parameters
    float acc_bias[3] {0., 0., 0.};   // acc calibration value in ACCEL_FS_SEL: 2g
    int   gry_count = 1;
    float gyro_bias[3] {1.64795, 0.305176, 0.75};  // gyro calibration value in GYRO_FS_SEL: 250dps
    float mag_bias_factory[3] {0., 0., 0.};
    float mag_bias[3] {-689.21f, -51.47f, 0.};  // mag calibration value in MAG_OUTPUT_BITS: 16BITS
    float mag_scale[3] {1., 1., 1.};
    float magnetic_declination = -5.6;  // Taiwan, 18th July 2024

    // Temperature
    int16_t temperature_count {0};  // temperature raw count output
    float temperature {0.f};        // Stores the real internal chip temperature in degrees Celsius

    // Self Test
    float self_test_result[6] {0.f};  // holds results of gyro and accelerometer self test

    // IMU Data
    float a[3] {0.f, 0.f, 0.f};
    float g[3] {0.f, 0.f, 0.f};
    float m[3] {0.f, 0.f, 0.f};
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // vector to hold quaternion
    float rpy[3] {0.f, 0.f, 0.f};
    float yaw {0.f};
    float lin_acc[3] {0.f, 0.f, 0.f};  // linear acceleration (acceleration with gravity component subtracted)
    QuaternionFilter quat_filter;
    size_t n_filter_iter {1};

    // Other settings
    bool has_connected {false};
    bool b_ahrs {true};

    // I2C
    I2C i2c_;

public:
    static constexpr uint16_t CALIB_GYRO_SENSITIVITY {131};     // LSB/degrees/sec
    static constexpr uint16_t CALIB_ACCEL_SENSITIVITY {16384};  // LSB/g

    bool setup(const uint8_t addr, I2C& i2c, const MPU9250Setting& mpu_setting=MPU9250Setting());

    void sleep(bool b) {
        uint8_t c = read_byte(mpu_i2c_addr, PWR_MGMT_1);  // read the value, change sleep bit to match b, write byte back to register
        if (b) {
            c = c | 0x40;  // sets the sleep bit
        } else {
            c = c & 0xBF;  // mask 1011111 keeps all the previous bits
        }
        write_byte(mpu_i2c_addr, PWR_MGMT_1, c);
    }

    void ahrs(const bool b) {
        b_ahrs = b;
    }

    void calibrateAccelGyro() {
        calibrate_acc_gyro_impl();
    }

    void calibrateMag() {
        calibrate_mag_impl();
    }

    bool isConnected() {
        has_connected = isConnectedMPU9250() && isConnectedAK8963();
        return has_connected;
    }

    bool isConnectedMPU9250() {
        uint8_t c = read_byte(mpu_i2c_addr, WHO_AM_I_MPU9250);
        bool b = (c == MPU9250_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU9255_WHOAMI_DEFAULT_VALUE);
        b |= (c == MPU6500_WHOAMI_DEFAULT_VALUE);
        return b;
    }

    bool isConnectedAK8963() {
        uint8_t c = read_byte(AK8963_ADDRESS, AK8963_WHO_AM_I);
        return (c == AK8963_WHOAMI_DEFAULT_VALUE);
    }

    bool isSleeping() {
        uint8_t c = read_byte(mpu_i2c_addr, PWR_MGMT_1);
        return (c & 0x40) == 0x40;
    }

    bool available() {
        return has_connected && (read_byte(mpu_i2c_addr, INT_STATUS) & 0x01);
    }

    bool update();
    bool update2();

    float getRoll() const { return rpy[0]; }
    float getPitch() const { return rpy[1]; }
    float getYaw() const { return rpy[2]; }
    float getyaw() const { return yaw; }

    float getEulerX() const { return rpy[0]; }
    float getEulerY() const { return -rpy[1]; }
    float getEulerZ() const { return -rpy[2]; }

    float getQuaternionX() const { return q[1]; }
    float getQuaternionY() const { return q[2]; }
    float getQuaternionZ() const { return q[3]; }
    float getQuaternionW() const { return q[0]; }

    float getAcc(const uint8_t i) const { return (i < 3) ? a[i] : 0.f; }
    float getGyro(const uint8_t i) const { return (i < 3) ? g[i] : 0.f; }
    float getMag(const uint8_t i) const { return (i < 3) ? m[i] : 0.f; }
    float getLinearAcc(const uint8_t i) const { return (i < 3) ? lin_acc[i] : 0.f; }

    float getAccX() const { return a[0]; }
    float getAccY() const { return a[1]; }
    float getAccZ() const { return a[2]; }
    float getGyroX() const { return g[0]; }
    float getGyroY() const { return g[1]; }
    float getGyroZ() const { return g[2]; }
    float getMagX() const { return m[0]; }
    float getMagY() const { return m[1]; }
    float getMagZ() const { return m[2]; }
    float getLinearAccX() const { return lin_acc[0]; }
    float getLinearAccY() const { return lin_acc[1]; }
    float getLinearAccZ() const { return lin_acc[2]; }

    float getAccBias(const uint8_t i) const { return (i < 3) ? acc_bias[i] : 0.f; }
    float getGyroBias(const uint8_t i) const { return (i < 3) ? gyro_bias[i] : 0.f; }
    float getMagBias(const uint8_t i) const { return (i < 3) ? mag_bias[i] : 0.f; }
    float getMagScale(const uint8_t i) const { return (i < 3) ? mag_scale[i] : 0.f; }

    float getAccBiasX() const { return acc_bias[0]; }
    float getAccBiasY() const { return acc_bias[1]; }
    float getAccBiasZ() const { return acc_bias[2]; }
    float getGyroBiasX() const { return gyro_bias[0]; }
    float getGyroBiasY() const { return gyro_bias[1]; }
    float getGyroBiasZ() const { return gyro_bias[2]; }
    float getMagBiasX() const { return mag_bias[0]; }
    float getMagBiasY() const { return mag_bias[1]; }
    float getMagBiasZ() const { return mag_bias[2]; }
    float getMagScaleX() const { return mag_scale[0]; }
    float getMagScaleY() const { return mag_scale[1]; }
    float getMagScaleZ() const { return mag_scale[2]; }

    float getTemperature() const { return temperature; }

    void setAccBias(const float x, const float y, const float z) {
        acc_bias[0] = x;
        acc_bias[1] = y;
        acc_bias[2] = z;
        write_accel_offset();
    }
    void setGyroBias(const float x, const float y, const float z) {
        gyro_bias[0] = x;
        gyro_bias[1] = y;
        gyro_bias[2] = z;
        write_gyro_offset();
    }
    void setMagBias(const float x, const float y, const float z) {
        mag_bias[0] = x;
        mag_bias[1] = y;
        mag_bias[2] = z;
    }
    void setMagScale(const float x, const float y, const float z) {
        mag_scale[0] = x;
        mag_scale[1] = y;
        mag_scale[2] = z;
    }
    void setMagneticDeclination(const float d) { magnetic_declination = d; }

    void setFilterIterations(const size_t n) {
        if (n > 0) n_filter_iter = n;
    }

    bool selftest() {
        return self_test_impl();
    }

private:
    void initMPU9250();

    void initAK8963();

public:
    void update_rpy(float qw, float qx, float qy, float qz);

    void update_accel_gyro();

private:
    void read_accel_gyro(int16_t* destination) {
        char raw_data[14];                                                 // x/y/z accel register data stored here
        read_bytes(mpu_i2c_addr, ACCEL_XOUT_H, 14, &raw_data[0]);             // Read the 14 raw data registers into data array
        destination[0] = ((int16_t)raw_data[0] << 8) | (int16_t)raw_data[1];  // Turn the MSB and LSB into a signed 16-bit value
        destination[1] = ((int16_t)raw_data[2] << 8) | (int16_t)raw_data[3];
        destination[2] = ((int16_t)raw_data[4] << 8) | (int16_t)raw_data[5];
        destination[3] = ((int16_t)raw_data[6] << 8) | (int16_t)raw_data[7];
        destination[4] = ((int16_t)raw_data[8] << 8) | (int16_t)raw_data[9];
        destination[5] = ((int16_t)raw_data[10] << 8) | (int16_t)raw_data[11];
        destination[6] = ((int16_t)raw_data[12] << 8) | (int16_t)raw_data[13];
    }

public:
    void update_mag();

private:
    bool read_mag(int16_t* destination);


    int16_t read_temperature_data() {
        char raw_data[2];                                    // x/y/z gyro register data stored here
        read_bytes(mpu_i2c_addr, TEMP_OUT_H, 2, &raw_data[0]);  // Read the two raw data registers sequentially into data array
        return ((int16_t)raw_data[0] << 8) | raw_data[1];       // Turn the MSB and LSB into a 16-bit value
    }

    // Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
    // of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    // ACCEL_FS_SEL: 2g (maximum sensitivity)
    // GYRO_FS_SEL: 250dps (maximum sensitivity)
    void calibrate_acc_gyro_impl() {
        set_acc_gyro_to_calibration();
        collect_acc_gyro_data_to(acc_bias, gyro_bias);
        write_accel_offset();
        write_gyro_offset();
        delay(100);
        initMPU9250();
        delay(1000);
    }

    void set_acc_gyro_to_calibration();

    void collect_acc_gyro_data_to(float* a_bias, float* g_bias);

    void write_accel_offset();

    void write_gyro_offset();

    // mag calibration is executed in MAG_OUTPUT_BITS: 16BITS
    void calibrate_mag_impl();

    void collect_mag_data_to(float* m_bias, float* m_scale);

    // Accelerometer and gyroscope self test; check calibration wrt factory settings
    bool self_test_impl();  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    

    float get_acc_resolution(const ACCEL_FS_SEL accel_af_sel) const {
        switch (accel_af_sel) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case ACCEL_FS_SEL::A2G:
                return 2.0 / 32768.0;
            case ACCEL_FS_SEL::A4G:
                return 4.0 / 32768.0;
            case ACCEL_FS_SEL::A8G:
                return 8.0 / 32768.0;
            case ACCEL_FS_SEL::A16G:
                return 16.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_gyro_resolution(const GYRO_FS_SEL gyro_fs_sel) const {
        switch (gyro_fs_sel) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
            case GYRO_FS_SEL::G250DPS:
                return 250.0 / 32768.0;
            case GYRO_FS_SEL::G500DPS:
                return 500.0 / 32768.0;
            case GYRO_FS_SEL::G1000DPS:
                return 1000.0 / 32768.0;
            case GYRO_FS_SEL::G2000DPS:
                return 2000.0 / 32768.0;
            default:
                return 0.;
        }
    }

    float get_mag_resolution(const MAG_OUTPUT_BITS mag_output_bits) const {
        switch (mag_output_bits) {
            // Possible magnetometer scales (and their register bit settings) are:
            // 14 bit resolution (0) and 16 bit resolution (1)
            // Proper scale to return milliGauss
            case MAG_OUTPUT_BITS::M14BITS:
                return 10. * 4912. / 8190.0;
            case MAG_OUTPUT_BITS::M16BITS:
                return 10. * 4912. / 32760.0;
            default:
                return 0.;
        }
    }

    void write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
        int i2c_err_ = i2c_.mpu_write_byte(address, subAddress, data);
        if(i2c_err_) std::cout<<"i2c write byte err = "<<i2c_err_<<std::endl;
    }

    uint8_t read_byte(uint8_t address, uint8_t subAddress) {
        return i2c_.mpu_read_byte(address, subAddress);
    }

    void read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, char* dest) {
        int num = i2c_.mpu_read_bytes(address, subAddress, count, dest);
        if(num!=count) std::cout<<"i2c read bytes number = "<<num<<", not "<<count<<std::endl;
    }

    void delay(int milliseconds) {
        i2c_.delay(milliseconds);
    }
};

#endif  // MPU9250_H
