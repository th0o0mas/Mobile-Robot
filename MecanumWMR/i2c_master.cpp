/*gcc -Wall -g -pthread -o prog prog.c -lpigpiod_if2 -lrt -lm*/

#include <cstring>
#include "pigpiod_if2.h"
#include "i2c_master.h"


bool I2C::init() {
	if ((pi=pigpio_start(NULL,NULL)) < 0) return false;
	return true;
}

void I2C::shutdown() {pigpio_stop(pi);}

void I2C::EnableSTM32(void) {
    int handle = i2c_open(pi, 1, STM_ADDRESS, 0);
    char command[2]={1,1};
    i2c_write_device(pi, handle, command, 2);
    i2c_close(pi, handle);
}

void I2C::DisableSTM32(void) {
    int handle = i2c_open(pi, 1, STM_ADDRESS, 0);
    char command[2]={1,0};
    i2c_write_device(pi, handle, command, 2);
    i2c_close(pi, handle);
}

void I2C::ReadSTM32Signal(char *const signal) {
    int handle = i2c_open(pi, 1, STM_ADDRESS, 0);
	i2c_read_device(pi, handle, signal, 1);
    i2c_close(pi, handle);
}

void I2C::SendSTM32(const char *const wheel_v_ref) {
    int handle = i2c_open(pi, 1, STM_ADDRESS, 0);
	char command_buffer[sizeof(float)*WHEEL_NUM + 1] = {sizeof(float)*WHEEL_NUM};
	memcpy(command_buffer+1, wheel_v_ref, command_buffer[0]);
	i2c_write_device(pi, handle, command_buffer, sizeof(command_buffer));
    i2c_close(pi, handle);
}

void I2C::ReadSTM32(char *const wheel_v_est) {
    int handle = i2c_open(pi, 1, STM_ADDRESS, 0);
    i2c_read_device(pi, handle, wheel_v_est, sizeof(float)*WHEEL_NUM);
    i2c_close(pi, handle);
}

int I2C::mpu_write_byte(uint8_t address, uint8_t subAddress, uint8_t data) {
	int mpu_handle = i2c_open(pi, 1, address, 0);
	int i2c_err = i2c_write_byte_data(pi, mpu_handle, subAddress, data);
	i2c_close(pi, mpu_handle);
	return i2c_err;
}

uint8_t I2C::mpu_read_byte(uint8_t address, uint8_t subAddress) {
	uint8_t data = 127;
	int mpu_handle = i2c_open(pi, 1, address, 0);
	data = i2c_read_byte_data(pi, mpu_handle, subAddress);
	i2c_close(pi, mpu_handle);
	return data;
}

int I2C::mpu_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, char* dest) {
	int mpu_handle = i2c_open(pi, 1, address, 0);
	int num_bytes = i2c_read_i2c_block_data(pi, mpu_handle, subAddress, dest, count);
	i2c_close(pi, mpu_handle);
	return num_bytes;
}

void I2C::delay(int milliseconds) {time_sleep(milliseconds*0.001);}




