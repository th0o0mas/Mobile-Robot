#ifndef __I2C_H
#define __I2C_H


#include <cstdint>
#ifndef WHEEL_NUM
#define WHEEL_NUM 4
#endif
#define STM_ADDRESS 0x12

class I2C
{
    public:
        I2C(): pi(0) {}
        bool init(void);
        void shutdown(void);
        void EnableSTM32(void);
        void DisableSTM32(void);
        void ReadSTM32Signal(char *const signal);
        void SendSTM32(const char *const wheel_v_ref);
        void ReadSTM32(char *const wheel_v_est);
        int mpu_write_byte(uint8_t address, uint8_t subAddress, uint8_t data);
        uint8_t mpu_read_byte(uint8_t address, uint8_t subAddress);
        int mpu_read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, char* dest);
        void delay(int milliseconds);
    private:
        int pi;
};

#endif // __I2C_H
