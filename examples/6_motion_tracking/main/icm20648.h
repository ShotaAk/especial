#ifndef ICM20648_H
#define ICM20648_H

#include <cstdint>
#include <driver/spi_master.h>

struct pwr_mgmt_t{
    bool reset_device = false;
    bool enable_sleep_mode = false;
    bool enable_low_power = false;
    bool disable_temp_sensor = false;
    unsigned int clock_source = 0;
    unsigned int disable_accel = 0;
    unsigned int disable_gyro = 0;
};

enum AXIS{
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
    AXIS_SIZE
};
class icm20648{
    uint8_t transaction(const uint8_t cmd, const uint8_t addr,
        const uint8_t data);
    uint8_t readRegister(const uint8_t address);
    uint16_t readRegister2Byte(const uint8_t addrHigh, const uint8_t addrLow);
    uint8_t writeRegister(const uint8_t address, const uint8_t data);
    bool changeUserBank(const uint8_t bank);
    void spidevInit(const int mosi_io_num, const int miso_io_num,
        const int sclk_io_num, const int cs_io_num);
    bool writeGyroConfig(const uint8_t fssel,
        const bool enableLPF, const uint8_t configLPF);
    float getGyro(const AXIS axis);
    
    spi_device_handle_t mHandler;
    uint8_t mGyroFSSel;

    const static int FS_SEL_SIZE;
    const static float GYRO_SENSITIVITY[];
    const static float ACCEL_SENSITIVITY[];

public:
    icm20648(const int mosi_io_num, const int miso_io_num, 
        const int sclk_io_num, const int cs_io_num);
    ~icm20648(){}
    int readWhoAmI(void);
    void writePwrMgmt(pwr_mgmt_t *mgmt);
    float getGyroX(void);
    float getGyroY(void);
    float getGyroZ(void);
};

#endif /* !ICM20648_H */
