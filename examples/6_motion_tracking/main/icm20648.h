#ifndef ICM20648_H
#define ICM20648_H

#include <cstdint>
#include <driver/spi_master.h>

struct pwr_mgmt_t{
    bool reset_device;
    bool enable_sleep_mode;
    bool enable_low_power;
    bool disable_temp_sensor;
    unsigned int clock_source;
    unsigned int disable_accel;
    unsigned int disable_gyro;
    pwr_mgmt_t() : 
        reset_device(false),
        enable_sleep_mode(false),
        enable_low_power(false),
        disable_temp_sensor(false),
        clock_source(0),
        disable_accel(0),
        disable_gyro(0) {
    }
};
class icm20648{
    uint8_t transaction(const uint8_t cmd, const uint8_t addr,
        const uint8_t data);
    uint8_t readRegister(const uint8_t address);
    uint8_t writeRegister(const uint8_t address, const uint8_t data);
    bool changeUserBank(const uint8_t bank);
    void spidevInit(const int mosi_io_num, const int miso_io_num,
        const int sclk_io_num, const int cs_io_num);
    
    spi_device_handle_t mHandler;

public:
    icm20648(const int mosi_io_num, const int miso_io_num, 
        const int sclk_io_num, const int cs_io_num);
    ~icm20648(){}
    int readWhoAmI(void);
    void writePwrMgmt(pwr_mgmt_t *mgmt);
};

#endif /* !ICM20648_H */
