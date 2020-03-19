#ifndef ICM20648_H
#define ICM20648_H

namespace ICM20648
{
    void init(const int mosi_io_num, const int miso_io_num,
        const int sclk_io_num, const int cs_io_num);
    int read_who_am_i(void);
    int read_accel_x(void);
}

#endif /* !ICM20648_H */
