#ifndef ICM20648_H
#define ICM20648_H

namespace ICM20648
{
    extern void init(const int mosi_io_num, const int miso_io_num, 
        const int sclk_io_num, const int cs_io_num);
    extern int read_who_am_i(void);
}

#endif /* !ICM20648_H */
