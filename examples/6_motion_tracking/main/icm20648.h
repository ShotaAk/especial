#ifndef ICM20648_H
#define ICM20648_H

#include <stdint.h>

namespace ICM20648
{
    extern uint8_t read_who_am_i(void);
    extern void init(void);
}

#endif /* !ICM20648_H */
