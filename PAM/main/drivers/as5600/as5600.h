#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

void as5600_init(void);
void as5600_set_zero(int channel);
int16_t as5600_get_angle(int channel);

#endif
