#ifndef __SG90_MOTOR_H
#define __SG90_MOTOR_H

#include "stdint.h"

/*********************************/
void SG90_motor_init();
void SG90_motor_set_UD_val(uint16_t val);
void SG90_motor_set_LR_val(uint16_t val);
void SG90_TEST();

#endif
