#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

void DelayInit();
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);

#endif /* __DELAY_H */
