#include "delay.h"

static volatile uint32_t usTicks = 0;

void SysTick_Handler(void)
{
	if (usTicks > 0) usTicks--;
}

void DelayInit() {
	while(SysTick_Config(SystemCoreClock / 1000000));
}

void DelayUs(uint32_t us)
{
	// Reload us value
	usTicks = us;
	// Wait until usTick reach zero
	while (usTicks);
}

void DelayMs(uint32_t ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
		// Delay 1ms
		DelayUs(1000);
	}
}
