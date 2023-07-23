/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


#include "derivative.h" /* include peripheral declarations SSKEAZ128M4 */

#define APP_START_ADDR          0x6000

static void JumpToApplication(uint32_t start_address)
{
	if(*(volatile uint32_t*)start_address != 0XFFFFFFFF)
	{
		void (*entry)(void);
		uint32_t pc, sp;

		SCB->VTOR=(uint32_t)(start_address);      /*Relocate interrupt table ptr*/
		sp = *((volatile uint32_t*)start_address);
		__set_PSP(sp);

		pc = *((volatile uint32_t*)(start_address + 4));
		entry = (void (*)(void))pc;
		entry();
	}
}

int main(void)
{
#define COUNTER_LIMIT 100

        int counter = 0;

        JumpToApplication(APP_START_ADDR);

        for(;;) {       
            counter++;

            if(counter > COUNTER_LIMIT) {
                counter = 0;
            }
        }

    /* to avoid the warning message for GHS: statement is unreachable*/
#if defined (__ghs__)
#pragma ghs nowarning 111
#endif
#if defined (__ICCARM__)
#pragma diag_suppress=Pe111
#endif
	return 0;
}
