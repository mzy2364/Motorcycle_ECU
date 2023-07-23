/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


#include "derivative.h" /* include peripheral declarations SSKEAZ128M4 */
#include "task.h"

int main(void)
{
	task_init();

	for(;;)
	{
		task_scheduler();
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
