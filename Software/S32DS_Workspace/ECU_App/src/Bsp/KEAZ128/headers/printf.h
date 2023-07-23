#ifndef	__PRINT_H
#define __PRINT_H
#include "uart.h"

#define COM_PORT  UART2

#ifdef	FALSE
#undef	FALSE
#endif
#define FALSE	(0)


#ifdef	TRUE
#undef	TRUE
#endif
#define	TRUE	(1)


/********************************************************************/

char	
in_char(void);

void
out_char(char);

int
char_present(void);

int		
printf(const char *, ... );

int
sprintf(char *, const char *, ... );


/********************************************************************/







#endif
