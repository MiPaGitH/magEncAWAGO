#ifndef _U_H
#define _U_H

enum
{
	read,
	write
};

enum
{
	clk,
	dat
};

extern void uInit( void );
extern void uTask( void );

#endif
