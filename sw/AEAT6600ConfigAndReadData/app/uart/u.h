#ifndef _U_H
#define _U_H

#define mOK 	0u
#define mNOK 	1u

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
