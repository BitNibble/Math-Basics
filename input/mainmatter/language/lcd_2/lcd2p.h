/************************************************************************
	LCD2P
Author: Sergio Santos 
   <sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: all
Date: 20042023
************************************************************************/
/****** Comment:
	tested Atemga128 16Mhz and Atmega328 8Mhz
************************************************************************/
#ifndef _LCD2P_H_
	#define _LCD2P_H_

/*** Global Library ***/
#include <inttypes.h>

/*** Global Constant & Macro ***/
#ifndef F_CPU
	#define F_CPU 16000000UL
#endif
// ASIGN PORT PINS TO LCD (can be setup in any way)
// CMD
#define LCD02P_RS 0
#define LCD02P_RW 1
#define LCD02P_EN 2
#define LCD02P_NC 3
// DATA
#define LCD02P_DB4 0 // 4
#define LCD02P_DB5 1 // 5
#define LCD02P_DB6 2 // 6
#define LCD02P_DB7 3 // 7

/*** Global Variable ***/
typedef struct{
	void (*write)(char c, unsigned short D_I);
	char (*read)(unsigned short D_I);
	void (*BF)(void);
	void (*putch)(char c);
	char (*getch)(void);
	void (*string)(const char *s); // RAW
	void (*string_size)(const char* s, uint8_t size); // RAW
	void (*hspace)(uint8_t n);
	void (*clear)(void);
	void (*gotoxy)(unsigned int y, unsigned int x);
	void (*reboot)(void);
}LCD02P;

/*** Global Header ***/
LCD02P lcd02p_enable(volatile uint8_t *cmdddr, volatile uint8_t *cmdpin, volatile uint8_t *cmdport, volatile uint8_t *dataddr, volatile uint8_t *datapin, volatile uint8_t *dataport);
LCD02P* lcd02p(void);

#endif

/*** EOF ***/

