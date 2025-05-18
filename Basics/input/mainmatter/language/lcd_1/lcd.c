/*************************************************************************
	LCD
Author: Sergio Santos 
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: all
Date: 12112022
Comment:
	Tested Atemga128 16Mhz and Atmega328 8Mhz                    
************************************************************************/
/***File Library***/
#include "lcd.h"
#include <util/delay.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <stdarg.h>
//#include <math.h>

/***File Variable***/
static LCD0 setup_lcd0;
static LCD1 setup_lcd1;
volatile uint8_t *lcd0_DDR;
volatile uint8_t *lcd0_PIN;
volatile uint8_t *lcd0_PORT;
uint8_t lcd0_detect;
volatile uint8_t *lcd1_DDR;
volatile uint8_t *lcd1_PIN;
volatile uint8_t *lcd1_PORT;
uint8_t lcd1_detect;

/***File Header***/
void LCD0_inic(void);
void LCD0_write(char c, unsigned short D_I);
char LCD0_read(unsigned short D_I);
void LCD0_BF(void);
void LCD0_putch(char c);
char LCD0_getch(void);
void LCD0_string(const char* s); // RAW
void LCD0_string_size(const char* s, uint8_t size); // RAW
void LCD0_hspace(uint8_t n);
void LCD0_clear(void);
void LCD0_gotoxy(unsigned int y, unsigned int x);
void LCD0_reboot(void);
void LCD1_inic(void);
void LCD1_write(char c, unsigned short D_I);
char LCD1_read(unsigned short D_I);
void LCD1_BF(void);
void LCD1_putch(char c);
char LCD1_getch(void);
void LCD1_string(const char* s); // RAW
void LCD1_string_size(const char* s, uint8_t size); // RAW
void LCD1_hspace(uint8_t n);
void LCD1_clear(void);
void LCD1_gotoxy(unsigned int y, unsigned int x);
void LCD1_reboot(void);
void lcd_set_reg(volatile uint8_t* reg, uint8_t hbits);
void lcd_clear_reg(volatile uint8_t* reg, uint8_t hbits);

/***Procedure & Function***/
LCD0 lcd0_enable(volatile uint8_t *ddr, volatile uint8_t *pin, volatile uint8_t *port)
{
	// LOCAL VARIABLES
	// ALLOCACAO MEMORIA PARA Estrutura
	//LCD0 setup_lcd0;
	
	// import parameters
	lcd0_DDR = ddr;
	lcd0_PIN = pin;
	lcd0_PORT = port;
	// initialize variables
	*lcd0_DDR = 0x00;
	*lcd0_PORT = 0xFF;
	lcd0_detect = *lcd0_PIN & (1 << NC);
	// Direccionar apontadores para PROTOTIPOS
	setup_lcd0.write = LCD0_write;
	setup_lcd0.read = LCD0_read;
	setup_lcd0.BF = LCD0_BF;
	setup_lcd0.putch = LCD0_putch;
	setup_lcd0.getch = LCD0_getch;
	setup_lcd0.string = LCD0_string; // RAW
	setup_lcd0.string_size = LCD0_string_size; // RAW
	setup_lcd0.hspace = LCD0_hspace;
	setup_lcd0.clear = LCD0_clear;
	setup_lcd0.gotoxy = LCD0_gotoxy;
	setup_lcd0.reboot = LCD0_reboot;
	// LCD INIC
	LCD0_inic();
	
	return setup_lcd0;
}

LCD0* lcd0(void){ return &setup_lcd0; }

void LCD0_inic(void)
{
	// LCD INIC
	*lcd0_DDR = ((1 << RS) | (1 << RW) | (1 << EN) | (0 << NC));
	*lcd0_PORT = (1 << NC);

	// INICIALIZACAO LCD datasheet
	_delay_ms(40); // using clock at 16Mhz
	LCD0_write(0x30, INST); // 0x30 function set
	_delay_us(37);
	LCD0_write(0x28, INST); // 0x28 function set
	_delay_us(37);
	LCD0_write(0x28, INST); // 0x28 function set
	_delay_us(37);
	LCD0_write(0x0C, INST); // 0x0C Display ON/OFF control
	_delay_us(37);
	LCD0_write(0x01, INST); // 0x01 Display clear
	_delay_ms(2);
	LCD0_write(0x04, INST); // 0x05 Entry mode set
	LCD0_BF();

	LCD0_clear();
	LCD0_gotoxy(0,0);
}
void LCD0_write(char c, unsigned short D_I)
{
	lcd_clear_reg(lcd0_PORT, (1 << RW)); // lcd as input
	lcd_set_reg(lcd0_DDR, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // mcu as output
	
	if(D_I) lcd_set_reg(lcd0_PORT, (1 << RS));  else lcd_clear_reg(lcd0_PORT, (1 << RS));
	
	lcd_set_reg(lcd0_PORT, (1 << EN));
	if(c & 0x80) *lcd0_PORT |= 1 << DB7; else *lcd0_PORT &= ~(1 << DB7);
	if(c & 0x40) *lcd0_PORT |= 1 << DB6; else *lcd0_PORT &= ~(1 << DB6);
	if(c & 0x20) *lcd0_PORT |= 1 << DB5; else *lcd0_PORT &= ~(1 << DB5);
	if(c & 0x10) *lcd0_PORT |= 1 << DB4; else *lcd0_PORT &= ~(1 << DB4);
	lcd_clear_reg(lcd0_PORT, (1 << EN));
	
	if(D_I) lcd_set_reg(lcd0_PORT, (1 << RS));  else lcd_clear_reg(lcd0_PORT, (1 << RS));
	
	lcd_set_reg(lcd0_PORT, (1 << EN));
	if(D_I) *lcd0_PORT |= (1 << RS); else *lcd0_PORT &= ~(1 << RS);
	if(c & 0x08) *lcd0_PORT |= 1 << DB7; else *lcd0_PORT &= ~(1 << DB7);
	if(c & 0x04) *lcd0_PORT |= 1 << DB6; else *lcd0_PORT &= ~(1 << DB6);
	if(c & 0x02) *lcd0_PORT |= 1 << DB5; else *lcd0_PORT &= ~(1 << DB5);
	if(c & 0x01) *lcd0_PORT |= 1 << DB4; else *lcd0_PORT &= ~(1 << DB4);
	lcd_clear_reg(lcd0_PORT, (1 << EN));
}
char LCD0_read(unsigned short D_I)
{
	char c = 0x00;
	lcd_clear_reg(lcd0_DDR, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // mcu as input
	lcd_set_reg(lcd0_PORT, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // pull up resistors
	lcd_set_reg(lcd0_PORT, (1 << RW)); // lcd as output
	
	if(D_I) lcd_set_reg(lcd0_PORT, (1 << RS));  else lcd_clear_reg(lcd0_PORT, (1 << RS));
	
	lcd_set_reg(lcd0_PORT, (1 << EN));
	if(*lcd0_PIN & (1 << DB7)) c |= 1 << 7; else c &= ~(1 << 7);
	if(*lcd0_PIN & (1 << DB6)) c |= 1 << 6; else c &= ~(1 << 6);
	if(*lcd0_PIN & (1 << DB5)) c |= 1 << 5; else c &= ~(1 << 5);
	if(*lcd0_PIN & (1 << DB4)) c |= 1 << 4; else c &= ~(1 << 4);
	lcd_clear_reg(lcd0_PORT, (1 << EN));
	
	if(D_I) lcd_set_reg(lcd0_PORT, (1 << RS));  else lcd_clear_reg(lcd0_PORT, (1 << RS));
	
	lcd_set_reg(lcd0_PORT, (1 << EN));
	if(*lcd0_PIN & (1 << DB7)) c |= 1 << 3; else c &= ~(1 << 3);
	if(*lcd0_PIN & (1 << DB6)) c |= 1 << 2; else c &= ~(1 << 2);
	if(*lcd0_PIN & (1 << DB5)) c |= 1 << 1; else c &= ~(1 << 1);
	if(*lcd0_PIN & (1 << DB4)) c |= 1 << 0; else c &= ~(1 << 0);
	lcd_clear_reg(lcd0_PORT, (1 << EN));
	
	return c;
}
void LCD0_BF(void)
// it has to read at minimum one equal and exit immediately if not equal, weird property.
{
	uint8_t i;
	char inst = 0x80;
	for(i=0; 0x80 & inst; i++){
		inst = LCD0_read(INST);
		if(i > 10)
			break;
	}
}
char LCD0_getch(void)
{
	char c;
	c = LCD0_read(DATA);
	LCD0_BF();
	return c;
}
void LCD0_putch(char c)
{
	LCD0_write(c, DATA);
	LCD0_BF();
}
void LCD0_string(const char* s)
{
	char tmp;
	while(*s){
		tmp = *(s++);
		LCD0_putch(tmp);
	}
}
void LCD0_string_size(const char* s, uint8_t size)
{
	char tmp;
	uint8_t pos = 0;
	while(*s){
		tmp=*(s++);
		pos++;
		if(pos > size) // 1 TO SIZE+1
			break;
		LCD0_putch(tmp);
	}
	while(pos < size){ // TO SIZE
		LCD0_putch(' ');
		pos++;
	}
}
void LCD0_hspace(uint8_t n)
{
	for(; n; n--){
		LCD0_putch(' ');
	}
}
void LCD0_clear(void)
{
	LCD0_write(0x01, INST);
    _delay_ms(1.53);
}
void LCD0_gotoxy(unsigned int y, unsigned int x)
{
	switch(y){
		case 0:
			LCD0_write((0x80 + x), INST);
			LCD0_BF();
		break;
		case 1:
			LCD0_write((0xC0 + x), INST);
			LCD0_BF();
		break;
		case 2:
			LCD0_write((0x94 + x), INST); // 0x94
			LCD0_BF();
		break;
		case 3:
			LCD0_write((0xD4 + x), INST); // 0xD4
			LCD0_BF();
		break;
		default:
		break;
	}
}
void LCD0_reboot(void)
{
	// low high detect pin NC
	uint8_t i;
	uint8_t tmp;
	tmp = *lcd0_PIN & (1 << NC);
	i = tmp ^ lcd0_detect;
	i &= tmp;
	if(i)
		LCD0_inic();
	lcd0_detect = tmp;
}

// LCD 1
LCD1 lcd1_enable(volatile uint8_t *ddr, volatile uint8_t *pin, volatile uint8_t *port)
{
	// LOCAL VARIABLES
	// ALLOCACAO MEMORIA PARA Estrutura
	//LCD1 setup_lcd1;
	
	// import parameters
	lcd1_DDR = ddr;
	lcd1_PIN = pin;
	lcd1_PORT = port;
	// initialize variables
	*lcd1_DDR = 0x00;
	*lcd1_PORT = 0xFF;
	lcd1_detect = *lcd1_PIN & (1 << NC);
	// Direccionar apontadores para PROTOTIPOS
	setup_lcd1.write = LCD1_write;
	setup_lcd1.read = LCD1_read;
	setup_lcd1.BF = LCD1_BF;
	setup_lcd1.putch = LCD1_putch;
	setup_lcd1.getch = LCD1_getch;
	setup_lcd1.string = LCD1_string; // RAW
	setup_lcd1.string_size = LCD1_string_size; // RAW
	setup_lcd1.hspace = LCD1_hspace;
	setup_lcd1.clear = LCD1_clear;
	setup_lcd1.gotoxy = LCD1_gotoxy;
	setup_lcd1.reboot = LCD1_reboot;
	// LCD INIC
	LCD1_inic();
	
	return setup_lcd1;
}

LCD1* lcd1(void){ return &setup_lcd1; }

void LCD1_inic(void)
{
	// LCD INIC
	*lcd1_DDR = (1 << RS) | (1 << RW) | (1 << EN) | (0 << NC);
	*lcd1_PORT = (1 << NC);

	// INICIALIZACAO LCD datasheet
	_delay_ms(40); // using clock at 16Mhz
	LCD1_write(0x30, INST); // 0x30 function set
	_delay_us(37);
	LCD1_write(0x28, INST); // 0x28 function set
	_delay_us(37);
	LCD1_write(0x28, INST); // 0x28 function set
	_delay_us(37);
	LCD1_write(0x0C, INST); // 0x0C Display ON/OFF control
	_delay_us(37);
	LCD1_write(0x01, INST); // 0x01 Display clear
	_delay_ms(2);
	LCD1_write(0x04, INST); // 0x05 Entry mode set
	LCD1_BF();

	LCD1_clear();
	LCD1_gotoxy(0,0);
}
void LCD1_write(char c, unsigned short D_I)
{
	lcd_clear_reg(lcd1_PORT, (1 << RW)); // lcd as input
	lcd_set_reg(lcd1_DDR, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // mcu as output
	
	if(D_I) lcd_set_reg(lcd1_PORT, (1 << RS));  else lcd_clear_reg(lcd1_PORT, (1 << RS));
	
	lcd_set_reg(lcd1_PORT, (1 << EN));
	if(c & 0x80) *lcd1_PORT |= 1 << DB7; else *lcd1_PORT &= ~(1 << DB7);
	if(c & 0x40) *lcd1_PORT |= 1 << DB6; else *lcd1_PORT &= ~(1 << DB6);
	if(c & 0x20) *lcd1_PORT |= 1 << DB5; else *lcd1_PORT &= ~(1 << DB5);
	if(c & 0x10) *lcd1_PORT |= 1 << DB4; else *lcd1_PORT &= ~(1 << DB4);
	lcd_clear_reg(lcd1_PORT, (1 << EN));
	
	if(D_I) lcd_set_reg(lcd1_PORT, (1 << RS));  else lcd_clear_reg(lcd1_PORT, (1 << RS));
	
	lcd_set_reg(lcd1_PORT, (1 << EN));
	if(D_I) *lcd1_PORT |= (1 << RS); else *lcd1_PORT &= ~(1 << RS);
	if(c & 0x08) *lcd1_PORT |= 1 << DB7; else *lcd1_PORT &= ~(1 << DB7);
	if(c & 0x04) *lcd1_PORT |= 1 << DB6; else *lcd1_PORT &= ~(1 << DB6);
	if(c & 0x02) *lcd1_PORT |= 1 << DB5; else *lcd1_PORT &= ~(1 << DB5);
	if(c & 0x01) *lcd1_PORT |= 1 << DB4; else *lcd1_PORT &= ~(1 << DB4);
	lcd_clear_reg(lcd1_PORT, (1 << EN));
}
char LCD1_read(unsigned short D_I)
{
	char c = 0x00;
	lcd_clear_reg(lcd1_DDR, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // mcu as input
	lcd_set_reg(lcd1_PORT, (1 << DB4) | (1 << DB5) | (1 << DB6) | (1 << DB7)); // pull up resistors
	lcd_set_reg(lcd1_PORT, (1 << RW)); // lcd as output
	
	if(D_I) lcd_set_reg(lcd1_PORT, (1 << RS));  else lcd_clear_reg(lcd1_PORT, (1 << RS));
	
	lcd_set_reg(lcd1_PORT, (1 << EN));
	if(*lcd1_PIN & (1 << DB7)) c |= 1 << 7; else c &= ~(1 << 7);
	if(*lcd1_PIN & (1 << DB6)) c |= 1 << 6; else c &= ~(1 << 6);
	if(*lcd1_PIN & (1 << DB5)) c |= 1 << 5; else c &= ~(1 << 5);
	if(*lcd1_PIN & (1 << DB4)) c |= 1 << 4; else c &= ~(1 << 4);
	lcd_clear_reg(lcd1_PORT, (1 << EN));
	
	if(D_I) lcd_set_reg(lcd1_PORT, (1 << RS));  else lcd_clear_reg(lcd1_PORT, (1 << RS));
	
	lcd_set_reg(lcd1_PORT, (1 << EN));
	if(*lcd1_PIN & (1 << DB7)) c |= 1 << 3; else c &= ~(1 << 3);
	if(*lcd1_PIN & (1 << DB6)) c |= 1 << 2; else c &= ~(1 << 2);
	if(*lcd1_PIN & (1 << DB5)) c |= 1 << 1; else c &= ~(1 << 1);
	if(*lcd1_PIN & (1 << DB4)) c |= 1 << 0; else c &= ~(1 << 0);
	lcd_clear_reg(lcd1_PORT, (1 << EN));
	
	return c;
}
void LCD1_BF(void)
{
	uint8_t i;
	char inst = 0x80;
	for(i=0; (0x80 & inst); i++){ // it has to read at minimum one equal and exit immediately if not equal, weird property.
		inst = LCD0_read(INST);
		if(i > 10)
			break;
	}
}
char LCD1_getch(void)
{
	char c;
	c = LCD1_read(DATA);
	LCD1_BF();
	return c;
}
void LCD1_putch(char c)
{
	LCD1_write(c, DATA);
	LCD1_BF();
}
void LCD1_string(const char* s)
{
	char tmp;
	while(*s){
		tmp = *(s++);
		LCD1_putch(tmp);
	}
}
void LCD1_string_size(const char* s, uint8_t size)
{
	char tmp;
	uint8_t pos = 0;
	while(*s){
		tmp = *(s++);
		pos++;
		if(pos > size)
			break;
		LCD1_putch(tmp);
	}
	while(pos < size){
		LCD1_putch(' ');
		pos++;
	}
}
void LCD1_hspace(uint8_t n)
{
	for(; n; n--){
		LCD1_putch(' ');
	}
}
void LCD1_clear(void)
{
	LCD1_write(0x01, INST);
	_delay_ms(1.53);
}
void LCD1_gotoxy(unsigned int y, unsigned int x)
{
	switch(y){
		case 0:
			LCD1_write((0x80 + x), INST);
			LCD1_BF();
		break;
		case 1:
			LCD1_write((0xC0 + x), INST);
			LCD1_BF();
		break;
		case 2:
			LCD1_write((0x94 + x), INST); // 0x94
			LCD1_BF();
		break;
		case 3:
			LCD1_write((0xD4 + x), INST); // 0xD4
			LCD1_BF();
		break;
		default:
		break;
	}
}
void LCD1_reboot(void)
{
	// low high detect pin NC
	uint8_t i;
	uint8_t tmp;
	tmp = *lcd1_PIN & (1 << NC);
	i = tmp ^ lcd1_detect;
	i &= tmp;
	if(i)
		LCD1_inic();
	lcd1_detect = tmp;
}
void lcd_set_reg(volatile uint8_t* reg, uint8_t hbits){
	*reg |= hbits;
}
void lcd_clear_reg(volatile uint8_t* reg, uint8_t hbits){
	*reg &= ~hbits;
}

/***EOF***/

