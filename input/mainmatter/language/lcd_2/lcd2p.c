/*************************************************************************
	LCD2P
Author: Sergio Santos 
	<sergio.salazar.santos@gmail.com>
License: GNU General Public License
Hardware: all
Date: 20042023
************************************************************************/
/*** Comment:
	Tested Atemga128 16Mhz and Atmega328 8Mhz                    
************************************************************************/
/*** File Library ***/
#include "lcd2p.h"
#include <util/delay.h>

/*** File Constant & Macro ***/
// CMD RS
#define LCD02P_INST 0
#define LCD02P_DATA 1

/*** File Variable ***/
static LCD02P setup_lcd02p;
volatile uint8_t *lcd02pcmd_DDR;
volatile uint8_t *lcd02pcmd_PIN;
volatile uint8_t *lcd02pcmd_PORT;
volatile uint8_t *lcd02pdata_DDR;
volatile uint8_t *lcd02pdata_PIN;
volatile uint8_t *lcd02pdata_PORT;
static uint8_t DDR_DATA_MASK;
uint8_t lcd02p_detect;

/*** File Header ***/
void LCD02P_inic(void);
void LCD02P_write(char c, unsigned short D_I);
char LCD02P_read(unsigned short D_I);
void LCD02P_BF(void);
void LCD02P_putch(char c);
char LCD02P_getch(void);
void LCD02P_string(const char* s); // RAW
void LCD02P_string_size(const char* s, uint8_t size); // RAW
void LCD02P_hspace(uint8_t n);
void LCD02P_clear(void);
void LCD02P_gotoxy(unsigned int y, unsigned int x);
void lcd02p_set_reg(volatile uint8_t* reg, uint8_t hbits);
void lcd02p_clear_reg(volatile uint8_t* reg, uint8_t hbits);
void LCD02P_reboot(void);

/*** Procedure & Function ***/
LCD02P lcd02p_enable(volatile uint8_t *cmdddr, volatile uint8_t *cmdpin, volatile uint8_t *cmdport, volatile uint8_t *dataddr, volatile uint8_t *datapin, volatile uint8_t *dataport)
{
	
	// import parameters
	lcd02pcmd_DDR = cmdddr;
	lcd02pcmd_PIN = cmdpin;
	lcd02pcmd_PORT = cmdport;
	lcd02pdata_DDR = dataddr;
	lcd02pdata_PIN = datapin;
	lcd02pdata_PORT = dataport;
	DDR_DATA_MASK = (1 << LCD02P_DB4) | (1 << LCD02P_DB5) | (1 << LCD02P_DB6) | (1 << LCD02P_DB7);
	// Direccionar apontadores para PROTOTIPOS
	setup_lcd02p.write = LCD02P_write;
	setup_lcd02p.read = LCD02P_read;
	setup_lcd02p.BF = LCD02P_BF;
	setup_lcd02p.putch = LCD02P_putch;
	setup_lcd02p.getch = LCD02P_getch;
	setup_lcd02p.string = LCD02P_string; // RAW
	setup_lcd02p.string_size = LCD02P_string_size; // RAW
	setup_lcd02p.hspace = LCD02P_hspace;
	setup_lcd02p.clear = LCD02P_clear;
	setup_lcd02p.gotoxy = LCD02P_gotoxy;
	setup_lcd02p.reboot = LCD02P_reboot;
	// LCD INIC
	LCD02P_inic();
	
	return setup_lcd02p;
}

LCD02P* lcd02p(void){ return &setup_lcd02p; }

void LCD02P_inic(void)
{
	// LCD INIC
	*lcd02pcmd_DDR |= (1 << LCD02P_RS) | (1 << LCD02P_RW) | (1 << LCD02P_EN) | (0 << LCD02P_NC);
	*lcd02pcmd_PORT |= (1 << LCD02P_NC);
	*lcd02pdata_DDR &= ~DDR_DATA_MASK;
	*lcd02pdata_PORT |= DDR_DATA_MASK;
	lcd02p_detect = *lcd02pcmd_PIN & (1 << LCD02P_NC);

	// INICIALIZACAO LCD datasheet
	_delay_ms(40); // using clock at 16Mhz
	LCD02P_write(0x30, LCD02P_INST); // 0x30 function set
	_delay_us(37);
	LCD02P_write(0x28, LCD02P_INST); // 0x28 function set
	_delay_us(37);
	LCD02P_write(0x28, LCD02P_INST); // 0x28 function set
	_delay_us(37);
	LCD02P_write(0x0C, LCD02P_INST); // 0x0C Display ON/OFF control
	_delay_us(37);
	LCD02P_write(0x01, LCD02P_INST); // 0x01 Display clear
	_delay_ms(2);
	LCD02P_write(0x04, LCD02P_INST); // 0x05 Entry mode set
	LCD02P_BF();

	LCD02P_clear();
	LCD02P_gotoxy(0,0);
}
void LCD02P_write(char c, unsigned short D_I)
{
	lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_RW)); // lcd as input
	lcd02p_set_reg(lcd02pdata_DDR, DDR_DATA_MASK); // mcu as output
	
	if(D_I) lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));  else lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));
	
	lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	if(c & 0x80) *lcd02pdata_PORT |= 1 << LCD02P_DB7; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB7);
	if(c & 0x40) *lcd02pdata_PORT |= 1 << LCD02P_DB6; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB6);
	if(c & 0x20) *lcd02pdata_PORT |= 1 << LCD02P_DB5; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB5);
	if(c & 0x10) *lcd02pdata_PORT |= 1 << LCD02P_DB4; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB4);
	lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	
	if(D_I) lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));  else lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));
	
	lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	if(c & 0x08) *lcd02pdata_PORT |= 1 << LCD02P_DB7; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB7);
	if(c & 0x04) *lcd02pdata_PORT |= 1 << LCD02P_DB6; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB6);
	if(c & 0x02) *lcd02pdata_PORT |= 1 << LCD02P_DB5; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB5);
	if(c & 0x01) *lcd02pdata_PORT |= 1 << LCD02P_DB4; else *lcd02pdata_PORT &= ~(1 << LCD02P_DB4);
	lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
}
char LCD02P_read(unsigned short D_I)
{
	char c = 0x00;
	lcd02p_clear_reg(lcd02pdata_DDR, DDR_DATA_MASK); // mcu as input
	lcd02p_set_reg(lcd02pdata_PORT, DDR_DATA_MASK); // pull up resistors
	lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_RW)); // lcd as output
	
	if(D_I) lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));  else lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));
	
	lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	if(*lcd02pdata_PIN & (1 << LCD02P_DB7)) c |= 1 << 7; else c &= ~(1 << 7);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB6)) c |= 1 << 6; else c &= ~(1 << 6);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB5)) c |= 1 << 5; else c &= ~(1 << 5);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB4)) c |= 1 << 4; else c &= ~(1 << 4);
	lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	
	if(D_I) lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));  else lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_RS));
	
	lcd02p_set_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	if(*lcd02pdata_PIN & (1 << LCD02P_DB7)) c |= 1 << 3; else c &= ~(1 << 3);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB6)) c |= 1 << 2; else c &= ~(1 << 2);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB5)) c |= 1 << 1; else c &= ~(1 << 1);
	if(*lcd02pdata_PIN & (1 << LCD02P_DB4)) c |= 1 << 0; else c &= ~(1 << 0);
	lcd02p_clear_reg(lcd02pcmd_PORT, (1 << LCD02P_EN));
	
	return c;
}
void LCD02P_BF(void)
// it has to read at minimum one equal and exit immediately if not equal, weird property.
{
	uint8_t i;
	char inst = 0x80;
	for(i=0; 0x80 & inst; i++){
		inst = LCD02P_read(LCD02P_INST);
		if(i > 1)
			break;
	}
}
char LCD02P_getch(void)
{
	char c;
	c = LCD02P_read(LCD02P_DATA);
	LCD02P_BF();
	return c;
}
void LCD02P_putch(char c)
{
	LCD02P_write(c, LCD02P_DATA);
	LCD02P_BF();
}
void LCD02P_string(const char* s)
{
	char tmp;
	while(*s){
		tmp = *(s++);
		LCD02P_putch(tmp);
	}
}
void LCD02P_string_size(const char* s, uint8_t size)
{
	char tmp;
	uint8_t pos = 0;
	while(*s){
		tmp=*(s++);
		pos++;
		if(pos > size) // 1 TO SIZE+1
			break;
		LCD02P_putch(tmp);
	}
	while(pos<size){ // TO SIZE
		LCD02P_putch(' ');
		pos++;
	}
}
void LCD02P_hspace(uint8_t n)
{
	for(; n; n--){
		LCD02P_putch(' ');
	}
}
void LCD02P_clear(void)
{
	LCD02P_write(0x01, LCD02P_INST);
    _delay_ms(1.53);
}
void LCD02P_gotoxy(unsigned int y, unsigned int x)
{
	switch(y){
		case 0:
			LCD02P_write((0x80 + x), LCD02P_INST);
			LCD02P_BF();
			break;
		case 1:
			LCD02P_write((0xC0 + x), LCD02P_INST);
			LCD02P_BF();
			break;
		case 2:
			LCD02P_write((0x94 + x), LCD02P_INST); // 0x94
			LCD02P_BF();
			break;
		case 3:
			LCD02P_write((0xD4 + x), LCD02P_INST); // 0xD4
			LCD02P_BF();
			break;
		default:
			break;
	}
}
void lcd02p_set_reg(volatile uint8_t* reg, uint8_t hbits){
	*reg |= hbits;
}
void lcd02p_clear_reg(volatile uint8_t* reg, uint8_t hbits){
	*reg &= ~hbits;
}
void LCD02P_reboot(void)
{
	// low high detect pin NC
	uint8_t i;
	uint8_t tmp;
	tmp = *lcd02pcmd_PIN & (1 << LCD02P_NC);
	i = tmp ^ lcd02p_detect;
	i &= tmp;
	if(i)
		LCD02P_inic();
	lcd02p_detect = tmp;
}

/*** EOF ***/

