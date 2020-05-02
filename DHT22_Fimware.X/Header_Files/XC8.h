//Author: Muhammet Ali Tekeci
#include <xc.h>

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 20000000

#define TRUE 1
#define FALSE 0


typedef signed int int8_t;
typedef unsigned int uint8_t;
typedef unsigned char uchar_t;
typedef signed char schar_t;
typedef unsigned long uint16_t;
typedef signed long int16_t;

void SetupConfHandle( void );
void __interrupt() T0int(void);
void OparateSystem( void );
void InitialVariableHande( void );
void SerialPrint( char Text[] );