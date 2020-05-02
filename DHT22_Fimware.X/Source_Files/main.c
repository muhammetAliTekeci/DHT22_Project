/*
 * File:   main.c
 * Author: Muhammet-PC
 *
 * Created on 15 Aral?k 2019 Pazar, 00:37
 */


#include <xc.h>
#include "XC8.h"
#include "stdio.h"

#define TIMER_WRITE(p,v) p##H = v >> 8, p##L = v 


char  PrintBuffer[40];

char  *s = PrintBuffer;
volatile unsigned long Counter;

unsigned char OneSecond = 0;

#define SHCP_PIN    PORTBbits.RB0
#define DS_PIN      PORTBbits.RB1
#define STCP_PIN    PORTBbits.RB2

#define DHT22_TRIS_PIN TRISCbits.TRISC0
#define DHT22_DATA_PIN PORTCbits.RC0

typedef enum 
{
    StartTransmitState = 1,
    WaitFirstTransmitState,
    CheckresponseState,
    StartReadState,
    ParseCollectingDateState,
    StopReadState
    
}Dht22Status;

Dht22Status Dht22CurrentStatus = StartTransmitState;


struct FLAGS
{
    unsigned int SendTxEnable :1;
    unsigned int Dht22StartRead :1;
};

struct FLAGS SystemFlag;

unsigned int PreviousPinValue =0, CurrentPinValue = 0;
unsigned int CurrentShiftBuffer, PreviousShiftBuffer;
#define TIMER_80_US_VALUE             ( 100 )
#define TIMER_RESET_VALUE             ( 30000ul)

uint8_t Timeout = FALSE;

unsigned char shiftRegIndex = 0; 
unsigned int TestArray[20];
uint8_t TempArray[5];
unsigned int CheckSum ,RhDecimalValue, RhFractionVal, TFractionVal, TunsignedDecimalValue;
unsigned int Index;
unsigned int ReceiveIndex;

char Rhmessage[ 20 ] =  {"Nem:     % 00.0 \r"};
char Tmessage [ 25 ] = { "Sicaklik: 00.0 oC \r"};

void main(void) 
{
    SetupConfHandle();
    InitialVariableHande();
    
    while(1)
    {
        OparateSystem();
        if( SystemFlag.SendTxEnable == TRUE )
        {
            Rhmessage[11] = (RhDecimalValue/100)%10 + 48; 
            Rhmessage[12] = (RhDecimalValue/10)%10 + 48; 
            Rhmessage[14] = (RhDecimalValue%10) + 48; 
            
            SerialPrint( Rhmessage );
  
            if( TunsignedDecimalValue > 0x8000 )
               {
                    TunsignedDecimalValue -= ( 1<< 15 );
                    Tmessage[9] ='-';
               }
            else
                Tmessage[9] =' ';
            
            Tmessage[10] = (TunsignedDecimalValue/100)%10 + 48; 
            Tmessage[11] = (TunsignedDecimalValue/10)%10 + 48; 
            Tmessage[13] = (TunsignedDecimalValue%10) + 48; 
            
            SerialPrint( Tmessage );
            
            SystemFlag.SendTxEnable = FALSE;
        }
    }
}


void SetupConfHandle( void ) 
{
    
    INTCONbits.GIE = 0;
    
    TRISBbits.TRISB0 = 0; 
    TRISBbits.TRISB1 = 0; 
    TRISBbits.TRISB2 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC2 = 0;  // input
    
    SHCP_PIN = 0;
    DS_PIN = 0;
    STCP_PIN = 0;
    CurrentShiftBuffer = 0;
    
    //DHT22
    DHT22_TRIS_PIN = 0;     // Output
    DHT22_DATA_PIN = 1;
    
    OPTION_REGbits.T0CS = 0;   //internal oscillator
    OPTION_REGbits.PSA = 0;   // Prescaler ok;
    
    OPTION_REGbits.PS0 = 1; // Prescalar 1:256
    OPTION_REGbits.PS1 = 1;
    OPTION_REGbits.PS2 = 1;
    
    
    //UART
    RCSTAbits.SPEN = 1; //Enable
    RCSTAbits.RX9 = 0;  // 8-bit
    
    
    
    
    TXSTAbits.CSRC = 1; // Master
    TXSTAbits.TX9 = 0;  // 8-bit selection
    TXSTAbits.TXEN = 1; //Transmit enable
    TXSTAbits.SYNC = 0; //asenkron
    TXSTAbits.BRGH = 1; // High Speed
    TXSTAbits.TRMT = 1;
    SPBRG = 31; // 38400 Badrate
    
    
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    PIE1bits.TMR1IE = 0;
    PIR1bits.TMR1IF = 0;
    TMR0 = 0;
    
    //Timer 1 Setup
    T1CONbits.TMR1ON = 0; // Timer 
    T1CONbits.T1CKPS0 = 0; // 1:2 prescalar
    T1CONbits.T1CKPS1 = 1; // 1:2 prescalar
    T1CONbits.T1OSCEN = 0;
    T1CONbits.T1INSYNC = 0; // Synchron
    T1CONbits.TMR1CS = 0; // Internal clock
    TMR1 = 0;  // Timer Val
    T1CONbits.TMR1ON = 1;
    
    PIE1bits.TMR1IE = 0;
    PIR1bits.TMR2IF = 0;
    
}

void InitialVariableHande( void )
{
    SystemFlag.Dht22StartRead = TRUE;
    SystemFlag.SendTxEnable = FALSE;
}

void OparateSystem( void )
{
    if( OneSecond >= 77 )
    {
        OneSecond = 0;
        SystemFlag.Dht22StartRead = TRUE;
                  
        

    
        if( CurrentShiftBuffer != PreviousShiftBuffer )
        {
            SHCP_PIN = 0;
            DS_PIN = 0;
            STCP_PIN = 0;

            for( shiftRegIndex = 0; shiftRegIndex < 16; shiftRegIndex++ )
            {
                if( CurrentShiftBuffer & ( 0x8000 >> shiftRegIndex ))
                    DS_PIN = 1;
                else
                    DS_PIN = 0;

                SHCP_PIN = 1;
                SHCP_PIN = 0;
            }
            STCP_PIN = 1;
            STCP_PIN = 0;

            PreviousShiftBuffer = CurrentShiftBuffer;
        }
    
    }
    
    
    while( SystemFlag.Dht22StartRead == TRUE )
    {
        uint8_t Idx ;
        uint8_t *DataPtr;
        
        PreviousPinValue = CurrentPinValue;
        CurrentPinValue = DHT22_DATA_PIN;
        
        switch( Dht22CurrentStatus )
        {
            case StartTransmitState:
                DHT22_TRIS_PIN = 0;
                DHT22_DATA_PIN = 0;
                TIMER_WRITE( TMR1, 0 );
                CheckSum = 0;
                ReceiveIndex = 0;
                Idx = 0;
                DataPtr = TempArray;
                Dht22CurrentStatus = WaitFirstTransmitState;
                break;

            case WaitFirstTransmitState:
                if( TMR1 > TIMER_RESET_VALUE )
                {
                    DHT22_DATA_PIN = 1;
                    DHT22_TRIS_PIN = 1;
                    TIMER_WRITE( TMR1, 0 );
                    Dht22CurrentStatus = CheckresponseState;
                }
                
                break;
                
            case CheckresponseState:
                if(CurrentPinValue != PreviousPinValue  )
                {
                    if( CurrentPinValue )
                        TIMER_WRITE( TMR1, 0 );
                    else
                    {
                        if( TMR1 > TIMER_80_US_VALUE )
                        {
                            TIMER_WRITE( TMR1, 0 );
                            Dht22CurrentStatus = StartReadState;
                        }
                    }
                }
                else
                {
                    if( TMR1 > 500 )
                    {
                        SerialPrint("Test Fail!\r ");
                        Dht22CurrentStatus = StopReadState;
                    }
                }
                break;

            case StartReadState:
                if(CurrentPinValue != PreviousPinValue )
                {
                    if( CurrentPinValue )
                        TIMER_WRITE( TMR1, 0 );
                    else
                    {
                        if( TMR1 > 20 && TMR1 < 50 )
                            *DataPtr &= ~( 0x80 >> Idx );
                        else
                            *DataPtr |= ( 0x80 >> Idx );

                        if( ++Idx > 7 )
                        {
                            Idx = 0;
                            if(  ++ReceiveIndex < 5 )
                                *DataPtr++;
                            else
                                Dht22CurrentStatus = ParseCollectingDateState;
                        }
                    }
                }
                else if( TMR1 > 500  && CurrentPinValue )
                {
                    SerialPrint("Time out!\r ");
                    Dht22CurrentStatus = StopReadState;
                }
                break;
                
            case ParseCollectingDateState:
                CheckSum = 0;
                 CheckSum = ( TempArray[0] + TempArray[1] + TempArray[2] + TempArray[3] );
                 CheckSum = (CheckSum & 0xFF);
                           

                
                if( CheckSum == TempArray[ 4 ] )
                {
                    RhDecimalValue = TempArray[ 0 ];
                    RhDecimalValue <<= 8;
                    RhDecimalValue |= TempArray[ 1 ];
                    
                    TunsignedDecimalValue = TempArray[ 2 ];
                    TunsignedDecimalValue <<= 8;
                    TunsignedDecimalValue |= TempArray[ 3 ];
                    
                    SystemFlag.SendTxEnable = TRUE;
                    Dht22CurrentStatus = StopReadState;
                }
                else
                {
                    SerialPrint(" CheckSum Error! \r");
                    Dht22CurrentStatus = StopReadState;
                }
                
                 Dht22CurrentStatus = StopReadState;
                break;
                    
            case StopReadState:
                SystemFlag.Dht22StartRead = FALSE;
                Dht22CurrentStatus = StartTransmitState;
                break;
        }
                
    }
}

void SerialPrint( char Text[] )
{
    while( *Text != '\0')
    {
        if( TXSTAbits.TRMT )
        {
            TXREG = *Text++;
        }
    }
}

//13 ms
void __interrupt() T0int(void)
{
   static unsigned char Cnt = 0;
    
    if( INTCONbits.TMR0IE && INTCONbits.TMR0IF )
    {
        if( OneSecond < 77 )
            OneSecond++;
        
        INTCONbits.TMR0IF = 0;
    }
    
    
}
