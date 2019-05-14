/*
 * File:   main.c
 * Author: Conan
 *
 * Created on April 24, 2019, 11:12 AM
 */
// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (1x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (24x Multiplier)
#pragma config UPLLIDIV = DIV_2 // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 256)
// DEVCFG1
#pragma config FNOSC = PRIPLL // Oscillator Selection Bits (Primary Osc w/PLL (XT+,HS+,EC+PLL))
#pragma config FSOSCEN = ON // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = HS // Primary Oscillator Configuration (HS osc mode)
#pragma config OSCIOFNC = ON // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_8 // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/8)
#pragma config FCKSM = CSDCMD // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576 // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
// DEVCFG0
#pragma config DEBUG = OFF // Background Debugger Enable (Debugger is disabled)
#pragma config ICESEL = ICS_PGx2 // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF // Program Flash Write Protect (Disable)
#pragma config BWP = OFF // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF // Code Protect (Protection Disabled)
/// timer
// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 40 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//
//#pragma config FPLLMUL = MUL_20
//#pragma config FPLLIDIV = DIV_2
//#pragma config FPLLODIV = DIV_1
//#pragma config FWDTEN = OFF
//#pragma config POSCMOD = HS
//#pragma config FNOSC = PRIPLL
//#pragma config FPBDIV = DIV_8


// Let compile time pre-processor calculate the PR1 (period)
#define SYS_FREQ 			(80000000L)
#define PB_DIV         		8
#define PRESCALE       		256
#define TOGGLES_PER_SEC		1000
#define T1_TICK       		(SYS_FREQ/PB_DIV/PRESCALE/TOGGLES_PER_SEC)
//\ timer
// 3000 ms
#define SuccessElapsed      3000

// 2000 ms
#define Timeout             2000


#include <xc.h>

#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.
#define SYSCLK 80000000L // Give the system?s clock frequency



/*  PORT MAPPING
    RD2     D0      >>>>> CLK
    RD3     D1      >>>>> DATA
 *  RD4     D2      (not used)
    RD5 	D3      >>>> output
    RD6 	D4      Monitor state receive > set data
    RD7 	D5      Monitor state receive > wait next clock
    RD8 	D6      Monitor state check data > 
    RD11	D7
 * 
    RB13	D8      Monitor CLK
    RB14	D9      MONITOR DATA
 * 
    RG9		D10     Error bit
 * PORTS:
 *      D:  0x09E0
 *      B:	0x6000
 *      G:	0x0200
 *  
 */

#define StatePOR 0x00
#define StateIDLE 0x10
#define StateINIT_RECEIVE 0x20
#define StateRECEIVE 0x30
#define StateRECEIVE_RAISING_EDGE 0x35
#define StateCHECK_NEXT 0x40
#define StateCHECK_DATA 0x50
#define StateSUCCESS 0x60
#define StateWAIT_RESET 0x70

char timer_event = 0;
// holds the current execution state
char programState = StatePOR;
// holds the current led index

void setErrorBit()
{
    LATGbits.LATG9 = 1;
}
void resetErrorBit()
{
    LATGbits.LATG9 = 0;
}
void SetLedON()
{
    LATDbits.LATD5 = 1;
}

void SetledOFF()
{
    LATDbits.LATD5 = 0;
}


int main(void){
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //STEP 1. Configure cache, wait states and peripheral bus clock
	// Configure the device for maximum performance but do not change the PBDIV
	// Given the options, this function will change the flash wait states, RAM
	// wait state and enable prefetch cache but will not change the PBDIV.
	// The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // STEP 2. configure Timer 1 using internal clock, 1:256 prescale

    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, T1_TICK);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);

    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();

	// configure PORTs
    mJTAGPortEnable(0);                      // Disable JTAG
    PORTSetPinsDigitalOut(IOPORT_B, BIT_13); // Configure pin as output.
    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_14); // Configure pin as output.
    PORTSetPinsDigitalOut(IOPORT_G, BIT_9); // Configure pin as output.

    
    PORTSetPinsDigitalOut(IOPORT_D, BIT_8); // Configure pin as output.
    PORTSetPinsDigitalOut(IOPORT_D, BIT_7); // Configure pin as output.
    PORTSetPinsDigitalOut(IOPORT_D, BIT_6); // Configure pin as output.
    PORTSetPinsDigitalOut(IOPORT_D, BIT_5); // Configure pin as output.
    
    PORTSetPinsDigitalIn(IOPORT_D, BIT_2); // CLOCK
    PORTSetPinsDigitalIn(IOPORT_D, BIT_3); // DATA
    
    // bit position counter
    // set = 0 at first clock, it is incremented during reception    
    char i;   
    // data buffer, each bit is set during reception
    char buffer;
    // serial comunication timeout monitor
    long WaitTime = -1;
    
    long SuccessCounter = -1;
    char SuccessFlag = 0;
    
    while( 1)
    {
        // monitor the input activity: copy to D8 & D9 
        LATBbits.LATB13 = PORTDbits.RD2;
        LATBbits.LATB14 = PORTDbits.RD3;
        
        if (timer_event == 1)
        {
            timer_event = 0;
            if (WaitTime > -1){
                if (WaitTime > Timeout)// transmission broken
                {
                    setErrorBit(); // set error led
                    programState = StateIDLE;
                    WaitTime = -2;
                }
                WaitTime++;
            }           
            if (SuccessCounter > -1){
                if (SuccessCounter > SuccessElapsed)// transmission broken
                {
                    SuccessFlag = 1;
                    SuccessCounter = -2;
                }
                SuccessCounter++;
            }
        }
        
        switch(programState) 
        {
            case StatePOR  :
                // Power On Reset

                // reset all ports
                LATB =0;
                LATD =0;
                LATG =0;
                // monitor the errorBit on startup
                setErrorBit();
                programState = StateIDLE;    
                
            case StateIDLE  :                
                // wait for the first RAISING EDGE on the Clock bit of the Clock PORT
                // remain on this state while RD2 = False
                if (PORTDbits.RD2 == 0)
                    break;                  
                // exit state condition found                
                programState = StateINIT_RECEIVE;
                
            case StateINIT_RECEIVE  :
                // initialize session
                
                // reset monitor bits
                resetErrorBit();
                LATDbits.LATD6 = 0;
                LATDbits.LATD7 = 0;
                LATDbits.LATD8 = 0;
                
                i = 0;
                buffer = 0;
                programState = StateRECEIVE;       
                
            case StateRECEIVE  :
                // load data into buffer
                buffer |= PORTDbits.RD3<< i;
                // increment buffer position for the next loop
                i++;    
                programState = StateRECEIVE_RAISING_EDGE;
                // reset timeout
                WaitTime = 0;
                // set Receive monitor
                LATDbits.LATD6 = 1;
                
            case StateRECEIVE_RAISING_EDGE :
                if (i>7)
                {
                    // transmission complete
                    programState = StateCHECK_DATA;
                    break;
                }
                // wait for the FALLING EDGE
                // remain on this state while RD2 = True
                if (PORTDbits.RD2 == 1)
                {               
                    break; 
                }                  
                // reset Receive monitor
                LATDbits.LATD6 = 0;
                programState =StateCHECK_NEXT;
                
            case StateCHECK_NEXT  :
                // wait for the RAISING FRONT on the Clock bit of the Clock PORT
                // remain on this state while condition True
                if (PORTDbits.RD2 == 0)
                {               
                    break; 
                }  
                programState = StateRECEIVE;
                break;
                
            case StateCHECK_DATA  :
                // reset Receive monitor
                LATDbits.LATD6 = 0;
                // set CHECK_DATA monitor
                LATDbits.LATD8 = 1;
                // stop timeout monitor
                WaitTime = -1;
                
                // if trasmission was successful notify with D3 LED
                if (buffer == 'a'){
                    SuccessCounter = 0;// start counter
                    SetLedON();
                    programState = StateSUCCESS;
                    break;
                }
                setErrorBit();
                programState = StateWAIT_RESET;
                break;
                
            case StateSUCCESS  :  
                if (SuccessFlag==0)
                    break;
                
                SuccessFlag= 0;
                SetledOFF();
                programState = StateWAIT_RESET;
                
            case StateWAIT_RESET  :
                 if (PORTDbits.RD2 == 1)
                {               
                    break; 
                }       
                programState = StateIDLE;           
        }
        
    }
    return 1;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// STEP 3. configure the Timer 1 interrupt handler

void __ISR(_TIMER_1_VECTOR, IPL2SOFT) Timer1Handler(void)
{
    // clear the interrupt flag
    mT1ClearIntFlag();

    // set the notify bit
    timer_event = 1;
}

