/* 
 * File:   main.c
 * Author: delli
 * Proj_name: CustomSerialCommunicationExample_SendData
 * Created on May 8, 2019, 3:49 PM
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

#include <p32xxxx.h>
#include <plib.h> // Include the PIC32 Peripheral Library.

//#define SYSCLK 80000000L // Give the system's clock frequency
#define StatePOR 0x00
#define StateIDLE 0x10
#define StateSEND 0x20
#define OUTPUT LATDbits.LATD5 //RD3
#define CLOCK LATDbits.LATD6  //RD4

void delay(int t) { // 1 ms di delay
   int n = t * 1900; //1900 è un numero ricavato sperimentalmente
   while (n > 0) {
      n--;
   }
}  

char CheckButton();
/*
 * 
 */

int oldButtonState;
int newButtonState;
char state = StatePOR;
char charToSend;
int i;

int main(void) {
    /*configure machine*/
    TRISD = (1<<4) | (1<<0); // uso il pulsante BUT2
    //SYSTEMConfigPerformance(SYSCLK);
    LATD = 0;
    OUTPUT = 0;
    int buttonPressed;
    
    while(1)
    {
        buttonPressed = CheckButton();
        switch (state)
        {
            case StatePOR:
                state = StateIDLE;
                break;
            case StateIDLE:
                if (buttonPressed)
                {
                    state = StateSEND;
                }
                break;
            case StateSEND:
                charToSend = 'a';
                for (i = 0; i < 8; i++)
                {
                    OUTPUT = ( unsigned int )( ( charToSend & (1 << i) ) ? 1 : 0);
                    CLOCK = 0;
                    delay(500);
                    CLOCK = 1;
                    delay(500);
                }
                /*CLOCK = 0;
                //OUTPUT =~ OUTPUT;
                //OUTPUT = 0;
                delay(1000);
                CLOCK = 1;
                //OUTPUT = 1;
                OUTPUT =~ OUTPUT;
                delay(1000);*/
                CLOCK = 0;
                state = StateIDLE;
                break;
            default:
                break;
        }
    }
    return 1;
}

char CheckButton()
{
    int temp = 0;
    newButtonState = !PORTDbits.RD4;
    if (oldButtonState > newButtonState)
    {
        temp = 1;
    }
    oldButtonState = !PORTDbits.RD4;
    return temp;
    /***/
}

