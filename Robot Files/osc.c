/*
file osc.c
*/

//***********************************************************************************
//
//                          set_osc_32MHz()
//
//    sets the oscillator from the default 1 MHz to 32 MHz
//
//    Note TOSC = 1/(32 MHz) = 31.25 ns and TCY = 4*TOSC = 125 ns
//
//***********************************************************************************

#include <xc.h>
#include "osc.h"

void set_osc_32MHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 1;     // Set the OSCILLATOR Control Register to 8 MHz
  OSCCONbits.IRCF1 = 1;      
  OSCCONbits.IRCF0 = 1;     
 
  OSCTUNEbits.PLLEN = 1;    // Enable PLL, boost by 4 -> 32 MHz

  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}

void set_osc_8MHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 1;     // Set the OSCILLATOR Control Register to 8 MHz
  OSCCONbits.IRCF1 = 1;      
  OSCCONbits.IRCF0 = 1;     

  OSCTUNEbits.PLLEN = 0;    // disable PLL
  
  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}

void set_osc_1MHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 1;     // Set the OSCILLATOR Control Register to 1 MHz
  OSCCONbits.IRCF1 = 0;      
  OSCCONbits.IRCF0 = 0;     

  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}

void set_osc_500kHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 0;     // Set the OSCILLATOR Control Register to 500 kHz
  OSCCONbits.IRCF1 = 1;      
  OSCCONbits.IRCF0 = 1;     

  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}

void set_osc_4MHz(void)
{
  int i;
 
  OSCCONbits.IRCF2 = 1;     // Set the OSCILLATOR Control Register to 500 kHz
  OSCCONbits.IRCF1 = 1;      
  OSCCONbits.IRCF0 = 0;     

  OSCTUNEbits.PLLEN = 0;    // disable PLL
  
  for(i=0;i<500;i++);       // delay to allow clock PLL to lock (stabilize)

      
}
