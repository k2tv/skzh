/******************************************************************************
    Filename: clock.c

    This file defines clock related functions for the CC253x family
    of RF system-on-chips from Texas Instruments.

******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include "clock.h"

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

/******************************************************************************
* @fn  clockSetMainSrc
*
* @brief  Function for setting the main system clock source.
*         The function turns off the clock source that is not being used.
*         TICKSPD is set to the same frequency as the source.
*
* @param  unsigned char source (one of CLOCK_SRC_HFRC or CLOCK_SRC_XOSC)
*
* @return void
*
******************************************************************************/
void clockSetMainSrc(unsigned char source)
{
    register unsigned char osc32k_bm = CLKCONCMD & CLKCON_OSC32K_BM;

    // Source can have the following values:
    // CLOCK_SRC_XOSC   0x00  High speed Crystal Oscillator (XOSC)
    // CLOCK_SRC_HFRC   0x01  Low power RC Oscillator (HFRC)
    
    SLEEPCMD &= ~SLEEP_OSC_PD_BM;       // power up both oscillators
    while (!CC2530_IS_HFRC_STABLE() || ((SLEEPSTA & SLEEP_OSC_PD_BM)!=0));// wait until the oscillator is stable
    asm("NOP");

    if (source == CLOCK_SRC_HFRC)
	{
        CLKCONCMD = (osc32k_bm | CLKCON_OSC_BM | TICKSPD_DIV_2 | CLKCON_CLKSPD_BM);
    }
    else if (source == CLOCK_SRC_XOSC)
	{
        CLKCONCMD = (osc32k_bm | TICKSPD_DIV_1);
    }
    CC2530_WAIT_CLK_UPDATE();
    SLEEPCMD |= SLEEP_OSC_PD_BM;        // power down the unused oscillator
}

/******************************************************************************
* @fn  clockSelect32k
*
* @brief  Function for selecting source for the 32kHz oscillator
*
* @param  unsigned char source (one of CLOCK_32K_XTAL or CLOCK_32K_RCOSC)
*
* @return unsigned char - SUCCESS or FAILED
*
******************************************************************************/
unsigned char clockSelect32k(unsigned char source)
{
    // System clock source must be high frequency RC oscillator before
    // changing 32K source. 
    if( !(CLKCONSTA & CLKCON_OSC_BM) )
      	return 1;
    
    if (source == CLOCK_32K_XTAL)
	{
        CLKCONCMD &= ~CLKCON_OSC32K_BM;
    }
    else if (source == CLOCK_32K_RCOSC)
	{
        CLKCONCMD |= CLKCON_OSC32K_BM;
    }
    CC2530_WAIT_CLK_UPDATE();
    
    return 0;
}

/*------------------------------------------------------------------------------
										  0ooo                                     
								ooo0	 (	 )                                     
								(	)	  ) /                                      
								 \ (	 (_/                                       
								  \_)		 Modify By:cuiqingwei [gary]                  
------------------------------------------------------------------------------*/
