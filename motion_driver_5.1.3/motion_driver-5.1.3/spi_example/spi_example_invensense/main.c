//******************************************************************************
//   The task is to test the SPI driver by checking the who am i value in the
//   Invensense IC.
//
//   Description: SPI master talks to SPI slave using 3-wire mode.
//   ACLK = ~32.768kHz, MCLK = SMCLK = DCO ~ 1048kHz.  BRCLK = SMCLK/2
//
//
//                   MSP430F552x
//                 -----------------
//             /|\|                 |
//              | |                 |
//              --|RST          P1.2|-> LED
//                |                 |
//                |             P4.1|-> Data Out (UCB1SIMO)
//                |                 |
//                |             P4.2|<- Data In (UCB1SOMI)
//                |                 |
//  		      |             P4.3|-> Serial Clock Out (UCB1CLK)
//                |                 |
//  		      |             P4.0|-> Chip select, (Active low)
//
//
//   Karthik K, built using code composer studio.
//******************************************************************************

#include <msp430f5528.h>
#include "msp430_spi.h"

unsigned char rx_buffer[10];
unsigned int j;
void main(void)
{
  volatile unsigned int i;

  WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer

  P1DIR |=BIT2;
  P1OUT |=BIT2;

  msp430_spi_enable();

  for(i=50;i>0;i--);                        // Wait for slave to initialize

  __bis_SR_register(GIE);                   //  enable interrupts

  /* check if the chip is in sleep */
  msp430_spi_read(0x00,0x6b,1,rx_buffer);
  if(rx_buffer[0]!=0x01){
	  rx_buffer[0]=1;
	  P1OUT |=BIT3;
  }

  /* Make sure you wake up the chip */
  msp430_spi_write(0x00,0x6b,1,rx_buffer);

  /* Delay for start up of the chip, at least 100ms*/
  for(j=0;j<0xffe;j++);

  msp430_spi_read(0x00,0x6b,1,rx_buffer);
  if(rx_buffer[0]==1){
	  rx_buffer[0]=1;
	  P1OUT &=~BIT3;
  }

  while(1){
	  msp430_spi_read(0x00,0x75,1,rx_buffer);
	  if(rx_buffer[0] == 0x68){
		  P1OUT ^= BIT2; // Check if the reading is correct
	  }
	  for(j=0;j<0xffe;j++);
	}
}

