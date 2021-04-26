/*
 $License:
 Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_spi.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430-SL
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, uMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_spi.c
 *      @brief      Serial communication functions needed by uMPL to
 *                  communicate to the MPU devices.
 *      @details    This driver assumes that uMPL is running on MPS4305528
 *                  with a sub master clock set to higher frequency ~
 *                  12MHz. For other MSP430 chips, the frequency may be
 *                  lower, USCI B1 may not be available, or SDA and SCL may be
 *                  mapped to pins other than those set in @e
 *                  msp430_enable_spi.
 *
 * */

#include "msp430.h"
#include "msp430_spi.h"

#define msp430_get_smclk_freq() 1000000

short spiEnabled = 0;
static unsigned char spi_rx_ready_flag=0;

/*
 * @brief  This function enables the peripheral and
 * 		   associated functionality for the polarity,
 * 		   phase, direction and configuration
 *
 * 		   For INV chip the SPI is definition says
 * 		   Latched at rising edge, transition at falling
 * 		   edge. Also MSB first and INV chip is slave mode
 * 		   => Clock polarity should be in-active high
 * 		   => Clock phase, should use TI default
 * 		   Also the specification says, 1MHz and lower for all
 * 		   register setting and only sensor register and interrupt
 * 		   register can use faster data rate upto 20MHz
 *         Here in the example choose 100KHz.
 */
int msp430_spi_enable(void) {

	unsigned long smclk = 0;
	unsigned int br = 0;

	if (spiEnabled)
		return 0;

	P4SEL |= (BIT1 + BIT2 + BIT3); // Peripheral function instead of I/O

	UCB1CTL1 |= UCSWRST; // **Put state machine in reset**
	UCB1CTL0 |= UCMST + UCSYNC + UCCKPL + UCMSB; // 3-pin, 8-bit SPI master, msb first
	UCB1CTL1 |= UCSSEL_2; // SMCLK = 12MHz speed., cannot be greater than 1Mhz. Reduce it.

	/*msp430_get_smclk_freq(&smclk);*/ // if not used as a function.
	smclk= msp430_get_smclk_freq();
	br = smclk / 100000L; // 1MHz.

	UCB1BR0 = (unsigned char) (br & 0xFF);
	UCB1BR1 = (unsigned char) ((br >> 8) & 0xFF);

	UCB1CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
	UCB1IE |= UCRXIE; // Enable USCI_B1 RX  interrupt

	//Configuring CS PIN P4.0 for SPI
	P4SEL &= ~BIT0;
	P4DIR |= BIT0;
	P4OUT |= BIT0;

	spiEnabled = 1;

	return 0;

}

int msp430_spi_disable(void) {
	UCB1CTL1 |= UCSWRST; // **Put state machine in reset**
	UCB1IE &= ~UCRXIE; // Clear USCI_A0 RX interrupt
	P3SEL &= ~(BIT1 + BIT2 + BIT3); // I/O function instead of Peripheral
	spiEnabled = 0;
	return 0;
}

/*
 * @ brief, spi write function for Invensense IC using MSP430
 * @ parameters
 * @ sel = this is the handle, used to be
 *   just for compatibility with I2C and can be used ofr
 *   chip select if needed.
 * @ reg_addr, the register address of the chip
 * @ length, length of the data to be written, assuming
 *   auto increment of the register address by the chip
 *   parameter data. ;
 * @ data, data pointer where the data is written to reg addr
 *   and sequentially hence forth if greater than 1
 * @ return, success or failure
 */
int msp430_spi_write(unsigned char sel, unsigned char reg_addr,
		unsigned char length, unsigned char const *data) {

	//SPI transmit start
	cs_low();

	unsigned char index = 0;

	//Now write MPU register address
	// Set the flag to make sure Rx interrupt sets it.

	spi_rx_ready_flag=0;
	msp430_spi_send(reg_addr);
	while((spi_rx_ready_flag==0));
	//Now write data to TXBUF that will be written to MPU
	for (index = 0; index < length; index++) {
		spi_rx_ready_flag=0;
		msp430_spi_send(data[index]);
		while((spi_rx_ready_flag==0));
	}

	//SPI transmit stop
	cs_high();
	return 0;

}

/*
 * @ brief, spi read function for Invensense IC using MSP430
 * @ parameters
 * @ sel = this is the handle, used to be
 *   just for compatibility with I2C and can be used ofr
 *   chip select if needed.
 * @ reg_addr, the register address of the chip
 * @ length, length of the data to be read, assuming
 *   auto increment of the register address by the chip
 *   parameter data. ;
 * @ data, data pointer where the data is read from reg addr
 *   and sequentially hence forth if greater than 1
 * @ return, success or failure
 */
int msp430_spi_read(unsigned char sel, unsigned char reg_addr,
		unsigned char length, unsigned char *data) {

	unsigned char dummy_val = 0x00;
	unsigned char index = 0;

	if (!spiEnabled)
		return -1;
	if (!length)
		return 0;

	cs_low();

	//Now setup to read the MPU register address
	spi_rx_ready_flag=0;
	msp430_spi_send(reg_addr +0x80);
	while((spi_rx_ready_flag==0));
	//receive the rest of the bytes from the MPU.
	for (index = 0; index < length; index++) {
		spi_rx_ready_flag=0;
		msp430_spi_send(dummy_val);
		while ((spi_rx_ready_flag==0));
		data[index] = UCB1RXBUF; // Read the value
	}
	cs_high();
	return 0;
}

int msp430_spi_send(unsigned char data) {
	UCB1TXBUF = data;
	return 0;
}

void cs_low(void) {
	P4OUT &= ~BIT0;
}

void cs_high(void) {
	P4OUT |= BIT0;
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void)
{
	volatile unsigned int i;

	switch (__even_in_range(UCB1IV, 4)) {
	case 0:
		break; // Vector 0 - no interrupt
	case 2: // Vector 2 - RXIFG
		while (!(UCB1IFG & UCTXIFG)); // USCI_B1 TX buffer ready?
		spi_rx_ready_flag = 1; // Interrupt detected.
		for (i = 0; i < 20; i++);
		break;
	case 4:
		break; // Vector 4 - TXIFG
	default:
		break;
	}

}
