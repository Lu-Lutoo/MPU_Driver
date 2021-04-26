/*
 * msp430_spi.h
 *
 *  Created on: Feb 6, 2012
 *      Author: kkatingari
 */

#ifndef MSP430_SPI_H_
#define MSP430_SPI_H_

extern short spiEnabled;

int msp430_spi_enable(void);
int msp430_spi_disable(void);

int msp430_spi_write(unsigned char sel, unsigned char reg_addr, unsigned char length,
		unsigned char const *data);

int msp430_spi_send(unsigned char data);

int msp430_spi_read(unsigned char sel, unsigned char reg_addr, unsigned char length,
		unsigned char *data);

void set_spi_rx_interrupt_flag(unsigned char value);
unsigned char get_spi_rx_interrupt_flag(void);

void cs_low();
void cs_high();

#endif /* MSP430_SPI_H_ */
