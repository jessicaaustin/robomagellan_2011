/*
 * spi_gyroscope
 *
 * setup the AVR as a master and for a single slave
 * and then read from that slave
 *
 * https://github.com/bobbens/LACE/blob/master/motherboard/spim_hw.c
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/delay.h>

#define F_CPU 8000000UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define BUFFER_SIZE 100

void
setupUsart() {
   UCSRB |= (1 << RXEN) | (1 << TXEN);   // Turn on the transmission and reception circuitry
   UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes

   UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
   UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register

   return;
}

void
printLine(char buffer[]) {
	int bufferIndex = 0;

	while (buffer[bufferIndex] != '\0') {
	  /*
	   * Do nothing until UDR is ready for more data to be written to it
	   */
      while ((UCSRA & _BV(UDRE)) == 0) {};

	  /*
	   * write the received byte back out the USART
	   */
      UDR = buffer[bufferIndex];
	  bufferIndex++;
	}

	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = '\r';
	while ((UCSRA & (1 << UDRE)) == 0) {};
	UDR = '\n';

	return;
}

void
setupSpiGyroscope(void)
{
	printLine("gyro setup");
	/*
	 * setup MISO
	 * and enable SPI
	 */
	DDRB = _BV(DDB4) | _BV(DDB5) | _BV(DDB7);
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1);

	/* disable slave */
	PORTB |= _BV(PORTB4);

	/*
	 * initialize the gyro by sending 16 bits
	 */
	PORTB &= ~(_BV(PORTB4));
	SPDR = 0u;
	SPDR = 0u;
	PORTB |= _BV(PORTB4);

	return;
}

void
readSpiGyroscope(void)
{
	char buffer[20];
	unsigned char MSB, LSB;
	unsigned short spiValue;

	/*
	 * expect MSB first, comprised of three leading zero bits,
	 * then ten data bits and then two trailing zero bits. decreasing
	 * values indicate clockwise rotation.
	 */

	PORTB &= ~(_BV(PORTB4));
	SPDR = (unsigned char) 0;
	while (! (SPSR & _BV(SPIF)))
		;
	MSB = SPDR & 0x7F;
	SPDR = (unsigned char) 0;
	while (! (SPSR & _BV(SPIF)))
		;
	LSB = (unsigned char) SPDR;
	PORTB |= _BV(PORTB4);

	sprintf(buffer, "M %x L %x", MSB, LSB);
	printLine(buffer);

	spiValue = MSB;
	sprintf(buffer, "s1 %u", spiValue);
	printLine(buffer);
	spiValue = spiValue << 8;
	sprintf(buffer, "s2 %u", spiValue);
	printLine(buffer);
	spiValue += LSB;
	sprintf(buffer, "s3 %u", spiValue);
	printLine(buffer);
	spiValue = spiValue >> 2;

	sprintf(buffer, "spiValue <%u>", spiValue);
	printLine(buffer);

	return;
}

int
main (void)
{
	setupUsart();
	printLine("Gyro 24");

	setupSpiGyroscope();

	for (;;) {
		readSpiGyroscope();
		_delay_ms(10000.0);
	}

	while (1);

	printLine("Done");

    return (0);
}
