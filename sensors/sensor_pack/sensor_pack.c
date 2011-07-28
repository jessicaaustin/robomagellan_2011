/*
 * package of sensors
 */

#define F_CPU 8000000UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define BUFFER_SIZE 100

#include <inttypes.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <util/delay.h>

int baseRotation;

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

unsigned short
readSpiGyroscope(void)
{
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

	spiValue = MSB;
	spiValue = spiValue << 8;
	spiValue += LSB;
	spiValue = spiValue >> 2;

	return spiValue;
}

void
setupSpiGyroscope(void)
{
	int	first, second, third;
	printLine("0|gyro setup");
	baseRotation = 0;

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

	(void) readSpiGyroscope();
	first = readSpiGyroscope();
	second = readSpiGyroscope();
	third = readSpiGyroscope();
	baseRotation = (int) ((first + second + third) / 3);
	
	return;
}

void
readRangefinder(void)
{
	char	buffer[20];
	short	temp_PIND;
	int		ticks;
	float	distance;

	/*
	 * stop the clock and then set TCNT1 to 1 microsecond per tick
	 */
	TCCR1B &= 0xF8;
	TCCR1B |= _BV(CS11);

	DDRD |= _BV(DDD6);
	PORTD &= ~(_BV(PORTD6)); /* set pin value low */

	/*
	 * set the pin high, wait 5 microseconds, then set it low
	 */
	PORTD |= _BV(PORTD6);
	TCNT1 = (unsigned int) 0;
	while (TCNT1 < 5);
	PORTD &= ~(_BV(PORTD6)); /* set pin value low */

	/*
	 * disable all pull-ups, set D6 as input, enable tri-state
	 */
	SFIOR |= _BV(PUD);
	DDRD &= ~(_BV(DDD6));
	PORTD |= _BV(PORTD6);

	/*
	 * stop the clock and then set TCNT1 to 8 microseconds per tick
	 */
	TCCR1B &= 0xF8;
	TCCR1B |= (_BV(CS11) | _BV(CS10));

	/*
	 * pause for 752 microseconds
	 */
	TCNT1 = (unsigned int) 0;
	while (TCNT1 < 94);
	
	while ((PIND & _BV(PIND6)) == 0);
	
	/*
	 * assume PIND6 is now high, time how long it stays high
	 */
	TCNT1 = (unsigned int) 0;
	for (;;) {
		temp_PIND = PIND;
		if (temp_PIND & _BV(PIND6))
			continue;
		else
			ticks = TCNT1;
			break;
	}
	/* convert to microseconds, determine one-way time and convert to cm */
	distance = (float) ticks * 0.1372;

	sprintf(buffer, "Rg|%d", (int) distance);
	printLine(buffer);

	/*
	 * stop the clock and then set TCNT1 to 128 microseconds per tick
	 */
	TCCR1B &= 0xF8;
	TCCR1B |= (_BV(CS12) | _BV(CS10));

	TCNT1 = (unsigned int) 0;
	/*
	while (TCNT1 < 65534);
	 */

	return;
}

int
main (void)
{
	char buffer[20];
	unsigned short currentRotation;

	setupUsart();
	printLine("Se|2");

	setupSpiGyroscope();

	for (;;) {
		currentRotation = readSpiGyroscope();
		if (currentRotation == baseRotation) {
			sprintf(buffer, "Gy|0|N");
		} else {
			if (currentRotation < baseRotation) {
				sprintf(buffer, "Gy|%u|R", baseRotation - currentRotation);
			} else {
				sprintf(buffer, "Gy|%u|L", currentRotation - baseRotation);
			}
		};
		printLine(buffer);
		_delay_ms(100.0);

		readRangefinder();
		_delay_ms(100.0);
	}

	printLine("Se|Off");

    return (0);
}
