#ifndef SERIAL_CPP
#define SERIAL_CPP

#ifndef F_CPU
#define F_CPU 3333333 //prescale 6
#endif

#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <string.h>
#include <stdio.h>


volatile bool stopSerialRecieve = false;

void serial(uint16_t baudrate)
{
	PORTA.DIRSET = 0x40;	//TXD an PA6
	PORTA.DIRCLR = 0x80;	//RXD an PA7
	USART0.BAUD = (uint16_t)USART0_BAUD_RATE(baudrate);
	
	USART0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;	//Send and RECIEVE
	
}

void sendChar(char c)
{
	while (!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	USART0.TXDATAL = c;
}

void sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		sendChar(str[i]);
	}
}

void sendBuffer(char *str, size_t len)
{
	for(size_t i = 0; i < len; i++)
	{
		sendChar(str[i]);
	}
}

/* Zeichen empfangen - Blocking */
char receiveChar()
{
	while (!(USART0.STATUS & USART_RXCIF_bm) )
	{
		if (stopSerialRecieve){
			 return 0xFF;
		}
	}
	return USART0.RXDATAL;
}

void receiveString( char* buffer, uint8_t maxLen )
{
	uint8_t nextChar;
	uint8_t stringLen = 0;

	nextChar = receiveChar();         // Warte auf und empfange das nächste Zeichen

	// Sammle solange Zeichen, bis:
	// * entweder das String Ende Zeichen kam
	// * oder das aufnehmende Array voll ist
	while( nextChar != '\n' && nextChar != 0xFF && stringLen < maxLen - 1 ) {
		*buffer++ = nextChar;
		stringLen++;
		nextChar = receiveChar();
	}

	// Noch ein '\0' anhängen um einen Standard
	// C-String daraus zu machen
	*buffer = '\0';
}


#endif