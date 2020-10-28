/*
 * I2C.c
 *
 * Created: 16/04/2019 3:41:13 AM
 *  Author: goo17e
 */ 

#include <avr/io.h>


void TWI_Init(char baud)
{
	TWI0.MBAUD = baud;
	TWI0.MCTRLB |= TWI_FLUSH_bm;
	TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm);
}

void TWI_Enable()
{
	TWI0.MCTRLA = 1<<TWI_SMEN_bp | 1 << TWI_ENABLE_bp;
	TWI0_MSTATUS |= TWI_BUSSTATE_IDLE_gc;
}

char TWI_Start(char addr)
{
	if ((TWI0.MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_BUSY_gc)
	{
		TWI0.MCTRLB &= ~(1<<TWI_ACKACT_bp);
		TWI0.MADDR = addr;
		
		if (addr & 1)
		{
			while (!(TWI0_MSTATUS & TWI_RIF_bm));
		}
		else
		{
			while (!(TWI0_MSTATUS & TWI_WIF_bm));
		}
		return TWI0.MSTATUS;
	}
	else
		return TWI0.MSTATUS;
}

char TWI_Read(char * c, char ACKorNACK)
{
	if ((TWI0.MSTATUS & TWI_BUSSTATE_gm)==TWI_BUSSTATE_OWNER_gc)
	{
		while (!(TWI0.MSTATUS & TWI_RIF_bm));
		
		if (ACKorNACK)
		TWI0.MCTRLB &= ~(1<<TWI_ACKACT_bp);	//Send ACK
		else
		TWI0.MCTRLB |= 1<<TWI_ACKACT_bp;	//Send NACK
		
		*c = TWI0.MDATA;
		
		return TWI0.MSTATUS;
	}
	else
		return TWI0.MSTATUS;
}

char TWI_Write(char data)
{
	if ((TWI0.MSTATUS&TWI_BUSSTATE_gm) == TWI_BUSSTATE_OWNER_gc)
	{
		while (!((TWI0.MSTATUS & TWI_WIF_bm) | (TWI0_MSTATUS & TWI_RXACK_bm)));
		
		TWI0.MDATA = data;
		return TWI0.MSTATUS;
	}
	else
		return TWI0.MSTATUS;
}

void TWI_Stop()
{
	TWI0.MCTRLB |= TWI_MCMD_STOP_gc;
}