#ifndef I2C_CPP
#define I2C_CPP

class I2C{
	I2C(uint8_t addr)
	{
		TWI0.SADDR = addr|0x01;				//Slave address & enable general call
		TWI0.SCTRLA = TWI_ENABLE_bm |	    //Enable slave peripheral
		TWI_APIEN_bm |						//Enable address match interrupt
		TWI_PIEN_bm |						//Enable stop interrupt
		TWI_DIEN_bm |						//Enable data interrupt
		TWI_SMEN_bm;						//Enable smart mode
	}

	
	
	
};

ISR(TWI0_TWIS_vect)
{
	if(TWI0.SSTATUS & TWI_APIF_bm)					//Address match/stop interrupt
	{
		if (TWI0.SSTATUS & TWI_COLL_bm)
		{
			TWI0.SSTATUS |= TWI_COLL_bm;			//Clear Collision flag
			TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction
			return;
		}
		if(TWI0.SSTATUS & TWI_AP_bm)
		TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;		//Send ACK after address match
		else
		TWI0_SCTRLB = TWI_SCMD_COMPTRANS_gc;	//complete transaction after Stop
	}
	
	if(TWI0.SSTATUS & TWI_DIF_bm)					//Data interrupt
	{
		if(TWI0.SSTATUS & TWI_DIR_bm)
		{
			TWI0.SDATA = ++nnn;						//Transmit data for Master to read
			TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
		}
		else
		{
			TWI0_SCTRLB = TWI_SCMD_RESPONSE_gc;
			nnn = TWI0.SDATA;						//Receive data written by Master
		}
	}
}
#endif
