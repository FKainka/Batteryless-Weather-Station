
#ifndef VCC_READER_CPP
#define VCC_READER_CPP

#include <avr/io.h>

/************************************************************************/
/*			Return VCC as mV                                            */
/************************************************************************/
uint16_t readVoltage(void)
{

	float Vcc_value = 0                   /* measured Vcc value */;
	ADC0.CTRLA = 1 << ADC_ENABLE_bp       /* ADC Enable: enabled */
	| 1 << ADC_FREERUN_bp                 /* ADC Free run mode: enabled */
	| ADC_RESSEL_10BIT_gc                 /* 10-bit mode */;
	ADC0.COMMAND |= 1;                    // start running ADC
	
	while(1) {
		if (ADC0.INTFLAGS)                // if an ADC result is ready
		{
			Vcc_value = ( 0x400 * 1.1 ) / ADC0.RES /* calculate the Vcc value */;
			ADC0.INTFLAGS = ADC_RESRDY_bm; // Nessecarry?
			//Turn off reF?
			return Vcc_value*1000;
		}
	}
}

void initAdc(){
	
	/* The App Note AN2447 uses Atmel Start to configure Vref but we'll do it explicitly in our code*/
	VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;  /* Set the Vref to 1.1V*/

	/* The following section is directly taken from Microchip App Note AN2447 page 13*/
	ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc    /* ADC internal reference, the Vbg*/;
	
	ADC0.CTRLC = ADC_PRESC_DIV4_gc        /* CLK_PER divided by 4 */
	| ADC_REFSEL_VDDREF_gc                /* Vdd (Vcc) be ADC reference */
	| 0 << ADC_SAMPCAP_bp                 /* Sample Capacitance Selection: disabled */;
	
	
}

#endif