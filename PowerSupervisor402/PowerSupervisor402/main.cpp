/*
* PowerSupervisor.cpp
*
* Created: 22.07.2020 14:30:03
* Author : donel
*
* Changelog 25.07: Added Mode: Send Update on Every interval
*			30.07: Pin Supervise Power & Sleep -> Pullup & Inverted (Reads low when high, vv.) Use Diodes
*			31.07: Increase Sleep time (30s), added off hyst, Hardware_ added capcitor for voltage drop
*			25.10: Minimize project
*/



/************************************************************************/
/*  Todo & check
Pulldown Resistors?
Diode on PA1 REset
Pegel?

Optimierung:
- Brown out ?
- Lower Frequenz
- ADC ausschaltem (AREF etc)
- Kein pullup! -> OK
-
- Transistor statt Reset PIN ESP32
+
Problem: Nur 2 & 6 haben full Async Interrupt detection                  */
/************************************************************************/
/**
Pin Layout:
PA6 & PA7 UART (PA6 = TXD, PA7 = RXD)
PA0	UPDI Pin
PA3	Interrupt Power
PA2 Interrupt Awaik
PA1	Reset/Power hold

-----
VCC						-	|* G|	-	GND
PA6 = TXD				-	|A	|	-	PA3	Interrupt Awaik
PA7 = RXD				-	|	|	-	PA0	UPDI Pin
PA1	Reset/Power hold	-	|  A|	-	PA2 Interrupt Power
-----
A = async interrupts
**/


/*
Modes:
Power pin toggle: say awaik?
0b00001X00 -> awaik -> await Commands  (Perhaps difficult, not asyncr. -> no wakeup from deep sleep? or trigger by RXD?)
0b00000100 -> sleeping -> sleep, Ther is power
0b00000000 -> power off -> start reactivation protocol

*/

//#define F_CPU 3333333 //prescale 6 CLK 20 MHz = default  -> Baud 57600  (ca 1.1 mA full load)
#define F_CPU 2000000// prescale 10, CLK = 20 Mhz -> Baud 57600			(ca 865 uA full load)
//#define F_CPU 1250000// prescale 16, CLK = 20 Mhz    -> Baud 57600			(ca 700 uA full load)
//#define F_CPU 625000// prescale 32, CLK = 20 Mhz		-> ! Baud 57600			(ca 565 uA  full load)


#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

//#define MY_DEBUG


#include "VccReader.cpp"
#include "msg_structs.c"

#define BOARD_ON PORTA.OUT = 0b01110000 ; //PORTA.OUT |= PIN1_bm;
#define BOARD_OFF PORTA.OUT = 0b00000110;

#define EXEC_TIME 5000

volatile char mode = 0b00000000;
char buffer[50];			//mode	on   Off  //sleep duration s
str_config config ={0x02,0b00000000,3500,3000,30};



void rtcDisable(){
	RTC.CTRLA = 0;
}


void rtcEnable(uint16_t sleep_duration){
	while (RTC.STATUS > 0) {} // Wait for all register to be synchronized

	RTC.PER = 1024*sleep_duration; //in seconds
	RTC.INTCTRL = 0 << RTC_CMP_bp
	| 1 << RTC_OVF_bp; //Overflow interrupt.
	
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc	//NO Prescaler
	| 1 << RTC_RTCEN_bp       	//Enable RTC
	| 1 << RTC_RUNSTDBY_bp;   	//Run in standby

	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; // 32KHz divided by 32, i.e run at 1.024kHz
}

void setup(){
	#if F_CPU == 2000000
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); 	/* Set the Main clock to internal 20MHz oscillator*/
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_10X_gc | CLKCTRL_PEN_bm);  	/* Set the Main clock division factor to 6X and keep the Main clock prescaler enabled. */
	#elif F_CPU == 1250000
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); 	/* Set the Main clock to internal 20MHz oscillator*/
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_16X_gc | CLKCTRL_PEN_bm);  	/* Set the Main clock division factor to 6X and keep the Main clock prescaler enabled. */
	#elif F_CPU == 625000
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); 	/* Set the Main clock to internal 20MHz oscillator*/
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_32X_gc | CLKCTRL_PEN_bm);  	/* Set the Main clock division factor to 6X and keep the Main clock prescaler enabled. */
	#endif
	

	PORTA.DIR |= PIN1_bm;	//OUTPUT PIN A1
	BOARD_OFF; // Start Value =off
	config.mode = 0;
	
	rtcEnable(config.deep_sleep_duration);
	
	initAdc();
	readVoltage(); //read once beacause of faulty readings
}


int main(void){
	setup();
	uint16_t volt = 0;
	#ifdef MY_DEBUG
	serial(57600);
	sendString("\n\nSTART\n");
	#endif
	while (1)
	{
		volt = readVoltage();
		
		#ifdef MY_DEBUG
		sprintf(buffer,"\nMode: %d Volt:%d\n", config.mode, volt);
		sendString(buffer);
		#endif


		if (config.mode == 0){
			if (volt >= config.hyst_on){
				config.mode = 1;
				BOARD_ON
				_delay_ms(EXEC_TIME);  //5s
				BOARD_OFF
			}
			else {
				BOARD_OFF
			}
		}
		else{
			if (volt <= config.hyst_off){
				config.mode = 0;
				BOARD_OFF
			}
			else {
				BOARD_ON
				_delay_ms(EXEC_TIME);  //5s
				BOARD_OFF
			}
		}
		
		#ifdef MY_DEBUG
		sprintf(buffer,"\nMode: %d Volt:%d\n", config.mode, volt);
		sendString(buffer);
		#endif

	}
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);//SLEEP_MODE_PWR_DOWN); SLEEP_MODE_STANDBY//Setting Sleep Mode (Standby: RTC)
	sleep_mode(); //GoTo Deep sleep
}



