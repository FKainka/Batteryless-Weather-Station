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
#ifdef MY_DEBUG
#include "Serial.cpp"
#endif
#include "msg_structs.c"

#define BOARD_SET_DIR PORTA.DIR = 255; //PIN1_bm;	//OUTPUT PIN A1
#define BOARD_ON PORTA.OUT = 255;//0b0100000 ; //PORTA.OUT |= PIN1_bm;
#define BOARD_OFF PORTA.OUT = 0 ; //0b00100000;

#define EXEC_TIME 5000
#define SLEEP_TIME 1800
#define SLEEP_PRESCALE 60
#define SLEEP_WAKEUPS 30
#define VOLTAGE_ON 3500

#ifdef MY_DEBUG
char buffer[50];			//mode	on   Off  //sleep duration s
#endif


ISR(RTC_CNT_vect)
{
	RTC.INTFLAGS = RTC_OVF_bm;        // Wake up from STANDBY. Just clear the flag (required) - the RTC overflow Event will handle the pulse
}


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


	
	rtcEnable(SLEEP_PRESCALE);
	set_sleep_mode(SLEEP_MODE_STANDBY); // set power saving mode as STANDBY, not POWER DOWN
	sleep_enable();                     // enable sleep mode
	sei();                              // turn on interrupts
	
	initAdc();
	readVoltage(); //read once because of faulty readings
	
	
}


int main(void){
	setup();
	uint16_t volt = 0;
	uint8_t sleep_circles = 0;
	#ifdef MY_DEBUG
	serial(57600);
	sendString("\n\nSTART\n");
	#endif
	
	//FOR some reason this musst be set her!
	BOARD_SET_DIR
	BOARD_OFF   // Start Value =off
	
	while (1)
	{
		if (sleep_circles == 0){
			sleep_circles = SLEEP_WAKEUPS;

			volt = readVoltage();
			
			#ifdef MY_DEBUG
			sprintf(buffer,"V1:%d\n", volt);
			sendString(buffer);
			#endif
			
			if (volt > VOLTAGE_ON){
				
				BOARD_ON
				
				#ifdef MY_DEBUG
				sprintf(buffer,"BOARD ON\n");
				sendString(buffer);
				#endif
				
				_delay_ms(EXEC_TIME);  //5s
				BOARD_OFF
				
				#ifdef MY_DEBUG
				sprintf(buffer,"BOARD OFF\n");
				sendString(buffer);
				#endif
				
			}
			
			#ifdef MY_DEBUG
			sprintf(buffer,"V2:%d\n", volt);
			sendString(buffer);
			_delay_ms(200);
			#endif
		}
		sleep_circles --;
		sleep_cpu();
	}

}




