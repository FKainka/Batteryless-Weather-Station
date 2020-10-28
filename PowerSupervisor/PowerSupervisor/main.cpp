/*
* PowerSupervisor.cpp
*
* Created: 22.07.2020 14:30:03
* Author : donel
*
* Changelog 25.07: Added Mode: Send Update on Every interval
*			30.07: Pin Supervise Power & Sleep -> Pullup & Inverted (Reads low when high, vv.) Use Diodes
*			31.07: Increase Sleep time (30s), added off hyst, Hardware_ added capcitor for voltage drop
*
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

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define MY_DEBUG

//#define F_CPU 3333333 //prescale 6 CLK 20 MHz = default  -> Baud 57600  (ca 1.1 mA full load)
#define F_CPU 2000000// prescale 10, CLK = 20 Mhz -> Baud 57600			(ca 865 uA full load)
//#define F_CPU 1250000// prescale 16, CLK = 20 Mhz    -> Baud 57600			(ca 700 uA full load)
//#define F_CPU 625000// prescale 32, CLK = 20 Mhz		-> ! Baud 57600			(ca 565 uA  full load)

#include "VccReader.cpp"
#include "Serial.cpp"
#include "msg_structs.c"

#define BOARD_ON PORTA.OUT &= PORTA.OUT |= PIN1_bm;
#define BOARD_OFF PORTA.OUT &= ~PIN1_bm;

#define PINA2MODE  0b00001100

#define MODE_AWAIK  0b00001000
#define MODE_SLEEP  0b00000100 //Power ON
#define MODE_OFF  0b00000000

volatile char mode = 0b00000000;
char buffer[50];			//mode	on Off //sleep duration
str_config config ={0x02,0b00000001,4000,2500,30};
str_voltage voltage = {CMD_SEND_VOLTAGE,0,""};


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
	
	/* Pin Control */
	PORTA.DIR &= ~ PIN2_bm; //INPUT  PIN A2
	PORTA.DIR &= ~ PIN3_bm; //INPUT  PIN A3
	
	PORTA.PIN2CTRL = PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm ; 	//Set Interrupt -> both edges ((ToDo: Not Pullup, extern pulldown))
	PORTA.PIN3CTRL = PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm ; 	 // PORT_PULLUPEN_bm|	//Set Interrupt -> both edges
	
	PORTA.DIR |= PIN1_bm;	//OUTPUT PIN A1
	BOARD_OFF; // Start Value =off
	//PORTn.OUT = 0 = Low
	
	rtcEnable(config.deep_sleep_duration);
	
	initAdc();
	readVoltage(); //read once beacause of faulty readings
	
	
	
	serial(57600);
}

// PINA ISR
ISR(PORTA_PORT_vect)
{
	stopSerialRecieve = true;
	PORTA_INTFLAGS = 0xFF; //Clear all interrupt flags
}

//RTC ISR
ISR(RTC_CNT_vect)
{
	RTC.INTFLAGS = RTC_OVF_bm;
}


void handleCmd(){
	receiveString(buffer,CMD_MSG_LEN);
	switch (buffer[0]){
		case CMD_SEND_VOLTAGE:
		{
			voltage.voltage = readVoltage();
			sprintf(voltage.str, "%dmV",(int)voltage.voltage); //ToDo: Remove? debug only, may cost time
			memcpy(buffer, &voltage,sizeof(str_voltage));
	
			sendBuffer(buffer,sizeof(str_voltage));
			break;
		}
		case CMD_SET_CONFIG:{
			#ifdef MY_DEBUG
			sendString("CONFIG\n");
			#endif
			str_config *msg = (str_config*)buffer;
			memcpy(&config,msg,sizeof(str_config));
			
			//print back buffer
			memcpy(buffer, &config,sizeof(str_config));
			sendBuffer(buffer,sizeof(str_config));

			#ifdef MY_DEBUG
			
			sprintf(buffer,"H_H:%d, H_L:%d\n",(int)config.hyst_on,(int)config.hyst_off);		//ToDo Remove Debug
			sendString(buffer);
			#endif
			break;
		}
		case CMD_GET_CONFIG:{
			#ifdef MY_DEBUG
			sendString("GET CONFIG\n");
			#endif
			//str_config *msg = (str_config*)buffer;
			memcpy(buffer, &config,sizeof(str_config));
			sendBuffer(buffer,sizeof(str_config));
			#ifdef MY_DEBUG
			sprintf(buffer,"H_H:%d, H_L:%d\n",(int)config.hyst_on,(int)config.hyst_off); 	//ToDo Remove Debug
			sendString(buffer);
			#endif
			break;
		}
		default:{
			buffer[0] = 0xff;
			buffer[1] = '\0';
			sendString(buffer);
			#ifdef MY_DEBUG
			sendString("UNKNOWN\n");
			#endif
		}
	}
}

int main(void){
	setup();
	uint16_t volt = 0;
	#ifdef MY_DEBUG
	sendString("\n\nSTART\n");
	#endif
	while (1)
	{
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			_delay_ms(50); //wait till read and debounce
			mode =(~PORTA.IN & PINA2MODE); //determine mode; Invet -> low = AKTIVE
			volt = readVoltage();
			#ifdef MY_DEBUG
			sprintf(buffer,"\nMode: %d, PORTA %d Volt:%d\n", mode, PORTA.IN,volt);
			sendString(buffer);
			#endif
			
			stopSerialRecieve = false;
		}
		sei(); //enable Interrupts
		
		
		/*if (config.mode & 0b00000010){ //mode with always send status
			str_status* msg = (str_status*)buffer;
			msg->voltage = readVoltage();
			msg->mode = mode;
			sendBuffer(buffer,sizeof(str_voltage));
			break;
		}*/
		
		if(mode >= MODE_AWAIK ){
			#ifdef MY_DEBUG
			sendString("Mode: AWAIK\n");
			#endif
			//rtcDisable();
			handleCmd();
		}
		if (mode == MODE_SLEEP){
			#ifdef MY_DEBUG
			sendString("Mode: SLEEP\n");
			#endif
			
			if (config.mode & 0b00000001){ //mode with min Threshold
				if (volt<config.hyst_off){
					#ifdef MY_DEBUG
					sendString("DEAKTIVATE BOARD - LOW VOLTAGE\n");
					#endif
					BOARD_OFF;
				}
				//if (!(RTC.CTRLA & 0b00000001))rtcEnable(config.deep_sleep_duration);
				set_sleep_mode(SLEEP_MODE_STANDBY);//SLEEP_MODE_PWR_DOWN); //Setting Sleep Mode (Standby: RTC)
				sleep_mode(); //GoTo Deep sleep
			}
			else{
				#ifdef MY_DEBUG
				sendString("GOTO DEEP SLEEP\n");
				_delay_ms(10);
				#endif
				//rtcDisable();
				set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Setting Sleep Mode (deepest) -Y 3uA??? should be lower
				sleep_mode(); //GoTo Deep sleep
			}
		}
		if (mode == MODE_OFF){
			#ifdef MY_DEBUG
			sendString("Mode: OFF\n");	
			_delay_ms(10);
			#endif
			if (volt>=config.hyst_on){
				#ifdef MY_DEBUG
				sendString("AKTIVATE BOARD\n");
				_delay_ms(10);
				#endif
				BOARD_ON;
			}
			else if (volt<config.hyst_off){
				BOARD_OFF;
			}
			
			//if (!(RTC.CTRLA & 0b00000001))rtcEnable(config.deep_sleep_duration);
			set_sleep_mode(SLEEP_MODE_STANDBY);//SLEEP_MODE_PWR_DOWN); //Setting Sleep Mode (Standby: RTC)
			sleep_mode(); //GoTo Deep sleep
		}
	}
}



