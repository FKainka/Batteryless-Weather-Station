#ifndef CONFIG_CPP
#define CONFIG_CPP

#include <Arduino.h>

#define CMD_MSG_LEN 10

#define CMD_SEND_VOLTAGE 0x01
#define CMD_SET_CONFIG 0x02
#define CMD_GET_CONFIG 0x03

/*
Additional possible commands:
- Turn off Main Board
- Turn off Main Board with  longer sleep (hybernate)
- Temp Hyst. Test


Modes:
- while sleep sleep
- periodic
- Chose Frequency etc


0bxxxxxxx1 = High Low Hyst
0bxxxxxx1x = Every weakup:  meassuren + send status / mode


*/



//MSG max len: 12, MSG end: 0xFF
/*
typedef struct{
uint8_t cmd;
uint8_t values[11];
}str_msg;
*/
/************************************************************************/
/* cmd, hyst_low, hyst_high, sleep_time,      
/*
/* Rearanged Structure due to padding erros   ?                       */
/************************************************************************/

typedef struct  __attribute__((packed)){
	uint8_t cmd;
	uint8_t mode;
	uint16_t hyst_on;
	uint16_t hyst_off;
	uint16_t deep_sleep_duration;  //in seconds
}str_config;

typedef struct __attribute__((packed)){
	uint8_t cmd;
	uint16_t voltage;
	char str[7];
}  str_voltage;

typedef struct  __attribute__((packed)){
	uint8_t cmd;
	uint16_t voltage;
	uint8_t mode;
	char str[6];
}str_status;
#endif