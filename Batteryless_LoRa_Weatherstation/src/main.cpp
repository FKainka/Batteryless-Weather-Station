/**
 * Sleep & Power Pins -> LOW ! 
 * 
 */

#include <Arduino.h>
#include <sys/time.h>
#include <Preferences.cpp>
#include <heltec.h>

#include <rom/rtc.h>

#include <WiFi.h>
#include <driver/adc.h>
#include <esp_wifi.h>
#include <esp_bt.h>

#include "BMESensor.hpp"
#include "LoRaModule.hpp"

//Brownout
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//#define TESTMODUS
#ifdef TESTMODUS
#define TIME_TO_SLEEP 30 /* Time ESP32 will go to sleep (in seconds) 30 s */
#else
#define TIME_TO_SLEEP 1800 /* Time ESP32 will go to sleep (in seconds) 30 min */
#endif

#define LORA_MSG "{\"T\":%.2f,\"H\":%.2f%,\"P\":%.2f}}" //Error last sign?  ,\"V\":%.2f,\"D\":%s

#define PIN_POWER GPIO_NUM_13
#define PIN_DEEP_SLEEP GPIO_NUM_12
#define LORA_LED 25
#define VEXT 21
#define PIN_START_MESS 17

#define BAND 868E6 //you can set band here directly,e.g. 868E6,915E6
#define SF DR_SF12

// LoRaWAN NwkSKey, network session key
uint8_t NWKSKEY[16] = {0x70, 0x9A, 0x01, 0x56, 0x30, 0xD5, 0x72, 0x7E, 0xB5, 0x83, 0x64, 0xBD, 0xE4, 0x1A, 0x20, 0xC4};
// LoRaWAN AppSKey, application session key
uint8_t APPSKEY[16] = {0x0A, 0xDF, 0x1B, 0xAE, 0x95, 0xE3, 0x0A, 0xF2, 0x12, 0xDB, 0xB9, 0x65, 0x29, 0xBD, 0x1A, 0x01};
// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011BCA; // <-- Change this address for every node!

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */

char text[52]; //LoRa send buffer

BMESensor myBmeSensor;
LoRaModule myLoRaModule;

void adcWifiBtOff()
{
    adc_power_off();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();
    esp_bt_controller_disable();
}

void startDeepSleep()
{
    //gpio_hold_en(GPIO_NUM_21);
    LoRa.flush();
    LoRa.end();
    LoRa.sleep();
    //  Heltec.VextOFF(); //auto off due to electronic
    // gpio_hold_en(GPIO_NUM_21);

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    //esp_deep_sleep_disable_rom_logging(); // suppress boot messages
    esp_deep_sleep_start();
}

double readVoltage(byte pin) //deprecated
{
    double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    for (int i = 0; i < 5; i++)       //Mittelung mit minimiren von ausreißern (insg. 6 Werte)
        reading = (reading + analogRead(pin)) / 2;

    if (reading < 1 || reading > 4095)
        return 0;
    // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
    double value = -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
    return value * 2;
} // Added an improved polynomial, use either, comment out as required

void loop() {}

void setup()
{

    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    //Serial.begin(115200);

    pinMode(PIN_POWER, OUTPUT);
    digitalWrite(PIN_POWER, HIGH);
    gpio_hold_en(PIN_POWER);

    pinMode(PIN_DEEP_SLEEP, OUTPUT);
    digitalWrite(PIN_DEEP_SLEEP, HIGH);

    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    //Woke up from timer or from ULP
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER || wakeup_reason == ESP_SLEEP_WAKEUP_ULP)
    {
    }
    else //Reset By Programm Upload or Reset Button
    {
        adcWifiBtOff();
    }

    myBmeSensor.init();
    myLoRaModule.init(DEVADDR, NWKSKEY, APPSKEY, SF);

    SENSOR_VALUES *values = myBmeSensor.getValues();
    sprintf(text, LORA_MSG, values->temperature, values->humidity, values->pressure);

#ifndef TESTMODUS
    // Heltec.VextON(); //auto off due to electronic
    myLoRaModule.send((uint8_t *)text, strlen(text));
#else
    Serial.println(text);
    Serial.flush();
#endif

    startDeepSleep();
}