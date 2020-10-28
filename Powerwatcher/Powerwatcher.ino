#include <Arduino.h>
#include "WiFi_Manager.hpp"
#include "MQTT_Manager.hpp"
#include <sys/time.h>

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_EXECUTE 300    /* Time between measurements (in seconds) 5 min */

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

#define PIN_POWER 25
#define PIN_DEEP_SLEEP 26
#define ADC_PIN_CAPACITOR 34

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool do_meassurement = false;
volatile int meassurement_cause = 0b00000000; //  x x x x trigger_init trigger_power trigger_deepSleep timer
uint64_t lastMess = 0;

struct meassurement_values_str
{
  double voltages[1];
  bool powers[2];
} meassurement_values;

void IRAM_ATTR
onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  do_meassurement = true;
  meassurement_cause = meassurement_cause | 0b00000001; //Timer
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR handleInterruptDeepSleep()
{
  portENTER_CRITICAL_ISR(&timerMux);
  do_meassurement = true;
  meassurement_cause = meassurement_cause | 0b00000010; //trigger_deepSleep
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR handleInterruptPower()
{
  portENTER_CRITICAL_ISR(&timerMux);
  do_meassurement = true;
  meassurement_cause = meassurement_cause | 0b00000100; //trigger_power
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  Serial.begin(115200);

  setup_wifi();
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  mqtt_init();

  Serial.println("Power Watch begin\n");

  pinMode(PIN_DEEP_SLEEP, INPUT_PULLUP);
  pinMode(PIN_POWER, INPUT_PULLUP);

  meassurement_cause = 0b00001000; //trigger_init
  meassurement();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, uS_TO_S_FACTOR * TIME_TO_EXECUTE, true);
  timerAlarmEnable(timer);

  attachInterrupt(digitalPinToInterrupt(PIN_DEEP_SLEEP), handleInterruptDeepSleep, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_POWER), handleInterruptPower, CHANGE);
}

void loop()
{

  if (do_meassurement)
  {
    meassurement();
  }
  mqtt_loop();
}


void createNewMeassurement()
{
  char txtMess[150];
  sprintf(txtMess, "timestamp, time_delta (us), adc_capacitor (V), power, deep_sleep, mess_cause");

  mqtt_send(txtMess);
  meassurement_cause = 0b00001000; //trigger_init
  meassurement();
}

void meassurement()
{
  uint64_t deltaUs = micros() - lastMess;
  lastMess = micros();
  meassurement_values.voltages[0] = readVoltage(ADC_PIN_CAPACITOR); 
  meassurement_values.powers[0] = digitalRead(PIN_POWER);
  meassurement_values.powers[1] = digitalRead(PIN_DEEP_SLEEP);

  char txtMess[100];
  portENTER_CRITICAL_ISR(&timerMux);
  sprintf(txtMess, "%lld, %lld, %.02f, %s, %s, %d",
          getUnixtime(), deltaUs, meassurement_values.voltages[0],
          meassurement_values.powers[0] ? "false" : "true",
          meassurement_values.powers[1] ? "false" : "true",
          meassurement_cause);
  do_meassurement = false;
  meassurement_cause = 0b00000000;
  portEXIT_CRITICAL_ISR(&timerMux);

  Serial.println(txtMess);
  mqtt_send(txtMess);
}

/* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */

double readVoltage(byte pin)
{
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  for (int i = 0; i < 5; i++)       //Mittelung mit minimiren von ausreißern (insg. 6 Werte)
    reading = (reading + analogRead(pin)) / 2;

  if (reading < 1 || reading > 4095)
    return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  double value = -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  return value * 2;
}

int64_t getUnixtime()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (tv_now.tv_sec);
}

int setUnixtime(int32_t unixtime)
{
  timeval epoch = {unixtime, 0};
  return settimeofday((const timeval *)&epoch, 0);
}
