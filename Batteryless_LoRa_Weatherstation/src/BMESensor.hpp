#ifndef SENSOR_BME_HPP
#define SENSOR_BME_HPP

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

#define SEALEVELPRESSURE_HPA (1013.25) //Const needed to calculate height

struct SENSOR_VALUES
{
    float temperature;
    float humidity;
    float pressure;
    float altitude;
} current_values;

class BMESensor
{

    Adafruit_BME280 bme; //BME280 environmental sensor

public:
    bool init()
    {
        bool success = true;
        success &= bme.begin(0x76);
        bme.setSampling(Adafruit_BME280::MODE_FORCED);
        success &= bme.sensorID() != 0xFF;
        return success;
    }

    SENSOR_VALUES *getValues()
    {
        for (int i = 0; i < 5; i++) // Up to 5 tries
        {
            bme.takeForcedMeasurement();
            current_values.temperature = bme.readTemperature();
            current_values.humidity = bme.readHumidity();
            current_values.pressure = bme.readPressure();
            current_values.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
            if (current_values.pressure != 0)
                break;
        }

        return &current_values;
    }

    void printValues(SENSOR_VALUES &values)
    {
        Serial.printf("\nTemperature: \t\t%.2f *C\nHumidity: \t\t%.2f %%\nPressure: \t\t%.2f hPa\nApprox. Altitude: \t%.2f m\n",
                      values.temperature, values.humidity, values.pressure, values.altitude);
    }

    void printValuesToChar(char *buffer, SENSOR_VALUES &values)
    {

        sprintf(buffer, "T\t%.2fC\nH\t%.2f%%\nP\t%.2fhPa",
                values.temperature, values.humidity, values.pressure);
    }

    void printValuesToChar(char *buffer)
    {
        printValuesToChar(buffer, *getValues());
    }

    void printValues()
    {
        printValues(*getValues());
    }
};

#endif