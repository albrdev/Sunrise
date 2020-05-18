#include <stdint.h>
#include <Arduino.h>
#include "Sunrise.h"

#define PIN_EN              2
#define SAMPLE_DURATION_MAX 200UL

Sunrise sunrise;
unsigned long int measurementInterval;
unsigned long int measurementDuration;
unsigned long int nextMeasurement;

void delayUntil(unsigned long int time)
{
    while((long)(millis() - time) < 0L);
}

void switchMode(measurementmode_t mode)
{
    while(true)
    {
        measurementmode_t measurementMode;
        if(!sunrise.GetMeasurementModeEE(measurementMode))
        {
            Serial.println("*** ERROR: Could not get measurement mode");
            while(true);
        }

        if(measurementMode == mode)
        {
            break;
        }

        Serial.println("Attempting to switch measurement mode...");
        if(!sunrise.SetMeasurementModeEE(mode))
        {
            Serial.println("*** ERROR: Could not set measurement mode");
            while(true);
        }

        if(!sunrise.HardRestart())
        {
            Serial.println("*** ERROR: Failed to restart the device");
            while(true);
        }
    }
}

void setup(void)
{
    delay(2500UL);
    Serial.begin(9600);
    while(!Serial);

    Serial.println("Initializing...");

    // 1.1  If a custom pin is prefered over VCC.
    //      Note:   If using the reset method/command, the state of the ENABLE pin must be set to LOW and then HIGH again to wake up the device.
    //              This is done automatically if Begin() is provided with the pin connected to ENABLE.
    sunrise.Begin(PIN_EN);

    // 2.1  If VCC is prefered:
    //      Note:   If using the reset method/command, you must manuelly set ENABLE pin to LOW and then back to HIGH again.
    //              Either by swapping the ENABLE pin from VCC to ground and back or cut the power to the device entirely.

    // 1.2  Awake() is only necessary if going with option 1.
    sunrise.Awake();

    switchMode(measurementmode_t::MM_CONTINUOUS);

    uint16_t mp;
    if(!sunrise.GetMeasurementPeriodEE(mp))
    {
        Serial.println("*** ERROR: Could not get measurement period");
        while(true);
    }
    Serial.print("Measurement period:  "); Serial.println(mp);
    measurementInterval = mp * 1000UL;

    uint16_t nos;
    if(!sunrise.GetNumberOfSamplesEE(nos))
    {
        Serial.println("*** ERROR: Could not get number of samples");
        while(true);
    }
    Serial.print("Number of samples:   "); Serial.println(nos);
    measurementDuration = SAMPLE_DURATION_MAX * nos;

    Serial.println("Done");
    Serial.println();
    Serial.flush();

    nextMeasurement = measurementInterval;
}

void loop(void)
{
    delayUntil(nextMeasurement);
    nextMeasurement += measurementInterval;
    delay(measurementDuration);
    delay(500UL);

    Serial.println("Measuring...");
    if(sunrise.ReadMeasurement())
    {
        Serial.print("Error status:  "); Serial.println(sunrise.GetErrorStatusRaw(), BIN);
        Serial.print("CO2:           "); Serial.println(sunrise.GetCO2());
        Serial.print("Temperature:   "); Serial.println(sunrise.GetTemperature());
        Serial.print("Count:         "); Serial.println(sunrise.GetMeasurementCount());
        Serial.print("Cycle time:    "); Serial.println(sunrise.GetMeasurementCycleTime());
    }
    else
    {
        Serial.println("*** ERROR: Could not read measurement");
    }
    Serial.println();
}
