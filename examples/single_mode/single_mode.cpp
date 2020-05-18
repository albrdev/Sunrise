#include <stdint.h>
#include <Arduino.h>
#include "Sunrise.h"

#define PIN_EN                  2
#define PIN_NRDY                3

#define MS_PER_H                ((60UL * 60UL) * 1000UL)
#define MEASUREMENT_INTERVAL    (5UL * 1000UL)

Sunrise sunrise;
uint16_t hourCount = 0U;
unsigned long int nextHour;
unsigned long int nextMeasurement;

volatile uint8_t isReady = false;
void nrdyISR(void)
{
    isReady = true;
}

bool awaitISR(unsigned long int timeout = 2000UL)
{
    timeout += millis();
    while(!isReady && (long)(millis() - timeout) < 0L);
    return isReady;
}

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
    pinMode(PIN_NRDY, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_NRDY), nrdyISR, FALLING);

    // Initial device setup. Also retrieves initial measurement state.
    if(!sunrise.Begin(PIN_EN, true))
    {
        Serial.println("Error: Could not initialize the device");
        while(true);
    }

    sunrise.Awake();
    switchMode(measurementmode_t::MM_SINGLE);

    metercontrol_t mc;
    if(!sunrise.GetMeterControlEE(mc))
    {
        Serial.println("*** ERROR: Could not get meter control");
        while(true);
    }

    if(!mc.nrdy)
    {
        Serial.println("*** ERROR: NRDY option should be enabled");
        while(true);
    }

    sunrise.Sleep();
    Serial.println("Done");
    Serial.println();
    Serial.flush();

    nextHour = millis() + MS_PER_H;
    nextMeasurement = millis();
}

void loop(void)
{
    // Wake up the sensor.
    sunrise.Awake();
    Serial.println("Measuring...");

    // Count hours (taking time rollover in consideration).
    if((long)(millis() - nextHour) >= 0L)
    {
        hourCount++;
        nextHour += MS_PER_H;
    }

    // Single measurement mode requires host device to increment ABC time.
    uint16_t tmpHourCount = sunrise.GetABCTime();
    if(hourCount != tmpHourCount)
    {
        sunrise.SetABCTime(hourCount);
        Serial.print("Setting ABC time:  "); Serial.println(hourCount);
    }

    // Reset ISR variable and start new measurement.
    isReady = false;
    if(sunrise.StartSingleMeasurement())
    {
        // Wait for the sensor to complate the measurement and signal the ISR (or timeout).
        unsigned long int measurementStartTime = millis();
        if(awaitISR())
        {
            unsigned long int measurementDuration = millis() - measurementStartTime;
            Serial.print("Duration:          "); Serial.println(measurementDuration);

            // Read and print measurement values.
            if(sunrise.ReadMeasurement())
            {
                Serial.print("Error status:      "); Serial.println(sunrise.GetErrorStatusRaw(), BIN);
                Serial.print("CO2:               "); Serial.println(sunrise.GetCO2());
                Serial.print("Temperature:       "); Serial.println(sunrise.GetTemperature());
            }
            else
            {
                Serial.println("*** ERROR: Could not read measurement");
            }
        }
        else
        {
            Serial.println("*** ERROR: ISR timeout");
        }
    }
    else
    {
        Serial.println("*** ERROR: Could not start single measurement");
    }
    Serial.println();

    // Put the sensor into sleep mode and wait for the next measurement.
    sunrise.Sleep();
    nextMeasurement += MEASUREMENT_INTERVAL;
    delayUntil(nextMeasurement);
}
