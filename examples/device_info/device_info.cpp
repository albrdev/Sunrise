#include <stdint.h>
#include <Arduino.h>
#include "Sunrise.h"

#define PIN_EN  2

Sunrise sunrise;

void printStaticInfo(void)
{
    uint16_t firmwareRevisionRaw;
    uint8_t firmwareRevisionMain;
    uint8_t firmwareRevisionSub;
    uint32_t sensorID;

    Serial.println("[Static info]");
    // Firmware revision raw
    if(!sunrise.GetFirmwareRevisionRaw(firmwareRevisionRaw))
    {
        Serial.println("Error: Could not get firmware revision raw");
        while(true);
    }
    Serial.print("Firmware revision raw:  "); Serial.println(firmwareRevisionRaw);

    // Firmware revision
    if(!sunrise.GetFirmwareRevision(firmwareRevisionMain, firmwareRevisionSub))
    {
        Serial.println("Error: Could not get raw firmware revision");
        while(true);
    }
    Serial.print("Firmware revision:      "); Serial.print(firmwareRevisionMain); Serial.print("."); Serial.println(firmwareRevisionSub);

    // Sensor ID
    if(!sunrise.GetSensorID(sensorID))
    {
        Serial.println("Error: Could not get sensor ID");
        while(true);
    }
    Serial.print("Sensor ID:              "); Serial.println(sensorID);
    Serial.println();
}

void printMeterControl(void)
{
    uint8_t meterControlRaw;
    metercontrol_t meterControl;

    Serial.println("[Meter control]");
    // Meter control raw
    if(!sunrise.GetMeterControlRawEE(meterControlRaw))
    {
        Serial.println("Error: Could not get meter control raw");
        while(true);
    }
    Serial.print("Raw:                    "); Serial.println(meterControlRaw, BIN);

    // Meter control
    if(!sunrise.GetMeterControlEE(meterControl))
    {
        Serial.println("Error: Could not get meter control");
        while(true);
    }

    Serial.print("NRDY:                   "); Serial.println(meterControl.nrdy ? "ENABLED" : "DISABLED");
    Serial.print("ABC:                    "); Serial.println(meterControl.abc ? "ENABLED" : "DISABLED");
    Serial.print("Static IIR:             "); Serial.println(meterControl.static_iir ? "ENABLED" : "DISABLED");
    Serial.print("Dynamic IIR:            "); Serial.println(meterControl.dynamic_iir ? "ENABLED" : "DISABLED");
    Serial.print("Pressure compensation:  "); Serial.println(meterControl.pressure_compensation ? "ENABLED" : "DISABLED");
    Serial.println();
}

void printMeasurementSettings(void)
{
    measurementmode_t measurementMode;
    uint16_t measurementPeriod;
    uint16_t numberOfSamples;
    uint16_t abcPeriod;
    uint16_t abcTarget;
    uint8_t staticIIRFilterParameter;

    Serial.println("[Measurement settings]");
    // Measurement mode
    if(!sunrise.GetMeasurementModeEE(measurementMode))
    {
        Serial.println("Error: Could not get measurement mode");
        while(true);
    }
    Serial.print("Measurement mode:             "); Serial.println(measurementMode == measurementmode_t::MM_CONTINUOUS ? "Continuous" : "Single");

    // Measurement period
    if(!sunrise.GetMeasurementPeriodEE(measurementPeriod))
    {
        Serial.println("*** ERROR: Could not get measurement period");
        while(true);
    }
    Serial.print("Measurement period:           "); Serial.print(measurementPeriod); Serial.println("s");

    // Number of samples
    if(!sunrise.GetNumberOfSamplesEE(numberOfSamples))
    {
        Serial.println("*** ERROR: Could not get number of samples");
        while(true);
    }
    Serial.print("Number of samples:            "); Serial.println(numberOfSamples);

    // ABC period
    if(!sunrise.GetABCPeriodEE(abcPeriod))
    {
        Serial.println("Error: Could not get ABC period");
        while(true);
    }
    Serial.print("ABC period:                   "); Serial.print(abcPeriod); Serial.println("h");

    // ABC target
    if(!sunrise.GetABCTargetEE(abcTarget))
    {
        Serial.println("Error: Could not get ABC target");
        while(true);
    }
    Serial.print("ABC target:                   "); Serial.print(abcTarget); Serial.println("ppm");

    // Static IIR filter parameter
    if(!sunrise.GetStaticIIRFilterParameterEE(staticIIRFilterParameter))
    {
        Serial.println("Error: Could not get static IIR fitler parameter");
        while(true);
    }
    Serial.print("Static IIR fitler parameter:  "); Serial.println(staticIIRFilterParameter);
    Serial.println();
}

void setup(void)
{
    delay(2500UL);
    Serial.begin(9600);
    while(!Serial);

    Serial.println("Initializing...");
    sunrise.Begin(PIN_EN);
    sunrise.Awake();

    Serial.println("Done");
    Serial.println();
    Serial.flush();

    printStaticInfo();
    printMeterControl();
    printMeasurementSettings();

    sunrise.Sleep();
}

void loop(void)
{
}
