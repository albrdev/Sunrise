/*! \file   Sunrise.hpp
    \author Alexander Brunström
    \date   2020-04-25

    Contains main class, structs and enum types associated with the Sunrise CO2 sensor.
 */

#ifndef __SUNRISE_HPP__
#define __SUNRISE_HPP__

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

#define SUNRISE_DEFAULT_ADDRESS 0x68
#define SUNRISE_INVALID_PIN     0xFF

typedef struct
{
    // Structure padding shouldn't occur here, no packing necessary.
    uint16_t errorstatus;
    uint8_t reserved1[4];
    uint16_t co2;
    int16_t temp;
    uint8_t reserved2[3]; // Aligned with together with 'count'
    uint8_t count;
    uint16_t cycleTime;
    uint16_t co2_up;
    uint16_t co2_f;
    uint16_t co2_u;
} measurement_t;

typedef struct
{
    struct
    {
        uint16_t time;
        uint16_t par[4];
    } abc;

    struct
    {
        uint16_t par[7];
    } filter;

    int16_t bap;
} state_t;

typedef enum : uint16_t
{
    ES_FATAL                = 0,
    ES_I2C                  = 1,
    ES_ALGORITHM            = 2,
    ES_CALIBRATION          = 3,
    ES_SELF_DIAGNOSTICS     = 4,
    ES_OUT_OF_RANGE         = 5,
    ES_MEMORY               = 6,
    ES_NOT_COMPLETED        = 7,

    ES_LOW_INTERNAL_VOLTAGE = 8,
    ES_MEASUREMENT_TIMEOUT  = 9
} errorstatus_t;

typedef struct
{
    bool nrdy;
    bool abc;
    bool static_iir;
    bool dynamic_iir;
    bool pressure_compensation;
} metercontrol_t;

typedef enum : uint8_t
{
    MM_CONTINUOUS   = 0x00,
    MM_SINGLE       = 0x01
} measurementmode_t;

typedef enum : uint16_t
{
    C_FACTORY_RESET = 0x7C02,
    C_ABC_FORCED    = 0x7C03,
    C_TARGET        = 0x7C05,
    C_ABC           = 0x7C06,
    C_ZERO          = 0x7C07
} calibration_t;

class Sunrise
{
private:
    uint8_t m_Address;
    uint8_t m_PinEN = SUNRISE_INVALID_PIN;

    uint16_t m_ErrorStatus = 0U;
    uint16_t m_CO2 = 0U;
    int16_t m_Temperature = 0;

    measurement_t m_Measurement =
    {
        0U,                 // Error status
        { 0U, 0U, 0U, 0U }, // Reserved 1
        0U,                 // CO2 (Filtered & pressure compensated)
        0,                  // Chip temperature
        { 0U, 0U, 0U },     // Reserved 2
        0U,                 // Measurement count
        0U,                 // Measurement cycle time
        0U,                 // CO2 (Unfiltered & pressure compensated)
        0U,                 // CO2 (Filtered)
        0U                  // CO2, (Unfiltered)
    };

    state_t m_State =
    {
        // ABC
        {
            0U,                             // Time
            { 0U, 0U, 0U, 0U }              // Par0-3
        },
        // Filter
        {
            { 0U, 0U, 0U, 0U, 0U, 0U, 0U }  // Par0-6
        },
        0U                                  // Barometric air pressure
    };

    bool BeginCommand(void) const;
    bool ReadRegister1(uint8_t reg, uint8_t& result) const;
    bool WriteRegister1(uint8_t reg, uint8_t value, unsigned long int delay) const;
    bool ReadRegister2(uint8_t reg, uint16_t& result) const;
    bool WriteRegister2(uint8_t reg, uint16_t value, unsigned long int delay) const;

public:
    static bool PingAddress(uint8_t address);

    operator bool(void) const;

    bool GetErrorStatus(errorstatus_t value) const;
    uint16_t GetErrorStatusRaw(void) const;
    bool ClearErrorStatus(void);

    uint16_t GetCO2(void) const;
    float GetTemperature(void) const;
    int16_t GetTemperatureRaw(void) const;
    uint8_t GetMeasurementCount(void) const;
    uint16_t GetMeasurementCycleTime(void) const;
    uint16_t GetCO2_UP(void) const;
    uint16_t GetCO2_F(void) const;
    uint16_t GetCO2_U(void) const;
    int16_t GetStateBarometricAirPressure(void) const;
    void SetStateBarometricAirPressure(int16_t value);
    bool GetBarometricAirPressure(int16_t& result) const;
    bool SetBarometricAirPressure(int16_t value) const;

    bool StartSingleMeasurement(void) const;
    bool ReadMeasurement(bool saveState);
    bool ReadMeasurement(void);
    bool ReadState(void);
    const state_t& GetState(void) const;
    void SetState(const state_t& value);
    uint16_t GetABCTime(void) const;
    void SetABCTime(uint16_t value);

    bool GetCalibrationStatus(uint8_t& result) const;
    bool ClearCalibrationStatus(void) const;
    bool GetCalibrationCommand(calibration_t& result) const;
    bool SetCalibrationCommand(calibration_t value, bool clearStatus = true) const;
    bool GetCalibrationTarget(uint16_t& result) const;
    bool SetCalibrationTarget(uint16_t value) const;

    bool GetCO2ValueOverride(uint16_t& result) const;
    bool SetCO2ValueOverride(uint16_t value) const;

    bool GetMeasurementModeEE(measurementmode_t& result) const;
    bool SetMeasurementModeEE(measurementmode_t value) const;
    bool GetMeasurementPeriodEE(uint16_t& result) const;
    bool SetMeasurementPeriodEE(uint16_t value) const;
    bool GetNumberOfSamplesEE(uint16_t& result) const;
    bool SetNumberOfSamplesEE(uint16_t value) const;
    bool GetABCPeriodEE(uint16_t& result) const;
    bool SetABCPeriodEE(uint16_t value) const;

    bool GetABCTargetEE(uint16_t& result) const;
    bool SetABCTargetEE(uint16_t value) const;
    bool GetStaticIIRFilterParameterEE(uint8_t& result) const;
    bool SetStaticIIRFilterParameterEE(const uint8_t value) const;

    bool GetMeterControlRawEE(uint8_t& result) const;
    bool GetMeterControlEE(metercontrol_t& result) const;
    bool SetMeterControlRawEE(uint8_t value) const;
    bool SetMeterControlEE(const metercontrol_t& value) const;

    bool Restart(void) const;
    bool HardRestart(void) const;
    bool WriteAddress(uint8_t value, bool restart = true);

    bool GetFirmwareRevisionRaw(uint16_t& result) const;
    bool GetFirmwareRevision(uint8_t& main, uint8_t& sub) const;
    bool GetSensorID(uint32_t& result) const;

    bool Ping(void) const;

    void Awake(void) const;
    void Sleep(void) const;

    bool Begin(bool readInitialState = false);
    bool Begin(uint8_t pinEN, bool readInitialState = false);
    bool Begin(int pinEN, bool readInitialState = false);

    Sunrise(uint8_t address = SUNRISE_DEFAULT_ADDRESS);
};

#endif // __SUNRISE_HPP__
