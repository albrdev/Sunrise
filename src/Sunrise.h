/*! \file   Sunrise.h
    \author Alexander Brunström
    \date   2020-04-25

    Contains main class, structs and enum types associated with the Sunrise CO2 sensor.
*/

#ifndef __SUNRISE_H__
#define __SUNRISE_H__

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
    uint8_t reserved2[3]; // Aligned with together with 'count'.
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

    struct
    {
        state_t state;
        int16_t bap;
    } m_State =
    {
        // State
        {
            // ABC
            {
                0U,                             // Time
                { 0U, 0U, 0U, 0U }              // Par0-3
            },
            // Filter
            {
                { 0U, 0U, 0U, 0U, 0U, 0U, 0U }  // Par0-6
            }
        },
        0U                                      // Barometric air pressure
    };

#ifndef __REGION__REGISTER_CONTROL__
    /*! Initiates a I2C command to the sensor.
        \return True on success, false on failure.
    */
    bool BeginCommand(void) const;

    /*! Reads a 1-byte register value from the sensor.
        \param[in]  reg     The register address.
        \param[out] result  The register value, in host endian. On failure, this parameter is left unchanged.
        \return             True on success, false on failure.
    */
    bool ReadRegister1(uint8_t reg, uint8_t& result) const;

    /*! Writes a 1-byte register value to the sensor.
        \param[in]  reg     The register address.
        \param[in]  value   The register value, in host endian.
        \param[in]  delay   The amount of time in milliseconds to wait for the sensor to finish writing.
        \return             True on success, false on failure.
    */
    bool WriteRegister1(uint8_t reg, uint8_t value, unsigned long int delay) const;

    /*! Reads a 2-byte register value from the sensor, taking endian into account.
        \param[in]  reg     The register address.
        \param[out] result  The register value, in host endian. On failure, this parameter is left unchanged.
        \return             True on success, false on failure.
    */
    bool ReadRegister2(uint8_t reg, uint16_t& result) const;

    /*! Writes a 2-byte register value to the sensor, taking endian into account.
        \param[in]  reg     The register address.
        \param[in]  value   The register value, in host endian.
        \param[in]  delay   The amount of time in milliseconds to wait for the sensor to finish writing.
        \return             True on success, false on failure.
    */
    bool WriteRegister2(uint8_t reg, uint16_t value, unsigned long int delay) const;
#endif  // __REGION__REGISTER_CONTROL__

public:
    /*! Querying a device to check if it responds according to the sensor.
        \param[in]  address The device address.
        \return             True on success (the device is most likely a Sunrise sensor), false otherwise.
    */
    static bool PingAddress(uint8_t address);

    /*! Checks if the sensors's error status has been set.
        \return True if the error status has NOT been set, false if an error has occured.
    */
    operator bool(void) const;

#ifndef __REGION__MEASUREMENT_VALUES__
    /*! Gets the full error status field.
        \return The error status.
    */
    uint16_t GetErrorStatusRaw(void) const;

    /*! Checks if a specific error has been set.
        \param[in]  value   The code for the specific error to check.
        \return             True if the specified error has occured, false otherwise.
    */
    bool GetErrorStatus(errorstatus_t value) const;

    /*! Clears the sensor's error status.
        \return True on success, false on failure.
    */
    bool ClearErrorStatus(void);

    /*! Gets the CO2 value previously retrieved by ReadMeasurement().
        \return The CO2 value in ppm.
    */
    uint16_t GetCO2(void) const;

    /*! Gets the temperature value previously retrieved by ReadMeasurement().
        This value is the actual temperature * 100.
        \return The raw temperature in centigrades (Celsius).
    */
    int16_t GetTemperatureRaw(void) const;

    /*! Gets the temperature value previously retrieved by ReadMeasurement().
        \return The temperature in centigrades (Celsius).
    */
    float GetTemperature(void) const;

    /*! Gets the measurement count previously retrieved by ReadMeasurement().
        This value gets reset when the sensor is powered down.
        \return The measurement count.
    */
    uint8_t GetMeasurementCount(void) const;

    /*! Gets the measurement cycle time previously retrieved by ReadMeasurement().
        This value show the number of cycles passed (incremented every 2 seconds) in the current measurement.
        e.g. a value of 3 means 3 * 2 = 6 total seconds passed.
        \return The measurement cycle time.
    */
    uint16_t GetMeasurementCycleTime(void) const;

    /*! Gets the CO2 (unfiltered, pressure compensated) value previously retrieved by ReadMeasurement().
        \return The CO2 value in ppm.
    */
    uint16_t GetCO2_UP(void) const;

    /*! Gets the CO2 (filtered) value previously retrieved by ReadMeasurement().
        \return The CO2 value in ppm.
    */
    uint16_t GetCO2_F(void) const;

    /*! Gets the CO2 (unfiltered) value previously retrieved by ReadMeasurement().
        \return The CO2 value in ppm.
    */
    uint16_t GetCO2_U(void) const;
#endif  // __REGION__REGISTER_VALUES__

#ifndef __REGION__BAROMETRIC_PRESSURE__
    /*! Gets the raw barometric air pressure cached in the internal state.
        See SetStateBarometricAirPressureRaw().
        \return The barometric air pressure in hPa.
    */
    int16_t GetStateBarometricAirPressureRaw(void) const;

    /*! Sets the raw barometric air pressure cached in the internal state.
        This value is automatically written (along with state) to the sensor at the next call to StartSingleMeasurement() when in single measurement mode.
        \param[in]  value   The barometric air pressure in hPa.
    */
    void SetStateBarometricAirPressureRaw(int16_t value);

    /*! Gets the barometric air pressure cached in the internal state.
        See SetStateBarometricAirPressure().
        \return The barometric air pressure in kPa.
    */
    float GetStateBarometricAirPressure(void) const;

    /*! Sets the barometric air pressure cached in the internal state.
        This value is automatically written (along with state) to the sensor at the next call to StartSingleMeasurement(), when in single measurement mode.
        \param[in]  value   The barometric air pressure in kPa.
    */
    void SetStateBarometricAirPressure(float value);
#endif  // __REGION__BAROMETRIC_PRESSURE__

#ifndef __REGION__MEASUREMENT_CONTROL__
    /*! Restores previous state and signals the sensor to start a new measurement.
        Has no effect on the sensor if the sensor is in continuous measurement mode.
        \return True on success, false on failure. Always returns true if the sensor is responsive and in continuous measurement mode.
    */
    bool StartSingleMeasurement(void) const;

    /*! Retrieves CO2 and other valus from the sensor.
        Optionally also saves the sensor's filtering and calibration state.
        \param[in]  saveState   If true, the sensor's filtering and calibration state is saved.
        \return                 True on success, false on failure.
    */
    bool ReadMeasurement(bool saveState);

    /*! Retrieves CO2 and other valus from the sensor.
        Also saves filtering and calibration state if the sensor is in single measurement mode, otherwise not.
        \return True on success, false on failure.
    */
    bool ReadMeasurement(void);
#endif  // __REGION__MEASUREMENT_CONTROL__

#ifndef __REGION__STATE__
    /*! Reads and internally saves the sensor's filtering and calibration state.
        Also saves filtering and calibration state if the sensor is in single measurement mode, otherwise not.
        \return True on success, false on failure.
    */
    bool ReadState(void);

    /*! Gets the cached, previously retrieved filtering and calibration state.
        \return A reference to the internal state.
    */
    const state_t& GetState(void) const;

    /*! Sets the cached filtering and calibration state.
        The state is automatically written to the sensor at the next call to StartSingleMeasurement(), when in single measurement mode.
        \param[in]  value   The state.
    */
    void SetState(const state_t& value);

    /*! Sets the cached filtering and calibration state.
        The state is automatically written to the sensor at the next call to StartSingleMeasurement(), when in single measurement mode.
        Optionally sets the raw barometric air pressure.
        \param[in]  value   The state.
        \param[in]  bap     The raw barometric air pressure.
    */
    void SetState(const state_t& value, int16_t bap);

    /*! Sets the cached filtering and calibration state.
        The state is automatically written to the sensor at the next call to StartSingleMeasurement(), when in single measurement mode.
        Optionally sets the barometric air pressure.
        \param[in]  value   The state.
        \param[in]  bap     The barometric air pressure.
    */
    void SetState(const state_t& value, float bap);

    /*! Gets the cached ABC time, i.e. the time passed since the last ABC calibration, in hours.
        \return             The time since the last ABC calibration.
    */
    uint16_t GetABCTime(void) const;

    /*! Sets the ABC time, i.e. the time passed since the last ABC calibration, in hours.
        The value is automatically written to the sensor at the next call to StartSingleMeasurement(), when in single measurement mode.
        \param[in]  value   The time since the last ABC calibration.
    */
    void SetABCTime(uint16_t value);
#endif  // __REGION__STATE__

#ifndef __REGION__CALIBRATION_CONTROL__
    /*! Gets the calibration status.
        Should be called after a measurement has been performed.
        The status should be cleared with before the next measurement.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetCalibrationStatus(uint8_t& result) const;

    /*! Clears the calibration status.
        \return             True on success, false on failure.
    */
    bool ClearCalibrationStatus(void) const;

    /*! Gets the calibration command.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetCalibrationCommand(calibration_t& result) const;

    /*! Sets the calibration command.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetCalibrationCommand(calibration_t value, bool clearStatus = true) const;

    /*! Gets the target calibration value.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetCalibrationTarget(uint16_t& result) const;

    /*! Sets the target calibration value.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetCalibrationTarget(uint16_t value) const;

    /*! Gets the CO2 value override.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetCO2ValueOverride(uint16_t& result) const;

    /*! Sets the CO2 value override.
        If the value lower than default, both filtered and unfiltered CO2 value will be set to this value after the next measurement.
        Default value: 32767 (no override).
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetCO2ValueOverride(uint16_t value) const;
#endif  // __REGION__CALIBRATION_CONTROL__

#ifndef __REGION__MEASUREMENT_SETTINGS__
    /*! Gets the measurement mode.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetMeasurementModeEE(measurementmode_t& result) const;

    /*! Sets the measurement mode.
        The sensor has two modes:
        - Continuous measurement mode (0)
        - Single measurement mode (1)
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetMeasurementModeEE(measurementmode_t value) const;

    /*! Gets the measurement period, i.e. the time between measurements, in seconds.
        A sensor restart is required to take effect.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetMeasurementPeriodEE(uint16_t& result) const;

    /*! Sets the measurement period, i.e. the time between measurements, in seconds.
        Only used by the sensor in continuous measurement mode.
        Odd numbers will be rounded up to nearest even number.
        A sensor restart is required to take effect.
        Allowed values: 2-65534.
        Default value: 16
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetMeasurementPeriodEE(uint16_t value) const;

    /*! Gets number of samples per measurement.
        Each sample takes max. 200 ms to complete.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetNumberOfSamplesEE(uint16_t& result) const;

    /*! Sets number of samples per measurement.
        Each sample takes max. 200 ms to complete.
        Allowed values: 1-1024.
        Default value: 8.
        A sensor restart is required to take effect.
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetNumberOfSamplesEE(uint16_t value) const;
#endif  // __REGION__MEASUREMENT_SETTINGS__

#ifndef __REGION__FILTERING_SETTINGS__
    /*! Gets the ABC (Automatic background calibration) period, in hours.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetABCPeriodEE(uint16_t& result) const;

    /*! Sets the ABC (Automatic background calibration) period, in hours.
        Allowed values: 1-65534.
        Default value: 180.
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetABCPeriodEE(uint16_t value) const;

    /*! Gets the ABC (Automatic background calibration) target, in ppm.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetABCTargetEE(uint16_t& result) const;

    /*! Sets the ABC (Automatic background calibration) target, in ppm.
        Default value: 400.
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetABCTargetEE(uint16_t value) const;

    /*! Gets the static IIR (Infinite impulse response) filter parameter.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetStaticIIRFilterParameterEE(uint8_t& result) const;

    /*! Sets the static IIR (Infinite impulse response) filter parameter.
        A higher value corresponds to a harder filtration.
        Allowed values: 2-10.
        Default value: .
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetStaticIIRFilterParameterEE(const uint8_t value) const;
#endif  // __REGION__FILTERING_SETTINGS__

#ifndef __REGION__METER_CONTROL__
    /*! Gets the raw meter control.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetMeterControlRawEE(uint8_t& result) const;

    /*! Sets the raw meter control.
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetMeterControlRawEE(uint8_t value) const;

    /*! Gets the meter control.
        \note               This method will read from the sensor's EEPROM and that usually takes a little longer than other methods.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetMeterControlEE(metercontrol_t& result) const;

    /*! Sets the meter control.
        \attention          This method will write to the sensor's EEPROM and should be used sparsly.
        \param[in]  value   The value to be set.
        \return             True on success, false on failure.
    */
    bool SetMeterControlEE(const metercontrol_t& value) const;
#endif  // __REGION__METER_CONTROL__

#ifndef __REGION__MISC__
    /*! Changes the sensor address.
        A sensor restart is required to take effect.
        \param[in]  value   The new address.
        \param[in]  restart If true, the sensor is also restarted.
        \return             True on success, false on failure.
    */
    bool ChangeAddress(uint8_t value, bool restart = true);

    /*! Gets the raw firmware revision.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetFirmwareRevisionRaw(uint16_t& result) const;

    /*! Gets the firmware revision separated into two values, main and sub.
        \param[out] main    The main revision. Remains unchanged on failure.
        \param[out] sub     The sub revision. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetFirmwareRevision(uint8_t& main, uint8_t& sub) const;

    /*! Gets the sensor ID.
        \param[out] result  The requested value. Remains unchanged on failure.
        \return             True on success, false on failure.
    */
    bool GetSensorID(uint32_t& result) const;
#endif  // __REGION__MISC__

#ifndef __REGION__BASIC_CONTROL__
    /*! Wakes up the sensor by setting the enable pin high.
        Intended to be used with single measurement mode.
        Only has effect if the enable pin has previously been set.
    */
    void Awake(void) const;

    /*! Puts the sensor into sleep mode by setting the enable pin low.
        Intended to be used with single measurement mode.
        Only has effect if the enable pin has previously been set.
    */
    void Sleep(void) const;

    /*! Restarts the sensor.
        Necessary to reload changes to certain settings.
        \return True on success, false on failure.
    */
    bool Restart(void) const;

    /*! Restarts the sensor by putting it in sleep mode, then waking it back up.
        Necessary to reload changes to certain settings.
        Call this if Restart() fails.
        Only has effect if enable pin is previously set.
        \return True on success, false on failure.
    */
    bool HardRestart(void) const;

    /*! Check if the device is active and responds according to Sunrise.
        \return True on success, false on failure.
    */
    bool Ping(void) const;
#endif  // __REGION__BASIC_CONTROL__

#ifndef __REGION__INITIALIZATION__
    /*! Initiates the sensor.
        Optionally retrieve the sensor's initial filtering and calibration state.
        \param[in]  readInitialState    If true, retrieve the initial filtering and calibration state.
        \return                         True on success, false on failure.
    */
    bool Begin(bool readInitialState = false);

    /*! Initiates the sensor with the enable pin.
        Optionally retrieve the sensor's initial filtering and calibration state.
        \param[in]  pinEN               The enable pin number.
        \param[in]  readInitialState    If true, retrieve the initial filtering and calibration state.
        \return                         True on success, false on failure.
    */
    bool Begin(uint8_t pinEN, bool readInitialState = false);

    /*! Initiates the sensor with the enable pin.
        Optionally retrieve the sensor's initial filtering and calibration state.
        \param[in]  pinEN               The enable pin number.
        \param[in]  readInitialState    If true, retrieve the initial filtering and calibration state.
        \return                         True on success, false on failure.
    */
    bool Begin(int pinEN, bool readInitialState = false);

    /*! Class constructor.
        Optionally specifies the sensor's device address, otherwise the default address is used.
        \param[in]  address The sensor I2C address.
    */
    Sunrise(uint8_t address = SUNRISE_DEFAULT_ADDRESS);
#endif  // __REGION__INITIALIZATION__
};

#endif // __SUNRISE_H__
