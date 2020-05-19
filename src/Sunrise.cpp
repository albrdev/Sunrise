#include "Sunrise.h"
#include <stddef.h>
#include <string.h>
#include "constants.h"

typedef enum : uint8_t
{
    WS_ACK = 0,
    WS_TX_OVERFLOW = 1,
    WS_NACK_ADDRESS = 2,
    WS_NACK_DATA = 3,
    WS_OTHER = 4
} wirestatus_t;

typedef enum : uint8_t
{
    MCI_NRDY = 0,
    MCI_ABC = 1,
    MCI_STATIC_IIR = 2,
    MCI_DYNAMIC_IIR = 3,
    MCI_PRESSURE_COMPENSATION = 4
} metercontrolindex_t;

static inline int16_t swapEndian(int16_t value)
{
    #ifndef SUNRISE_HOST_BIGENDIAN
    return ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);
    #else
    return value;
    #endif
}

static inline uint16_t swapEndian(uint16_t value)
{
    #ifndef SUNRISE_HOST_BIGENDIAN
    return ((value >> 8) & 0x00FF) | ((value << 8) & 0xFF00);
    #else
    return value;
    #endif
}

static inline uint32_t swapEndian(uint32_t value)
{
    #ifndef SUNRISE_HOST_BIGENDIAN
    return ((value >> 24) & 0x000000FF) | ((value >> 8) & 0x0000FF00) | ((value << 8) & 0x00FF0000) | ((value << 24) & 0xFF000000);
    #else
    return value;
    #endif
}

static size_t I2CRead(int addr, void* result, size_t size, bool sendStop = true, unsigned long int timeout = DELAY_TIMEOUT)
{
    Wire.requestFrom(addr, size, sendStop);
    timeout += millis();
    size_t avail;
    while((avail = (size_t)Wire.available()) < size && (long)(millis() - timeout) < 0L);
    return avail > 0U ? Wire.readBytes((uint8_t*)result, avail) : 0U;
}

static uint8_t I2CWrite(int addr, const void* data, size_t size, bool sendStop = true, unsigned long int delayTime = DELAY_SRAM)
{
    Wire.beginTransmission(addr);
    Wire.write((uint8_t*)data, size);
    delay(1UL);
    uint8_t result = Wire.endTransmission(sendStop);

    delay(delayTime);
    return result;
}

bool Sunrise::BeginCommand(void) const
{
    Wire.beginTransmission(m_Address);
    delay(1UL);
    bool status = (Wire.endTransmission(true) == wirestatus_t::WS_NACK_ADDRESS);
    delay(DELAY_SRAM);
    return status;
}

bool Sunrise::ReadRegister1(uint8_t reg, uint8_t& result) const
{
    return BeginCommand() && I2CWrite(m_Address, &reg, sizeof(reg)) == wirestatus_t::WS_ACK && I2CRead(m_Address, &result, sizeof(result)) == sizeof(result);
}

bool Sunrise::WriteRegister1(uint8_t reg, uint8_t value, unsigned long int delayTime) const
{
    uint8_t tmpBuffer[sizeof(reg) + sizeof(value)];
    tmpBuffer[0] = reg;
    memcpy(&tmpBuffer[sizeof(reg)], &value, sizeof(value));
    return BeginCommand() && I2CWrite(m_Address, tmpBuffer, sizeof(tmpBuffer), true, delayTime * 2UL) == wirestatus_t::WS_ACK;
}

bool Sunrise::ReadRegister2(uint8_t reg, uint16_t& result) const
{
    bool status = BeginCommand() && I2CWrite(m_Address, &reg, sizeof(reg)) == wirestatus_t::WS_ACK && I2CRead(m_Address, &result, sizeof(result)) == sizeof(result);
    if(status)
    {
        result = swapEndian(result);
    }

    return status;
}

bool Sunrise::WriteRegister2(uint8_t reg, uint16_t value, unsigned long int delayTime) const
{
    uint8_t tmpBuffer[sizeof(reg) + sizeof(value)];
    tmpBuffer[0] = reg;
    value = swapEndian(value);
    memcpy(&tmpBuffer[sizeof(reg)], &value, sizeof(value));
    return BeginCommand() && I2CWrite(m_Address, tmpBuffer, sizeof(tmpBuffer), true, delayTime * 2UL) == wirestatus_t::WS_ACK;
}

bool Sunrise::PingAddress(uint8_t address)
{
    Wire.beginTransmission(address);
    if(Wire.endTransmission(true) != wirestatus_t::WS_NACK_ADDRESS)
    {
        return false;
    }
    delay(DELAY_SRAM);

    const uint8_t reg = REG_MB_I2C_ADDRESS;
    uint8_t tmpResult;
    if(!I2CWrite(address, &reg, sizeof(reg)) == wirestatus_t::WS_ACK)
    {
        return false;
    }

    if(I2CRead(address, &tmpResult, sizeof(tmpResult)) != sizeof(tmpResult))
    {
        return false;
    }

    return tmpResult == address;
}

Sunrise::operator bool(void) const
{
    return m_Measurement.errorstatus == 0U;
}

uint16_t Sunrise::GetErrorStatusRaw(void) const
{
    return m_Measurement.errorstatus;
}

bool Sunrise::GetErrorStatus(errorstatus_t value) const
{
    return (m_Measurement.errorstatus & (1 << value)) != 0;
}

bool Sunrise::ClearErrorStatus(void)
{
    bool status = WriteRegister1(REG_CLEAR_ERRORSTATUS, 0x00, DELAY_SRAM);
    if(status)
    {
        m_Measurement.errorstatus = 0x00;
    }

    return status;
}

uint16_t Sunrise::GetCO2(void) const
{
    return m_Measurement.co2;
}

int16_t Sunrise::GetTemperatureRaw(void) const
{
    return m_Measurement.temp;
}

float Sunrise::GetTemperature(void) const
{
    return (float)m_Measurement.temp / 100.0f;
}

uint8_t Sunrise::GetMeasurementCount(void) const
{
    return m_Measurement.count;
}

uint16_t Sunrise::GetMeasurementCycleTime(void) const
{
    return m_Measurement.cycleTime;
}

uint16_t Sunrise::GetCO2_UP(void) const
{
    return m_Measurement.co2_up;
}

uint16_t Sunrise::GetCO2_F(void) const
{
    return m_Measurement.co2_f;
}

uint16_t Sunrise::GetCO2_U(void) const
{
    return m_Measurement.co2_u;
}

int16_t Sunrise::GetStateBarometricAirPressureRaw(void) const
{
    return swapEndian(m_State.bap);
}

void Sunrise::SetStateBarometricAirPressureRaw(int16_t value)
{
    if(value < BAROMETRIC_AIR_PRESSURE_MIN || value > BAROMETRIC_AIR_PRESSURE_MAX)
    {
        return;
    }

    m_State.bap = swapEndian(value);
}

float Sunrise::GetStateBarometricAirPressure(void) const
{
    return (float)GetStateBarometricAirPressureRaw() / KPA_IN_HPA;
}

void Sunrise::SetStateBarometricAirPressure(float value)
{
    SetStateBarometricAirPressureRaw((int16_t)(value * KPA_IN_HPA));
}

bool Sunrise::StartSingleMeasurement(void) const
{
    measurementmode_t mode;
    if(!GetMeasurementModeEE(mode))
    {
        return false;
    }

    if(mode != measurementmode_t::MM_SINGLE)
    {
        return true;
    }

    uint8_t mc;
    if(!GetMeterControlRawEE(mc))
    {
        return false;
    }

    size_t size = (mc & (1 << metercontrolindex_t::MCI_PRESSURE_COMPENSATION)) == 0U ? sizeof(m_State) : sizeof(m_State.state);

    uint8_t cmd[] = { REGM_START_SINGLE_MEASUREMENT, 0x01 };
    if(!BeginCommand())
    {
        return false;
    }

    if(I2CWrite(m_Address, cmd, sizeof(cmd), false) != wirestatus_t::WS_ACK)
    {
        return false;
    }

    return I2CWrite(m_Address, &m_State, size, true, DELAY_SRAM * 13UL) == wirestatus_t::WS_ACK;
}

bool Sunrise::ReadMeasurement(bool saveState)
{
    if(!BeginCommand())
    {
        return false;
    }

    const uint8_t reg = REG_ERRORSTATUS_MSB;
    if(I2CWrite(m_Address, &reg, sizeof(reg)) != wirestatus_t::WS_ACK)
    {
        return false;
    }

    if(I2CRead(m_Address, &m_Measurement, sizeof(m_Measurement)) != sizeof(m_Measurement))
    {
        return false;
    }

    m_Measurement.errorstatus = swapEndian(m_Measurement.errorstatus);
    m_Measurement.co2 = swapEndian(m_Measurement.co2);
    m_Measurement.temp = swapEndian(m_Measurement.temp);
    m_Measurement.cycleTime = swapEndian(m_Measurement.cycleTime);
    m_Measurement.co2_up = swapEndian(m_Measurement.co2_up);
    m_Measurement.co2_f = swapEndian(m_Measurement.co2_f);
    m_Measurement.co2_u = swapEndian(m_Measurement.co2_u);

    return !saveState || ReadState();
}

bool Sunrise::ReadMeasurement(void)
{
    measurementmode_t mode;
    if(!GetMeasurementModeEE(mode))
    {
        return false;
    }

    return ReadMeasurement(mode == measurementmode_t::MM_SINGLE);
}

bool Sunrise::ReadState(void)
{
    uint8_t cmd[] = { REGM_ABC_TIME_MSB };

    if(!BeginCommand())
    {
        return false;
    }

    if(I2CWrite(m_Address, cmd, sizeof(cmd)) != wirestatus_t::WS_ACK)
    {
        return false;
    }

    if(I2CRead(m_Address, &m_State.state, sizeof(m_State.state)) != sizeof(m_State.state))
    {
        return false;
    }

    return true;
}

const state_t& Sunrise::GetState(void) const
{
    return m_State.state;
}

void Sunrise::SetState(const state_t& value)
{
    m_State.state = value;
}

uint16_t Sunrise::GetABCTime(void) const
{
    return swapEndian(m_State.state.abc.time);
}

void Sunrise::SetABCTime(uint16_t value)
{
    m_State.state.abc.time = swapEndian(value);
}

bool Sunrise::GetCalibrationStatus(uint8_t& result) const
{
    return ReadRegister1(REG_CALIBRATION_STATUS, result);
}

bool Sunrise::ClearCalibrationStatus(void) const
{
    return WriteRegister1(REG_CALIBRATION_STATUS, 0x00, DELAY_SRAM);
}

bool Sunrise::GetCalibrationCommand(calibration_t& result) const
{
    uint16_t tmpResult;
    bool status = ReadRegister2(REG_CALIBRATION_COMMAND_MSB, tmpResult);
    if(status)
    {
        result = (calibration_t)tmpResult;
    }

    return status;
}

bool Sunrise::SetCalibrationCommand(calibration_t value, bool clearStatus) const
{
    if(clearStatus)
    {
        if(!ClearCalibrationStatus())
        {
            return false;
        }
    }

    return WriteRegister2(REG_CALIBRATION_COMMAND_MSB, value, DELAY_SRAM);
}

bool Sunrise::GetCalibrationTarget(uint16_t& result) const
{
    return ReadRegister2(REG_CALIBRATION_TARGET_MSB, result);
}

bool Sunrise::SetCalibrationTarget(uint16_t value) const
{
    return WriteRegister2(REG_CALIBRATION_TARGET_MSB, value, DELAY_SRAM);
}

bool Sunrise::GetCO2ValueOverride(uint16_t& result) const
{
    return ReadRegister2(REG_CO2_VALUE_OVERRIDE_MSB, result);
}

bool Sunrise::SetCO2ValueOverride(uint16_t value) const
{
    return WriteRegister2(REG_CO2_VALUE_OVERRIDE_MSB, value, DELAY_SRAM);
}

bool Sunrise::GetMeasurementModeEE(measurementmode_t& result) const
{
    uint8_t tmpResult;
    bool status = ReadRegister1(REG_MEASUREMENT_MODE, tmpResult);
    if(status)
    {
        result = (measurementmode_t)tmpResult;
    }

    return status;
}

bool Sunrise::SetMeasurementModeEE(measurementmode_t value) const
{
    return WriteRegister1(REG_MEASUREMENT_MODE, value, DELAY_EEPROM);
}

bool Sunrise::GetMeasurementPeriodEE(uint16_t& result) const
{
    return ReadRegister2(REG_MEASUREMENT_PERIOD_MSB, result);
}

bool Sunrise::SetMeasurementPeriodEE(uint16_t value) const
{
    if(value < MEASUREMENT_PERIOD_MIN || value > MEASUREMENT_PERIOD_MAX)
    {
        return false;
    }

    return WriteRegister2(REG_MEASUREMENT_PERIOD_MSB, value, DELAY_EEPROM);
}

bool Sunrise::GetNumberOfSamplesEE(uint16_t& result) const
{
    return ReadRegister2(REG_NUMBER_OF_SAMPLES_MSB, result);
}

bool Sunrise::SetNumberOfSamplesEE(uint16_t value) const
{
    if(value < NUMBER_OF_SAMPLES_MIN || value > NUMBER_OF_SAMPLES_MAX)
    {
        return false;
    }

    return WriteRegister2(REG_NUMBER_OF_SAMPLES_MSB, value, DELAY_EEPROM);
}

bool Sunrise::GetABCPeriodEE(uint16_t& result) const
{
    return ReadRegister2(REG_ABC_PERIOD_MSB, result);
}

bool Sunrise::SetABCPeriodEE(uint16_t value) const
{
    if(value < ABC_PERIOD_MIN || value > ABC_PERIOD_MAX)
    {
        return false;
    }

    return WriteRegister2(REG_ABC_PERIOD_MSB, value, DELAY_EEPROM);
}

bool Sunrise::GetABCTargetEE(uint16_t& result) const
{
    return ReadRegister2(REG_ABC_TARGET_MSB, result);
}

bool Sunrise::SetABCTargetEE(uint16_t value) const
{
    return WriteRegister2(REG_ABC_TARGET_MSB, value, DELAY_EEPROM);
}

bool Sunrise::GetStaticIIRFilterParameterEE(uint8_t& result) const
{
    return ReadRegister1(REG_STATIC_IIR_FILTER_PARAMETER, result);
}

bool Sunrise::SetStaticIIRFilterParameterEE(uint8_t value) const
{
    if(value < STATIC_IIR_FILTER_PARAMETER_MIN || value > STATIC_IIR_FILTER_PARAMETER_MAX)
    {
        return false;
    }

    return WriteRegister1(REG_STATIC_IIR_FILTER_PARAMETER, value, DELAY_EEPROM);
}

bool Sunrise::GetMeterControlRawEE(uint8_t& result) const
{
    return ReadRegister1(REG_METER_CONTROL, result);
}

bool Sunrise::SetMeterControlRawEE(uint8_t value) const
{
    return WriteRegister1(REG_METER_CONTROL, value, DELAY_EEPROM);
}

bool Sunrise::GetMeterControlEE(metercontrol_t& result) const
{
    uint8_t tmpResult;
    bool status = GetMeterControlRawEE(tmpResult);
    if(status)
    {

        result.nrdy = (tmpResult & (1 << metercontrolindex_t::MCI_NRDY)) == 0U;
        result.abc = (tmpResult & (1 << metercontrolindex_t::MCI_ABC)) == 0U;
        result.static_iir = (tmpResult & (1 << metercontrolindex_t::MCI_STATIC_IIR)) == 0U;
        result.dynamic_iir = (tmpResult & (1 << metercontrolindex_t::MCI_DYNAMIC_IIR)) == 0U;
        result.pressure_compensation = (tmpResult & (1 << metercontrolindex_t::MCI_PRESSURE_COMPENSATION)) == 0U;
    }

    return status;
}

bool Sunrise::SetMeterControlEE(const metercontrol_t& value) const
{
    uint8_t tmpValue;
    if(!GetMeterControlRawEE(tmpValue))
    {
        return false;
    }

    tmpValue = (tmpValue & 0xE0) | (value.nrdy << metercontrolindex_t::MCI_NRDY) | (value.abc << metercontrolindex_t::MCI_ABC) | (value.static_iir << metercontrolindex_t::MCI_STATIC_IIR) | (value.dynamic_iir << metercontrolindex_t::MCI_DYNAMIC_IIR) | (value.pressure_compensation << metercontrolindex_t::MCI_PRESSURE_COMPENSATION);
    return SetMeterControlRawEE(tmpValue);
}

bool Sunrise::ChangeAddress(uint8_t value, bool restart)
{
    if(value == m_Address)
    {
        return true;
    }

    if(value < I2C_ADDRESS_MIN || value > I2C_ADDRESS_MAX)
    {
        return false;
    }

    if(!WriteRegister1(REG_MB_I2C_ADDRESS, value, DELAY_EEPROM))
    {
        return false;
    }

    bool status = true;
    if(restart)
    {
        status = Restart();
    }

    m_Address = value;
    return status;
}

bool Sunrise::GetFirmwareRevisionRaw(uint16_t& result) const
{
    return ReadRegister2(REG_FIRMWARE_REV_MSB, result);
}

bool Sunrise::GetFirmwareRevision(uint8_t& main, uint8_t& sub) const
{
    uint16_t tmpResult;
    if(!GetFirmwareRevisionRaw(tmpResult))
    {
        return false;
    }

    main = (tmpResult >> 8) & 0x00FF;
    sub = tmpResult & 0x00FF;
    return true;
}

bool Sunrise::GetSensorID(uint32_t& result) const
{
    uint8_t cmd[] = { REG_SENSOR_ID_MMSB };
    bool status = BeginCommand() && I2CWrite(m_Address, cmd, sizeof(cmd)) == wirestatus_t::WS_ACK && I2CRead(m_Address, &result, sizeof(result)) == sizeof(result);
    if(status)
    {
        result = swapEndian(result);
    }

    return status;
}

void Sunrise::Awake(void) const
{
    if(m_PinEN == SUNRISE_INVALID_PIN)
    {
        return;
    }

    digitalWrite(m_PinEN, HIGH);
    delay(DELAY_WAKEUP);
}

void Sunrise::Sleep(void) const
{
    if(m_PinEN == SUNRISE_INVALID_PIN)
    {
        return;
    }

    digitalWrite(m_PinEN, LOW);
}

bool Sunrise::Restart(void) const
{
    if(!WriteRegister1(REG_SCR, 0xFF, DELAY_SRAM))
    {
        return false;
    }

    delay(DELAY_WAKEUP);
    return true;
}

bool Sunrise::HardRestart(void) const
{
    if(m_PinEN == SUNRISE_INVALID_PIN)
    {
        return false;
    }

    Sleep();
    delay(1UL);
    Awake();

    return true;
}

bool Sunrise::Ping(void) const
{
    uint8_t tmpResult;
    if(!ReadRegister1(REG_MB_I2C_ADDRESS, tmpResult))
    {
        return false;
    }

    return tmpResult == m_Address;
}

bool Sunrise::Begin(bool readInitialState)
{
    Wire.begin();
    Wire.setClock(I2C_CLOCK_FREQUENCY);
    delay(1UL);

    if(!Ping())
    {
        return false;
    }

    if(readInitialState)
    {
        if(!ReadState())
        {
            return false;
        }
    }

    return true;
}

bool Sunrise::Begin(uint8_t pinEN, bool readInitialState)
{
    m_PinEN = pinEN;
    pinMode(m_PinEN, OUTPUT);

    Awake();
    bool status = Begin(readInitialState);
    Sleep();

    return status;
}

bool Sunrise::Begin(int pinEN, bool readInitialState)
{
    return Begin((uint8_t)pinEN, readInitialState);
}

Sunrise::Sunrise(uint8_t address) : m_Address(address) { }
