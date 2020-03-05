/*
	Arduino Library for LTC2942
	
	Copyright (c) 2018 Macro Yau

	https://github.com/MacroYau/LTC2942-Arduino-Library
*/

#include "LTC2942.h"

LTC2942::LTC2942(uint8_t rSense) {
	_rSense = rSense;
	_prescalerM = 0xFF;
	_batteryCapacity = 5500; // Default value when M = 128
}

bool LTC2942::begin(TwoWire &wirePort) {
	// Wire.begin() should be called in the application code in advance
	_i2cPort = &wirePort;

	// Checks device ID
	uint8_t chipID = getStatus() >> A_CHIP_ID_OFFSET;
	if (chipID != CHIP_ID_LTC2942) {
		return false;
	}

	return true;
}

void LTC2942::startMeasurement() {
	uint8_t value = readByteFromRegister(REG_B_CONTROL);
	value &= SHUTDOWN_MASK;
	writeByteToRegister(REG_B_CONTROL, value);
}

void LTC2942::stopMeasurement() {
	uint8_t value = readByteFromRegister(REG_B_CONTROL);
	value |= 1;
	writeByteToRegister(REG_B_CONTROL, value);
}

uint8_t LTC2942::getStatus() {
	return readByteFromRegister(REG_A_STATUS);
}

uint16_t LTC2942::getRawAccumulatedCharge() {
	if (_prescalerM == 0xFF) {
		// Needs to obtain M from register B
		uint8_t value = readByteFromRegister(REG_B_CONTROL);
		value &= ~PRESCALER_M_MASK;
		_prescalerM = value >> B_PRESCALER_M_OFFSET;
		_prescalerM = 1 << _prescalerM;
	}
	uint16_t acr = readWordFromRegisters(REG_C_ACC_CHG_MSB);
	return acr;
}

uint16_t LTC2942::getRemainingCapacity() {
	uint16_t acr = getRawAccumulatedCharge();
	float fullRange = 65536 * ((float) _prescalerM / 128) * 0.085;
	float offset = fullRange - _batteryCapacity;
	//charge in mAh, multiplier of 50 is split to 5 and 10 to prevent unsigned long overflow
    return (uint16_t)(((uint32_t)acr * _num / _den) - _offset);	
}

/*
float LTC2942::getRemainingCapacity() {
	uint16_t acr = getRawAccumulatedCharge();
	float fullRange = 65536 * ((float) _prescalerM / 128) * 0.085;
	float offset = fullRange - _batteryCapacity;
	return (acr * ((float) _prescalerM / 128) * 0.085 * ((float) 50 / _rSense)) - offset; // mAh
}
*/

/*
 *  Note:
 *  1. Datasheet conversion formula divide by 65535, in this library we divide by 65536 (>> 16) to reduce computational load 
 *     this is acceptable as the difference is much lower than the resolution of LTC2942 voltage measurement (78mV)
 *  2. Return is in unsigned short and mV to prevent usage of float datatype, the resolution of LTC2942 voltage measurement (78mV), 
 *     floating point offset is acceptable as it is lower than the resolution of LTC2942 voltage measurement (78mV)
 */
uint16_t LTC2942::getVoltage(bool oneShot) {
	if (oneShot) {
		setADCMode(ADC_MODE_MANUAL_VOLTAGE);
		//for(uint16_t i=0; i<4000; ++i);  // Small delay to wait for the converstion to complete - Ideally it should be 10ms
		delay(10);
	}
	uint16_t value = readWordFromRegisters(REG_I_VOLTAGE_MSB);
	uint32_t vBat = ((uint32_t)value * LTC2942_FULLSCALE_VOLTAGE);	// FULLSCALE_VOLTAGE is in mV, to avoid using float datatype
    vBat >>= 16;
	return (uint16_t)vBat;	//mV
}

/*
 *  Note:
 *  1. Datasheet conversion formula divide by 65535, in this library we divide by 65536 (>> 16) to reduce computational load 
 *     this is acceptable as the difference is much lower than the resolution of LTC2942 temperature measurement (3 Celcius)
 *  2. Return is in short to prevent usage of float datatype, floating point offset is acceptable as it is lower than the resolution of LTC2942 voltage measurement (3 Celcius).
 *     Unit of 0.01 Celcius
 */
int16_t LTC2942::getTemperature(bool oneShot) {
	if (oneShot) {
		setADCMode(ADC_MODE_MANUAL_TEMP);
		//for(uint16_t i=0; i<4000; ++i);  // Small delay to wait for the converstion to complete - Ideally it should be 10ms
		delay(10);
	}
	uint16_t value = readWordFromRegisters(REG_M_TEMP_MSB);	
	uint32_t tBat = ((uint32_t)value * LTC2942_FULLSCALE_TEMPERATURE);
    tBat >>= 16;
    tBat -= 2731;  // Convert temperature from kelvin to degree celsius	
	return (int16_t)tBat;
}

void LTC2942::setADCMode(uint8_t mode) {
	if (mode > 0b11) {
		return;
	}

	uint8_t value = readByteFromRegister(REG_B_CONTROL);
	value &= ADC_MODE_MASK;
	value |= (mode << B_ADC_MODE_OFFSET);
	writeByteToRegister(REG_B_CONTROL, value);
}

void LTC2942::setPrescalerM(uint8_t m) {
	if (m < 1 || m > 128) {
		return;
	}

	// Updates instance variable to avoid unnecessary access to register
	_prescalerM = m;
	m = findExponentOfPowerOfTwo(m);

	uint8_t value = readByteFromRegister(REG_B_CONTROL);
	value &= PRESCALER_M_MASK;
	value |= (m << B_PRESCALER_M_OFFSET);
	writeByteToRegister(REG_B_CONTROL, value);
}

/*
// Credit to https://github.com/DelfiSpace/LTC2942/blob/master/LTC2942.cpp
void LTC2942::setBatteryCapacity(uint16_t mAh) {
	_batteryCapacity = mAh;		
    unsigned int k, a;
    uint8_t m = 7;
    for(k = 128; k > 1; k = k / 2) {
        a = 278524 * k / _rSense / 128;
        if (a < (2 * mAh)) {
            break;
        }
        m--;
    }

    _num = 87 * 50 * k;
    _den = 128 * _rSense;
    _offset = (64 * _num / _den) - mAh;
	setPrescalerM(m);
}
*/

void LTC2942::setBatteryCapacity(uint16_t mAh) {
	_batteryCapacity = mAh;
	float q = (float) mAh / 1000;
	uint8_t m = 23 * q;
	if (_rSense != 50) {
		m *= ((float) _rSense / 50);
	}
	if (m > 128) {
		m = 128;
	}
	m = roundUpToPowerOfTwo(m);
    _num = 87 * 50 * m;
    _den = 128 * _rSense;
    _offset = (64 * _num / _den) - mAh;	
	setPrescalerM(m);
}


void LTC2942::setBatteryToFull() {
	writeWordToRegisters(REG_C_ACC_CHG_MSB, 0xFFFF);
}

void LTC2942::setRawAccumulatedCharge(uint16_t charge) {
	writeWordToRegisters(REG_C_ACC_CHG_MSB, charge);
}

void LTC2942::setChargeThresholds(uint16_t high, uint16_t low) {
	writeWordToRegisters(REG_E_CHG_THR_H_MSB, high);
	writeWordToRegisters(REG_G_CHG_THR_L_MSB, low);
}

void LTC2942::setVoltageThresholds(uint16_t high, uint16_t low) {
	writeByteToRegister(REG_K_VOLTAGE_THR_H, (uint8_t) (high / 23.4375));
	writeByteToRegister(REG_L_VOLTAGE_THR_L, (uint8_t) (low / 23.4375));
}

void LTC2942::setTemperatureThresholds(int16_t high, int16_t low) {
	writeByteToRegister(REG_M_TEMP_MSB, (uint8_t) ((high + 2731) / 23.4375));
	writeByteToRegister(REG_N_TEMP_LSB, (uint8_t) ((low + 2731) / 23.4375));
}

void LTC2942::configureALCC(uint8_t mode) {
	if (mode >= ALCC_MODE_NOT_ALLOWED) {
		return;
	}

	uint8_t value = readByteFromRegister(REG_B_CONTROL);
	value &= ALCC_CONFIG_MASK;
	value |= (mode << B_ALCC_CONFIG_OFFSET);
	writeByteToRegister(REG_B_CONTROL, value);
}

uint8_t LTC2942::findExponentOfPowerOfTwo(uint8_t value) {
	if (value > 64) {
		return 7;
	}

	for (uint8_t i = 0; i < 7; i++) {
		if ((value >> i) & 1) {
			return i;
		}
	}
}

uint8_t LTC2942::roundUpToPowerOfTwo(uint8_t value) {
	// Reference: https://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
	value--;
	value |= value >> 1;
	value |= value >> 2;
	value |= value >> 4;
	value++;
	return value;
}

uint16_t LTC2942::readWordFromRegisters(uint8_t msbAddress) {
	uint16_t value = 0;
	uint8_t msb = 0;
	uint8_t lsb = 0;

	_i2cPort->beginTransmission(LTC2942_ADDRESS);
	_i2cPort->write(msbAddress);
	_i2cPort->endTransmission(false);

	_i2cPort->requestFrom(LTC2942_ADDRESS, 2);
	msb = _i2cPort->read();
	lsb = _i2cPort->read();
	_i2cPort->endTransmission();
	value = (msb << 8) | lsb;

	return value;
}

bool LTC2942::writeWordToRegisters(uint8_t msbAddress, uint16_t value) {
	_i2cPort->beginTransmission(LTC2942_ADDRESS);
	_i2cPort->write(msbAddress);
	_i2cPort->write((uint8_t) (value >> 8));
	_i2cPort->write((uint8_t) value);
	return (_i2cPort->endTransmission() == 0);
}

uint8_t LTC2942::readByteFromRegister(uint8_t address) {
	uint8_t value = 0;

	_i2cPort->beginTransmission(LTC2942_ADDRESS);
	_i2cPort->write(address);
	_i2cPort->endTransmission(false);

	_i2cPort->requestFrom(LTC2942_ADDRESS, 1);
	value = _i2cPort->read();
	_i2cPort->endTransmission();

	return value;
}

bool LTC2942::writeByteToRegister(uint8_t address, uint8_t value) {
	_i2cPort->beginTransmission(LTC2942_ADDRESS);
	_i2cPort->write(address);
	_i2cPort->write(value);
	return (_i2cPort->endTransmission() == 0);
}

void LTC2942::resetAlert() {
	_i2cPort->requestFrom(LTC2942_ARA_ADR, 1);
	_i2cPort->read();
	_i2cPort->endTransmission();
}