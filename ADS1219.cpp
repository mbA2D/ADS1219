#include <ADS1219.h>
#include "Arduino.h"

ADS1219::ADS1219()
{
	_set_reg_defaults();
	_offset = 0;
	_ext_ref_v = 0;
}

void ADS1219::init(uint8_t address, TwoWire *i2c)
{
	_addr = address;
	_i2c = i2c;
	
	//check if i2c->begin has already been called.
	if (TWCR == 0)
	{
		_i2c->begin();
	}
	_i2c->setWireTimeout(3000, true);
	
	_set_reg_defaults();
}

void ADS1219::reset()
{
	_write_command_byte(ADS1219_COMMAND_RESET);
	_set_reg_defaults();
}

void ADS1219::set_conf_reg(uint8_t conf_byte)
{
	_conf_reg.conf_byte = conf_byte;
	_write_conf_reg();
}

void ADS1219::set_mux(uint8_t mux)
{
	if (mux <= ADS1219_MUX_P_AVDD_2_N_AVDD_2)
	{
		_conf_reg.bits.mux = mux;
		_write_conf_reg();
	}
}

void ADS1219::set_gain(uint8_t gain)
{
	if (gain <= ADS1219_GAIN_4)
	{
		_conf_reg.bits.gain = gain;
		_write_conf_reg();
	}
}

void ADS1219::set_dr(uint8_t dr)
{
	if (dr <= ADS1219_DR_1000SPS)
	{
		_conf_reg.bits.dr = dr;
		_write_conf_reg();
	}
}

void ADS1219::set_cm(uint8_t cm)
{
	if (cm <= ADS1219_CM_CONTINUOUS)
	{		
		_conf_reg.bits.cm = cm;
		_write_conf_reg();
	}
}

void ADS1219::set_vref(uint8_t vref)
{
	if (vref <= ADS1219_VREF_EXTERNAL)
	{
		_conf_reg.bits.vref = vref;
		_write_conf_reg();
	}
}

void ADS1219::calibrate_offset()
{
	//set ain to mid-supply (don't change other settings)
	set_mux(ADS1219_MUX_P_AVDD_2_N_AVDD_2);
	
	int32_t min = 0;
	int32_t max = 0;
	int32_t sum = 0;
	//get num_measurements values
	for (uint8_t i = 0; i < ADS1219_NUM_MEASUREMENTS_FOR_OFFSET_CAL; i++)
	{
		_calibration_readings[i] = 0;
		
		int32_t reading = trigger_and_get_single_shot_result();
		//Serial.println(reading);

		//drop high and low as we read the values;
		//average the rest
		//everything should be close to 0 so taking the sum into a 32-bit int should be fine (no overflow).
		if (reading > max || max == 0)
		{
			_calibration_readings[i] = max;
			max = reading;
		}
		else if (reading < min || min == 0)
		{
			_calibration_readings[i] = min;
			min = reading;
		}
		else
		{
			_calibration_readings[i] = reading;
		}
		sum += _calibration_readings[i];
	}
	
	_offset = sum / (ADS1219_NUM_MEASUREMENTS_FOR_OFFSET_CAL - 2);
	//Serial.print("Offset: ");
	//Serial.println(_offset);
}

void ADS1219::trigger_conversion()
{
	_write_command_byte(ADS1219_COMMAND_START_SYNC);
}

void ADS1219::shut_down()
{
	_write_command_byte(ADS1219_COMMAND_POWERDOWN);
}

int32_t ADS1219::trigger_and_get_single_shot_result()
{
	//assuming already in single-shot mode (would also work in continuous mode
	trigger_conversion();
	while (!get_drdy())
	{
		delayMicroseconds(1100); //max sample rate is 1000SPS. add a little headroom for the delay.
	}
	return get_conversion_result();
}

int32_t ADS1219::get_conversion_result()
{
	uint8_t byte1, byte2, byte3;
	int32_t result = 0;
	
	_write_command_byte(ADS1219_COMMAND_RDATA);
	
	uint8_t response_length = 3;
	_wire_request_from(response_length);
	byte1 = _i2c->read();
	byte2 = _i2c->read();
	byte3 = _i2c->read();

	result = (result | byte1) << 8;
	result = (result | byte2) << 8;
	result = (result | byte3);
	result = result - _offset;

	return result;
}

bool ADS1219::get_drdy()
{
	_write_command_byte(ADS1219_COMMAND_RREG_STATUS);
	
	uint8_t response_length = 1;
	//Serial.println("Request");
	_wire_request_from(response_length);
	//_i2c->requestFrom(_addr, response_length);
	//uint8_t bytes_read = _i2c->requestFrom(_addr, response_length);
	//Serial.print("Read: ");
	//Serial.print(bytes_read);
	//Serial.println(" bytes");
	_status_reg.status_byte = _i2c->read();
	
	//Serial.println(_status_reg.status_byte, BIN);
	
	return bool(_status_reg.bits.drdy);
}

void ADS1219::set_ext_ref_v_for_calc(float ref_v)
{
	_ext_ref_v = ref_v;
}

float ADS1219::measure_voltage()
{
	//this function should return the voltage at the pins of the ADC.
	
	//vref
	float ref_v;
	if(_conf_reg.bits.vref == ADS1219_VREF_INTERNAL) ref_v = ADS1219_VOLTAGE_REFERENCE_V;
	else ref_v = _ext_ref_v;

	//gain
	uint8_t gain;
	if (_conf_reg.bits.gain == ADS1219_GAIN_1) gain = 1;
	else gain = 4;
	
	int32_t reading = trigger_and_get_single_shot_result();
	//Serial.print("Reading Before Scaling: ");
	//Serial.println(reading, HEX);
	
	return ((float(reading) / float(ADS1219_FULL_SCALE_POS_CODE)) * (ref_v / gain));

}

void ADS1219::_set_reg_defaults()
{
	//set everything back to 0 (default)
	_conf_reg.conf_byte = 0;
	_status_reg.status_byte = 0;
}

void ADS1219::_write_command_byte(uint8_t command_byte)
{
	_i2c->beginTransmission(_addr);
	_i2c->write(command_byte);
	_i2c->endTransmission(true);
}

void ADS1219::_write_conf_reg()
{
	_i2c->beginTransmission(_addr);
	_i2c->write(ADS1219_COMMAND_WREG);
	_i2c->write(_conf_reg.conf_byte);
	_i2c->endTransmission(true);
}

uint8_t ADS1219::read_conf_reg()
{
	return _read_conf_reg();
}

uint8_t ADS1219::_read_conf_reg()
{
	_write_command_byte(ADS1219_COMMAND_RREG_CONF);
	uint8_t response_length = 1;
	_wire_request_from(response_length);
	//_i2c->requestFrom(_addr, response_length);
	return uint8_t(_i2c->read());
}

uint8_t ADS1219::_wire_request_from(uint8_t num_bytes)
{
	uint8_t num_bytes_read = 0;
	num_bytes_read = _i2c->requestFrom(_addr, num_bytes);
	uint8_t timeout_count = 0;
	while (timeout_count < 10 && _i2c->getWireTimeoutFlag())
	{
		//if a timeout occurred, and there were less than 10 timeouts, then try again.
		_i2c->clearWireTimeoutFlag();
		num_bytes_read = _i2c->requestFrom(_addr, num_bytes);
		timeout_count++;
	}
	return num_bytes_read;
}
