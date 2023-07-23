#ifndef ADS1219_h
#define ADS1219_h

#include "Wire.h"

#define ADS1219_VOLTAGE_REFERENCE_V					2.048
#define ADS1219_FULL_SCALE_POS_CODE					0x7FFFFF //2^23
#define ADS1219_FULL_SCALE_NEG_CODE					0x800000 //-(2^23)
#define ADS1219_NUM_MEASUREMENTS_FOR_OFFSET_CAL		5

//############# COMMAND BYTES ###############
#define ADS1219_COMMAND_RESET		0x06
#define ADS1219_COMMAND_START_SYNC	0x08
#define ADS1219_COMMAND_POWERDOWN	0x02
#define ADS1219_COMMAND_RDATA		0x10
#define ADS1219_COMMAND_RREG_CONF	0x20 //read configuration register
#define ADS1219_COMMAND_RREG_STATUS	0x24 //read status register
#define ADS1219_COMMAND_WREG		0x40 //writes to configuration register

//############# REGISTER VALUES ##############
#define ADS1219_MUX_P_AIN0_N_AIN1		0
#define ADS1219_MUX_P_AIN2_N_AIN3		1
#define ADS1219_MUX_P_AIN1_N_AIN2		2
#define ADS1219_MUX_P_AIN0_N_AGND		3
#define ADS1219_MUX_P_AIN1_N_AGND		4
#define ADS1219_MUX_P_AIN2_N_AGND		5
#define ADS1219_MUX_P_AIN3_N_AGND		6
#define ADS1219_MUX_P_AVDD_2_N_AVDD_2	7

#define ADS1219_GAIN_1			0
#define ADS1219_GAIN_4			1

#define ADS1219_DR_20SPS		0
#define ADS1219_DR_90SPS		1
#define ADS1219_DR_330SPS		2
#define ADS1219_DR_1000SPS		3

#define ADS1219_CM_SINGLE_SHOT	0
#define ADS1219_CM_CONTINUOUS	1

#define ADS1219_VREF_INTERNAL 	0
#define ADS1219_VREF_EXTERNAL	1

#define ADS_1219_DRDY_FALSE		0
#define ADS_1219_DRDY_TRUE		1

typedef union _ads1219_conf_reg
{
	struct
	{
		uint8_t vref: 1; //bit 0
		uint8_t cm: 1; // bit 1
		uint8_t dr: 2; // bit 2-3
		uint8_t gain: 1; // bit 4
		uint8_t mux: 3; // bit 5-7
	}bits;
	uint8_t conf_byte;
}ads1219_conf_reg;

typedef union _ads1219_status_reg
{
	struct
	{
		uint8_t res: 7; //reserved
		uint8_t drdy: 1;
	}bits;
	uint8_t status_byte;
}ads1219_status_reg;


class ADS1219
{
	public:
		ADS1219();
	
		void init(uint8_t address, TwoWire *i2c = &Wire);
		void reset();
		
		void set_conf_reg(uint8_t conf_byte);
		void set_mux(uint8_t mux);
		void set_gain(uint8_t gain);
		void set_dr(uint8_t dr);
		void set_cm(uint8_t cm);
		void set_vref(uint8_t vref);
		uint8_t read_conf_reg();

		void calibrate_offset();
		void trigger_conversion(); //also wakes up the analog portions of the device
		void shut_down(); //shuts down analog portions of the device, stops continuous conversions
		
		int32_t trigger_and_get_single_shot_result();
		int32_t get_conversion_result();
		bool get_drdy();
		
		void set_ext_ref_v_for_calc(float ref_v);
		float measure_voltage(); //uses single-shot mode
	
	private:
		void _set_reg_defaults();
		void _write_command_byte(uint8_t command_byte);
		void _write_conf_reg();
		uint8_t _read_conf_reg();
		uint8_t _wire_request_from(uint8_t num_bytes);
		
		int32_t _calibration_readings[ADS1219_NUM_MEASUREMENTS_FOR_OFFSET_CAL];
		int32_t _offset;
		uint8_t _addr;
		TwoWire *_i2c;
		
		float _ext_ref_v;
		
		ads1219_conf_reg	_conf_reg;
		ads1219_status_reg	_status_reg;
	
};

#endif
