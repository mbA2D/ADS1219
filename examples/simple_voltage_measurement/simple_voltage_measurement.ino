#include <ADS1219.h>

ADS1219 adc;

void setup()
{
	Serial.begin(115200);
	Serial.println("ADC Starting");
	Serial.println("Single-ended read on all inputs");
	
	adc.init(0x40);
	adc.calibrate_offset();
	
	//use defaults for gain (1), cm (single-shot), dr (20SPS), vref (internal)
}


void loop()
{
	adc.set_mux(ADS1219_MUX_P_AIN0_N_AGND);
	Serial.print("AIN0: ");
	Serial.println(adc.measure_voltage());
	
	adc.set_mux(ADS1219_MUX_P_AIN1_N_AGND);
	Serial.print("AIN1: ");
	Serial.println(adc.measure_voltage());
	
	adc.set_mux(ADS1219_MUX_P_AIN2_N_AGND);
	Serial.print("AIN2: ");
	Serial.println(adc.measure_voltage());
	
	adc.set_mux(ADS1219_MUX_P_AIN3_N_AGND);
	Serial.print("AIN3: ");
	Serial.println(adc.measure_voltage());
	
	Serial.println("----------------");
}