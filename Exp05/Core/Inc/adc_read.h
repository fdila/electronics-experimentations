#include <stdint.h>

#define TCAL_30C *(uint16_t*)(0x1FFF7A2C)
#define TCAL_110C *(uint16_t*)(0x1FFF7A2E)
#define VBandGapC *(uint16_t*)(0x1FFF7A2A)

float get_temp(uint16_t adc_temp_val, uint16_t vrefint){

	uint32_t VrefCal = (3300000/vrefint)*VBandGapC;
	//uint32_t mv_30c = TCAL_30C * VrefCal / 3300000;
	uint32_t cod_per_degree = (((TCAL_110C - TCAL_30C)/80));
	float mv_per_degree = cod_per_degree * 3300000 / (2^12);
	
	uint32_t mv_temp = adc_temp_val*VrefCal / 2^12;
	
	float temp = mv_temp * mv_per_degree;
	
	return mv_per_degree;
}

