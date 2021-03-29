#include <stdint.h>

#define TCAL_30C *(uint16_t*)(0x1FFF7A2C)
#define TCAL_110C *(uint16_t*)(0x1FFF7A2E)
#define VBandGapC *(uint16_t*)(0x1FFF7A2A)

float get_temp(uint16_t adc_t_val, uint16_t vrefint){
	float temp;
	
	
	return temp;
}

