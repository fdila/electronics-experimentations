#include <stdint.h>

#define TCAL_30C *(uint16_t*)(0x1FFF7A2C)
#define TCAL_110C *(uint16_t*)(0x1FFF7A2E)
#define VBandGapC *(uint16_t*)(0x1FFF7A2A)

uint32_t get_temp(uint16_t adc_t_val, uint16_t vrefint){
	float temp;
	uint32_t VrefCal = (3300000/vrefint)*VBandGapC;
	return VrefCal;
}

