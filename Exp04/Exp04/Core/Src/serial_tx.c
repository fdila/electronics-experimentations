#include <stdint.h>

uint16_t tx_buffer[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
uint8_t tx_index = 0;
uint8_t tx_length = sizeof(tx_buffer);

uint8_t tx_fun(void){
	
	uint8_t* pointer = (uint8_t*) tx_buffer;
	uint8_t to_tx = *(pointer + tx_index);
	
	if (tx_index < tx_length)
		tx_index++;
	
	return to_tx;
}



