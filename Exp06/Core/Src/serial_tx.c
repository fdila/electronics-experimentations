#include <stdint.h>
#include <main.h>

uint16_t tx_index = 0;

uint8_t tx_fun(uint16_t* tx_buffer, uint16_t tx_length){
	
	tx_length = tx_length*2;
	uint8_t* pointer = (uint8_t*) tx_buffer;
	uint8_t to_tx = *(pointer + tx_index);
	uint8_t is_finished = 1;
	
	if(tx_index < tx_length){
			USART3->DR = to_tx;
	} else {
		tx_index = 0;
		USART3->CR1 &= ~USART_CR1_TXEIE;
		is_finished = 0;
	}
		
	if (tx_index < tx_length)
		tx_index++;
	
	return is_finished;
}



