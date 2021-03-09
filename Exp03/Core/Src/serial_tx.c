
extern unsigned char tx_index;
extern unsigned char tx_length;

void tx_fun(void){
	if (tx_index < tx_length)
		tx_index++;
}



