#define SERIAL_TX_H

unsigned char tx_buffer[] = {'a', 'b', 'c', 'd', 10};
unsigned char tx_index = 0;
unsigned char tx_length = sizeof(tx_buffer)/sizeof(tx_buffer[0]);

void tx_fun(void);

