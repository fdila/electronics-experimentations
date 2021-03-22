#define SERIAL_TX_H
#include <stdint.h>

extern uint8_t tx_index;
extern uint8_t tx_length;
extern uint8_t tx_buffer;

uint8_t tx_fun(void);

