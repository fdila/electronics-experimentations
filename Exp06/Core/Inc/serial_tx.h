#define SERIAL_TX_H
#include <stdint.h>

extern uint16_t tx_index;

uint8_t tx_fun(uint16_t* buffer, uint16_t length);

