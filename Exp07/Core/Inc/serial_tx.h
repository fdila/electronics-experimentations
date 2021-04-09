#define SERIAL_TX_H
#include <stdint.h>

extern volatile uint16_t tx_index;

uint8_t tx_fun(volatile uint16_t* buffer, uint16_t length);

