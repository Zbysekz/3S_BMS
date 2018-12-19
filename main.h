
#include <inttypes.h>
void _delay_s(int sec);
void setBit(volatile uint8_t *port, int bit);
void clearBit(volatile uint8_t *port, int bit);
uint8_t getBit(volatile uint8_t port, int bit);

void CellBalancing(void);
void ResetOptos();