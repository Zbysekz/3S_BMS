
//#include <inttypes.h>
void _delay_s(int sec);
void setBit(volatile uint8_t *port, int bit);
void clearBit(volatile uint8_t *port, int bit);
uint8_t getBit(volatile uint8_t port, int bit);

void CellBalancing(void);
void ResetOptos();

void ReadCells();

extern volatile uint8_t goOff;
extern uint8_t currentState;
extern uint16_t cellA,cellB,cellC;
