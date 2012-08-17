#ifndef	_PCATTACH_H_ 
#define	_PCATTACH_H_ 
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#define uint_8 byte

class PCATTACH {

public:
PCATTACH();
~PCATTACH();



void PCattachInterrupt(byte pin, void (*userFunc)(void), int mode);
void PCdetachInterrupt(byte pin);


static void PCint(uint8_t);

/*
volatile uint8_t *port_to_pcmask[];
typedef void (*voidFuncPtr)(void);
volatile static voidFuncPtr PCintFunc[];
volatile static uint8_t PCintLast[];
*/


};

