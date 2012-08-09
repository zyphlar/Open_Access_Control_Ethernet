#ifndef	_WIEGAND26_H_ 
#define	_WIEGAND26_H_ 
#endif

#include <WProgram.h>

class WIEGAND26 {
public:
WIEGAND26();
~WIEGAND26();


//const byte reader1Pins[]; // Reader 1 connected to pins 4,5
//const byte reader2Pins[];        // Reader2 connected to pins 6,7
//const byte reader3Pins[];      // Reader3 connected to pins X,Y

//volatile long reader1;
//volatile int  reader1Count;
//volatile long reader2;
//volatile int  reader2Count;
//volatile long reader3;
//volatile int  reader3Count;

void initReaderOne(void);
void initReaderTwo(void);
void reader1One(void);
void reader1Zero(void);
void reader2One(void);
void reader2Zero(void);
void initReaderThree(void);
void reader3One(void);
void reader3Zero(void);

private:

};
