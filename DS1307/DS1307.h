/* Adds support for the DS1307 real-time clock chip (RTC)
*/

#ifndef	_DS1307_H_
#define	_DS1307_H_
#endif

#ifndef	_Wire_H_
#define	_Wire_H_
#endif

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DS1307_I2C_ADDRESS 0x68
  

extern byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;

class DS1307 {
public:
DS1307();
~DS1307();

void setDateDs1307(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year);          // 0-99

void getDateDs1307(byte *second,
          byte *minute,
          byte *hour,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *month,
          byte *year);

private:
byte decToBcd(byte val);
byte bcdToDec(byte val);

};
