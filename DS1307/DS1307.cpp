/*
* Maurice Ribble
* 4-17-2008
* http://www.glacialwanderer.com/hobbyrobotics
* This code tests the DS1307 Real Time clock on the Arduino board.
* The ds1307 works in binary coded decimal or BCD.  You can look up
* bcd in google if you aren't familior with it.  There can output
* a square wave, but I don't expose that in this code.  See the
* ds1307 for it's full capabilities.
*
* Add Wire.begin(); to main setup function when ready to use.
*
* Modified 9/12/2009 by Arclight for C library use.
* arclight@gmail.com 
*/

#include <DS1307.h>
#include <Wire.h>


DS1307::DS1307(){

}

DS1307::~DS1307(){
}


// Convert normal decimal numbers to binary coded decimal
byte DS1307::decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte DS1307::bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Stops the DS1307, but it has the side effect of setting seconds to 0
// Probably only want to use this for testing

/*
void DS1307::stopDs1307()
{
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.write(0x80);
  Wire.endTransmission();
}
*/

// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers

void DS1307::setDateDs1307(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
{
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   Wire.write(0);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   Wire.endTransmission();
}

// Gets the date and time from the ds1307
void DS1307::getDateDs1307(byte *second,
          byte *minute,
          byte *hour,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *month,
          byte *year)
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
}

/*
  // Change these values to what you want to set your clock to.
  // You probably only want to set your clock once and then remove
  // the setDateDs1307 call.
byte   second = 20;
byte   minute = 9;
byte   hour = 19;
byte   dayOfWeek = 6;
byte   dayOfMonth = 12;
byte   month = 9;
byte   year = 9;
setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);
*/
