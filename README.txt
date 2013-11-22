/* Open Access Control for Hacker Spaces - Ethernet Branch
 * Branch last modified by HeatSync Labs
 * https://github.com/zyphlar/Open_Access_Control_Ethernet
 * by Will Bradley
 * 
 * Upstream created by 23B Shop Hacker Space
 * http://blog.shop.23b.org
 * by John Norman and Dan Lozano
 * Readme updated 8/8/2012
*/

For the latest version of THIS BRANCH, visit:
https://github.com/zyphlar/Open_Access_Control_Ethernet

For the latest version of the UPSTREAM software, visit:
http://code.google.com/p/open-access-control/

Note: Unpack the libraries (WIEGAND26, DS1307, PCATTACH) into your arduino
libraries directory.  This is usually:

~/arduino/hardware/libraries/
(restart the Arduino program afterwards to apply the changes.)

The hardware design for the Open Access Control for Hacker Spaces uses the Arduino 
board with Atmega 328. It has been tested with:

-Arduino Uno
-Arduino Duemilanove
-Freeduino through hole (NKC)

The software has the following features:
-Shield compatible with Arduino
-Designed for use with Wiegand26 format readers (up to 3 support in software)
-DS1307 Real-time clock support
-(2) Wiegand26 reader inputs (optoisolated)
-(4) Alarm zone monitor ports using Analog0..3
-(4) Relay outputs


USAGE:

The following pin assignments are assumed by this software:

-Pins 2,3 for Reader 1
-Pins 4,5 for Reader 2
-Pins 5,6,7,8 for Relays
-Pins A0,A1,A2,A3 for alarm sensors
-Pins A4,A5 for SDA,SCL (I2C)

You'll need to adjust the IPAddress and the PRIVPASSWORD to fit your environment/preferences.


(g*32767+G-1)/2 
