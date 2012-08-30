/*
 * Open Source RFID Access Controller - Ethernet Branch
 *
 * 8/8/2012 v0.01 (branch based on upstream 4/3/2011 v1.32)
 * Will Bradley - will@heatsynclabs.org
 * 
 * Upstream:
 * Last build test with Arduino v00.21
 * Arclight - arclight@23.org
 * Danozano - danozano@gmail.com
 *
 * Notice: This is free software and is probably buggy. Use it at
 * at your own peril.  Use of this software may result in your
 * doors being left open, your stuff going missing, or buggery by
 * high seas pirates. No warranties are expressed on implied.
 * You are warned.
 *
 * For latest downloads of this ETHERNET branch, check out
 * https://github.com/zyphlar/Open_Access_Control_Ethernet
 *
 * For latest downloads of the UPSTREAM software, including 
 * Eagle CAD files for the hardware, check out
 * http://code.google.com/p/open-access-control/downloads/list
 *
 *
 * This program interfaces the Arduino to RFID, PIN pad and all
 * other input devices using the Wiegand-26 Communications
 * Protocol. It is recommended that the keypad inputs be
 * opto-isolated in case a malicious user shorts out the 
 * input device.
 * Outputs go to a Darlington relay driver array for door hardware/etc control.
 * Analog inputs are used for alarm sensor monitoring.  These should be
 * isolated as well, since many sensors use +12V. Note that resistors of
 * different values can be used on each zone to detect shorting of the sensor
 * or wiring.
 *
 * Version 1.00+ of the hardware implements these features and uses the following pin 
 * assignments on a standard Arduino Duemilanova or Uno:
 *
 * Relay outpus on digital pins 6,7,8,9
 * DS1307 Real Time Clock (I2C):A4 (SDA), A5 (SCL)
 * Analog pins (for alarm):A0,A1,A2,A3 
 * Reader 1: pins 2,3
 * Reader 2: pins 4,5
 * Ethernet: pins 10,11,12,13 (Not connected to the board, reserved for the Ethernet shield)
 *
 * Quickstart tips: 
 * Set the privilege password(PRIVPASSWORD) value to a numeric DEC or HEX value.
 * Define the static user list by swiping a tag and copying the value received into the #define values shown below under Adam, Bob, and Carl.
 * Change MAC and IP as appropriate for your network.
 * Compile and upload the code, then log in via HTTP to the IP you specified.
 *
 * Guide to log keys and data:
 * A=alarm armed (# level)
 * a=added user (# usernum)
 * C=keypad command (# command)
 * c=second half
 * c=checked user (0=failed, #=found usernum)
 * D=denied access (# card num)
 * d=second half
 * d=deleted user (# usernum)
 * E=second (#=second)
 * F=priv fail (0=wrong pw, 1=too many attempts, 2=not logged in)
 * f=second half
 * f=card fail (#=usermask)
 * G=granted access (# card num)
 * g=second half of card
 * H=hour (#=hour)
 * i=attempt to write to invalid eeprom address (# usernum)
 * I=attempt to delete from invalid eeprom address (# usernum)
 * L=locked (1=door1, 2=door2, 3=bedtime)
 * M=minute (#=minute)
 * m=alarm state (# level)
 * R=read tag (# card num)
 * r=second half of tag
 * Q=superuser authed (#=superuser)
 * S=auth (0=privileged mode enabled)
 * s=alarm sensor (# zone)
 * t=alarm trained (#=sensor value)
 * T=alarm triggered (0)
 * U=unlocked door (1=door1, 2=door2, # card num)
 * u=second half of card
 * Z=user db cleared (0)
 * z=log cleared (0)
 */

#include <Wire.h>         // Needed for I2C Connection to the DS1307 date/time chip
#include <EEPROM.h>       // Needed for saving to non-voilatile memory on the Arduino.
#include <avr/pgmspace.h> // Allows data to be stored in FLASH instead of RAM


#include <Ethernet.h>     // Ethernet stuff
#include <SPI.h>          


#include <DS1307.h>       // DS1307 RTC Clock/Date/Time chip library
#include <WIEGAND26.h>    // Wiegand 26 reader format libary
#include <PCATTACH.h>     // Pcint.h implementation, allows for >2 software interupts.


/* Static user List - Implemented as an array for testing and access override 
 */                               

#define DEBUG 2                         // Set to 2 for display of raw tag numbers in log files, 1 for only denied, 0 for never.               

#define adam   0xABCDE                  // Name and badge number in HEX. We are not using checksums or site ID, just the whole
#define bob   0xBCDEF                  // output string from the reader.
#define carl   0xA1B2C3
const long  superUserList[] = { adam, bob, carl};  // Super user table (cannot be changed by software)

#define PRIVPASSWORD 0x1234             // Console "priveleged mode" password

#define DOORDELAY 5000                  // How long to open door lock once access is granted. (2500 = 2.5s)
#define SENSORTHRESHOLD 100             // Analog sensor change that will trigger an alarm (0..255)

#define EEPROM_ALARM 0                  // EEPROM address to store alarm triggered state between reboots (0..511)
#define EEPROM_ALARMARMED 1             // EEPROM address to store alarm armed state between reboots
#define EEPROM_ALARMZONES 20            // Starting address to store "normal" analog values for alarm zone sensor reads.
#define KEYPADTIMEOUT 5000              // Timeout for pin pad entry. Users on keypads can enter commands after reader swipe.

#define EEPROM_FIRSTUSER 24
#define EEPROM_LASTUSER 1024
#define NUMUSERS  ((EEPROM_LASTUSER - EEPROM_FIRSTUSER)/5)  //Define number of internal users (200 for UNO/Duemillanova)


#define DOORPIN1 relayPins[0]           // Define the pin for electrified door 1 hardware
#define DOORPIN2 relayPins[2]           // Define the pin for electrified door 2 hardware
#define ALARMSTROBEPIN relayPins[3]     // Define the "non alarm: output pin. Can go to a strobe, small chime, etc
#define ALARMSIRENPIN  relayPins[1]     // Define the alarm siren pin. This should be a LOUD siren for alarm purposes.

byte reader1Pins[]={2,3};               // Reader 1 connected to pins 4,5
byte reader2Pins[]= {4,5};              // Reader2 connected to pins 6,7

//byte reader3Pins[]= {10,11};                // Reader3 connected to pins X,Y (Not implemented on v1.x and 2.x Access Control Board)

const byte analogsensorPins[] = {0,1,2,3};    // Alarm Sensors connected to other analog pins
const byte relayPins[]= {6,7,8,9};            // Relay output pins

bool door1Locked=true;                        // Keeps track of whether the doors are supposed to be locked right now
bool door2Locked=true;

unsigned long door1locktimer=0;               // Keep track of when door is supposed to be relocked
unsigned long door2locktimer=0;               // after access granted.

boolean doorChime=false;                       // Keep track of when door chime last activated
boolean doorClosed=false;                      // Keep track of when door last closed for exit delay

unsigned long alarmDelay=0;                    // Keep track of alarm delay. Used for "delayed activation" or level 2 alarm.
unsigned long alarmSirenTimer=0;               // Keep track of how long alarm has gone off


unsigned long consolefailTimer=0;               // Console password timer for failed logins
byte consoleFail=0;
#define numUsers (sizeof(superUserList)/sizeof(long))                  //User access array size (used in later loops/etc)
#define NUMDOORS (sizeof(doorPin)/sizeof(byte))
#define numAlarmPins (sizeof(analogsensorPins)/sizeof(byte))

//Other global variables
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;     // Global RTC clock variables. Can be set using DS1307.getDate function.

byte alarmActivated = EEPROM.read(EEPROM_ALARM);                   // Read the last alarm state as saved in eeprom.
byte alarmArmed = EEPROM.read(EEPROM_ALARMARMED);                  // Alarm level variable (0..5, 0==OFF) 

boolean sensor[4]={false};                                         // Keep track of tripped sensors, do not log again until reset.
unsigned long sensorDelay[2]={0};                                  // Same as above, but sets a timer for 2 of them. Useful for logging
                                                                   // motion detector hits for "occupancy check" functions.

// Enable up to 3 door access readers.
volatile long reader1 = 0;
volatile int  reader1Count = 0;
volatile long reader2 = 0;
volatile int  reader2Count = 0;
int userMask1=0;
int userMask2=0;
boolean keypadGranted=0;                                       // Variable that is set for authenticated users to use keypad after login

//volatile long reader3 = 0;                                   // Uncomment if using a third reader.
//volatile int  reader3Count = 0;

unsigned long keypadTime = 0;                                  // Timeout counter for  reader with key pad
unsigned long keypadValue=0;


boolean privmodeEnabled = false;                               // Switch for enabling "priveleged" commands

// Log buffer
char logKeys[40]={0};
int logData[40]={0};
int logCursor=0;


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1,177);

// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
EthernetServer server(80);

/* Create an instance of the various C++ libraries we are using.
 */

DS1307 ds1307;        // RTC Instance
WIEGAND26 wiegand26;  // Wiegand26 (RFID reader serial protocol) library
PCATTACH pcattach;    // Software interrupt library

/* Set up some strings that will live in flash instead of memory. This saves our precious 2k of
 * RAM for something else.
*/


const prog_uchar httpheaderok[]   PROGMEM  = {"HTTP/1.1 200 OK\r\nCache-Control: no-store\r\nContent-Type: text/html\r\n\r\n"};
const prog_uchar title[]          PROGMEM  = {"<h2>OAC</h2>"};
const prog_uchar help[]           PROGMEM  = {"<hr/>See source for command syntax."};  //<pre>Numbers must be padded.\n\n?e=0000 - enable priv (0 to logout)\n?s000 - show user\n?m000&p000&t00000000 - modify user(0-200) perm(0-255) tag(0-f)\n?a - list all\n?r000 - remove user\n?o1 ?o2 - open door 1/2\n?u ?u=1 ?u=2 - unlock all/1/2\n?l - lock all\n?1 - disarm\n?2 - arm\n?3 - train\n?9 - status\n?z - show log\n?y - clear log</pre>"};    //\n?d=00&w=0&m=00&y=00&h=00&i=00&s=00 - set day-dayofweek-month-year-hour-min-sec
const prog_uchar noauth[]         PROGMEM  = {"<a href='/'>Not logged in.</a>"};
const prog_uchar unlockboth[]     PROGMEM  = {"Unlocked all."};
const prog_uchar unlock1[]        PROGMEM  = {"Unlocked 1."};
const prog_uchar unlock2[]        PROGMEM  = {"Unlocked 2."};
const prog_uchar open1[]          PROGMEM  = {"Opened 1."};
const prog_uchar open2[]          PROGMEM  = {"Opened 2."};
const prog_uchar lockboth[]       PROGMEM  = {"Locked all."};

const int divisor = 32767; 

void setup(){           // Runs once at Arduino boot-up

  // start the Ethernet connection and the server:
  Ethernet.begin(mac, ip);
  server.begin();

    Wire.begin();   // start Wire library as I2C-Bus Master

  /* Attach pin change interrupt service routines from the Wiegand RFID readers
   */
  pcattach.PCattachInterrupt(reader1Pins[0], callReader1Zero, CHANGE); 
  pcattach.PCattachInterrupt(reader1Pins[1], callReader1One,  CHANGE);  
  pcattach.PCattachInterrupt(reader2Pins[1], callReader2One,  CHANGE);
  pcattach.PCattachInterrupt(reader2Pins[0], callReader2Zero, CHANGE);

  //Clear and initialize readers
  wiegand26.initReaderOne(); //Set up Reader 1 and clear buffers.
  wiegand26.initReaderTwo(); 


  //Initialize output relays

  for(byte i=0; i<4; i++){        
    pinMode(relayPins[i], OUTPUT);                                                      
    digitalWrite(relayPins[i], LOW);                  // Sets the relay outputs to LOW (relays off)
  }


  ds1307.setDateDs1307(0,41,2,5,30,8,12);         
  /*  Sets the date/time (needed once at commissioning)
   
   byte second,        // 0-59
   byte minute,        // 0-59
   byte hour,          // 1-23
   byte dayOfWeek,     // 1-7
   byte dayOfMonth,    // 1-28/29/30/31
   byte month,         // 1-12
   byte year);          // 0-99
   */



//  Serial.begin(57600);	               	       // Set up Serial output at 8,N,1,57600bps
  
  
  logReboot();
  chirpAlarm(1);                               // Chirp the alarm to show system ready.

//  hardwareTest(100);                         // IO Pin testing routing (use to check your inputs with hi/lo +(5-12V) sources)
                                               // Also checks relays


}
void loop()                                     // Main branch, runs over and over again
{

  // listen for incoming clients
  EthernetClient client = server.available();
  
  String readString = String(100); //string for fetching data from address

  if (client) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        
        //read char by char HTTP request
        if (readString.length() < 100) {
          //store characters to string
          readString += c;
        }
        
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          PROGMEMprintln(client,httpheaderok);
          
          if(readString.indexOf("?e=") > 0 || readString.indexOf("&e=") > 0) { // login -- use e= to allow ?e and &e
            int offset = readString.indexOf("e=");
            char pass[5] = {readString[offset+2],readString[offset+3],readString[offset+4],readString[offset+5],'\0'};

            if(login(strtoul(pass,NULL,16))) {
              client.println("authok");  
            }
            else {
              client.println("authfail");
            }
          }
          if(readString.indexOf("?s") > 0) { // show user
            int offset = readString.indexOf("?s");
            char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};
            
            if(privmodeEnabled==true) {
              client.println("<pre>");
              client.print("UserNum:");
              client.print(" ");
              client.print("Usermask:");
              client.print(" ");
              client.println("TagNum:");
              dumpUser(client, atoi(usernum));
              client.println("</pre>");
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?m") > 0) { // modify user #, permission #, tag # (?m000&p000&t00000000 must be zero-padded)
            int offset = readString.indexOf("?m");  // user, 3 chars
            int initialoffset = offset; // save for comparison
            char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};

            offset = readString.indexOf("&p");  // permissions mask, 3 chars
            char usermask[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};

            offset = readString.indexOf("&t");  // tag, 8 chars
            char usertag[9] = {readString[offset+2],readString[offset+3],readString[offset+4],readString[offset+5],
                               readString[offset+6],readString[offset+7],readString[offset+8],readString[offset+9],'\0'};
            
            if(offset-initialoffset == 10){
              if(privmodeEnabled==true) {
                client.println("<pre>");
                client.println("prev:");
                dumpUser(client, atoi(usernum));
                addUser(atoi(usernum), atoi(usermask), strtoul(usertag,NULL,16));
                client.println("cur:");
                dumpUser(client, atoi(usernum));
                client.println("</pre>");
              }
              else{
                PROGMEMprintln(client,noauth);
                logprivFail();
              } 
            }
            else {
              client.println("err:query");
            }
          }
          if(readString.indexOf("?a") > 0) {  //list all users
            if(privmodeEnabled==true) {
              //logDate();
              client.println("<pre>");
              client.print("UserNum:");
              client.print(" ");
              client.print("Usermask:");
              client.print(" ");
              client.println("TagNum:");
              for(int i=0; i<(NUMUSERS); i++){
                dumpUser(client,i);
              }
              client.println("</pre>");
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?r") > 0) {  //remove user (?r000)
            int offset = readString.indexOf("?r");
            char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};
            
            if(privmodeEnabled==true) {
              client.println("r");           
              client.println("<pre>");
              client.println("prev:");
              dumpUser(client,atoi(usernum));
              deleteUser(atoi(usernum));
              client.println("cur:");
              dumpUser(client,atoi(usernum));
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?o") > 0) {  // open door ?o1 or ?o2
            int offset = readString.indexOf("?o");
            char doornum[2] = {readString[offset+2],'\0'};
  
            if(privmodeEnabled==true) {
              if(atoi(doornum) == 1){  
                alarmState(0);                                       // Set to door chime only/open doors                                                                       
                armAlarm(4);
                doorUnlock(1);                                       // Open the door specified
                door1locktimer=millis();
                PROGMEMprintln(client,open1);
              }                    
              else{
                if(atoi(doornum) == 2){  
                  alarmState(0);                                       // Set to door chime only/open doors                                                                       
                  armAlarm(4);
                  doorUnlock(2);                                        
                  door2locktimer=millis();
                  PROGMEMprintln(client,open2);
                }
                else {
                  client.println("err:door#");
                }
              }
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?u") > 0) {  //unlock (?u or ?u=1 or ?u=2)
            if(privmodeEnabled==true) {
              int offset = readString.indexOf("?u="); // see if we're unlocking a specific door
              if(offset > 0) {
                char doornum[2] = {readString[offset+3],'\0'};
                if(atoi(doornum) == 1){ 
                  doorUnlock(1);
                  alarmState(0);
                  armAlarm(4);
                  door1Locked=false;
                  chirpAlarm(3);
                  PROGMEMprintln(client,unlock1);
                }
                else {
                  if(atoi(doornum) == 2){ 
                    doorUnlock(2);
                    alarmState(0);
                    armAlarm(4);
                    door2Locked=false;
                    chirpAlarm(3);          
                    PROGMEMprintln(client,unlock2);
                  }
                  else {
                    client.println("err:door#");
                  }
                }
              }
              else {  // not unlocking a specific door; unlock all.
                PROGMEMprintln(client,unlockboth);
                unlockall();
              }
              printStatus(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?l") > 0) {  //lock
            if(privmodeEnabled==true) {
              lockall();
              chirpAlarm(1); 
              PROGMEMprintln(client,lockboth);  
              printStatus(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?1") > 0) {  // disarm
            if(privmodeEnabled==true) {
              armAlarm(0);
              alarmState(0);
              chirpAlarm(1);  
              printStatus(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?2") > 0) { // arm
            if(privmodeEnabled==true) {
              chirpAlarm(20);        // 200 chirps = ~30 seconds delay
              armAlarm(1);                           
              printStatus(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?3") > 0) { // train
            if(privmodeEnabled==true) {
              trainAlarm();
              printStatus(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?9") > 0) { // status
            printStatus(client);
          }
          if(readString.indexOf("?z") > 0) { // log
            if(privmodeEnabled==true) {
              printLog(client);
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?y") > 0) { // clear log
            if(privmodeEnabled==true) {
              for(int i=0;i<sizeof(logKeys);i++) {
                logKeys[i] = 0;
                logData[i] = 0;
              }
              logCursor = 0;
              addToLog('z',0);
              //logDate();
              
              client.println("y");
            }
            else{
              PROGMEMprintln(client,noauth);
              logprivFail();
            }
          }
          if(readString.indexOf("?") < 0) {
            PROGMEMprintln(client,title);
            PROGMEMprintln(client,help);
          }
          if(readString.indexOf("&e=") > 0) { // if e is passed as a second parameter, log out.
            login(strtoul("0000",NULL,16)); // 0000 = logout
          }

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
  }
  
  
  /* Check if doors are supposed to be locked and lock/unlock them 
   * if needed. Uses global variables that can be set in other functions.
   */

  if(((millis() - door1locktimer) >= DOORDELAY) && (door1locktimer !=0))
  { 
    if(door1Locked==true){
     doorLock(1);
     door1locktimer=0;    }

    else {                        
      doorUnlock(1); 
      door1locktimer=0;
                        }                         
   }

  

 if(((millis() - door2locktimer) >= DOORDELAY) && (door2locktimer !=0))
  { 
    if(door2Locked==true) {
     doorLock(2); 
     door2locktimer=0;
                          }
   
    else {
     doorUnlock(2); 
     door2locktimer=0;
                         }   
  }   

  /*  Set optional "failsafe" time to lock up every night.
  */

  ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);   // Get the current date/time

  if(hour==23 && minute==59 && door1Locked==false){
         doorLock(1);
         door1Locked==true;      
         addToLog('L',3);
  }


  // Notes: RFID polling is interrupt driven, just test for the reader1Count value to climb to the bit length of the key
  // change reader1Count & reader1 et. al. to arrays for loop handling of multiple reader output events
  // later change them for interrupt handling as well!
  // currently hardcoded for a single reader unit

  /* This code checks a reader with a 26-bit keycard input. Use the second routine for readers with keypads.  
   * A 5-second window for commands is opened after each successful key access read.
   */

  if(reader1Count >= 26){                           //  When tag presented to reader1 (No keypad on this reader)
    logTagPresent(reader1,1);                       //  write log entry to serial port


/* Check a user's security level and take action as needed. The
*  usermask is a variable from 0..255. By default, 0 and 255 are for
*  locked out users or uninitialized records.
*  Modify these for each door as needed.
*/

  userMask1=checkUser(reader1);    

  if(userMask1>=0) {    
    
   switch(userMask1) {

   case 0:                                      // No outside privs, do not log denied.
    {                                           // authenticate only.
    logAccessGranted(reader1, 1);
    break;
    }

   case 20:                                                // Example Limited hours user
    {                                                      // Can enter from 5:00pm to 11:00pm
    ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    
    if((hour >=17) && (hour <=23)){
         logAccessGranted(reader1, 1);                    // Log and unlock door 2
         alarmState(0);
         armAlarm(0);                                     //  Deactivate Alarm
      //   chirpAlarm(1);                            
         door1locktimer=millis();
         doorUnlock(1);                                   // Unlock the door.
    }
     break;
    }      

   case 255:                                              // Locked out user     
    {
     addToLog('f',userMask1);
     break;
    }
   
   default:  
    {            
         logAccessGranted(reader1, 1);           // Log and unlock door 1
         alarmState(0);
         armAlarm(0);                            //  Deactivate Alarm                  
         door1locktimer=millis();
         doorUnlock(1);                          // Unlock the door.
         break;
    }
                       }                                      

  }
    else 
    {                                           
    if(checkSuperuser(reader1) >= 0) {              // Check if a superuser, grant access.
      logAccessGranted(reader1, 1);                 // Log and unlock door 1
         alarmState(0);
         armAlarm(0);                               //  Deactivate Alarm
         door1locktimer=millis();
         doorUnlock(1);                             // Unlock the door.
                                      }
      else{                                
       logAccessDenied(reader1,1);                   // No tickee, no laundree
          }
    }

    wiegand26.initReaderOne();                     // Reset for next tag scan

  }



  
  if(reader2Count >= 26){                                // Tag presented to reader 2
    logTagPresent(reader2,2);                            // Write log entry to serial port
    chirpAlarm(1);                                       // Chirp alarm to show that tag input done              
                                                         // CHECK TAG IN OUR LIST OF USERS. -1 = no match                                  
  keypadGranted=false;                                   // Reset the keypad authorized variable

  userMask2=checkUser(reader2);    

  if(userMask2>=0){    
    switch(userMask2) {
 
   case 0:                         // No outside privs, do not log denied.
    {                              // authenticate and log only.
    logAccessGranted(reader2, 2);
    break;
    }
      
  case 10:                         // Authenticating immediately locks up and arms alarm
    {                              // 
    logAccessGranted(reader2, 2);
    runCommand(0x2);
    break;
    }
    
   case 20:                                               //Limited hours user
    {
    ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    
    if((hour >=17) && (hour <=23)){
         logAccessGranted(reader2, 2);                    // Log and unlock door 2
         alarmState(0);
         armAlarm(0);                                     //  Deactivate Alarm                           
         door2locktimer=millis();
         doorUnlock(2);                                   // Unlock the door.
         keypadGranted=1;
    }
     break;
    }
  
   case 255:                                               // Locked out      
    {
     addToLog('f',userMask2);
     break;
    }
    
    default:  
    {            
         logAccessGranted(reader2, 2);           // Log and unlock door 2
         alarmState(0);
         armAlarm(0);                            //  Deactivate Alarm                          
         door2locktimer=millis();
         doorUnlock(2);                          // Unlock the door.
         keypadGranted=1;
         break;
    }
                 }                                      

  }
    else 
    {                                             
     if(checkSuperuser(reader2) >= 0) {              // Check if a superuser, grant access.
      logAccessGranted(reader2, 2);                 // Log and unlock door 2
         alarmState(0);
         armAlarm(0);                              //  Deactivate Alarm
         chirpAlarm(1);                            
         door1locktimer=millis();
         doorUnlock(1);                            // Unlock the door.
         keypadGranted=1;
                                      }
      else{                                
      logAccessDenied(reader2,2);                 //  no tickee, no laundree
          }
    }
    
    wiegand26.initReaderTwo();                   //  Reset for next tag scan
    unsigned long keypadTime=0;                  //  Timeout counter for  reader with key pad
    long keypadValue=0;
    keypadTime=millis();  
                                         
   if(keypadGranted==1) 
    {
      while((millis() - keypadTime)  <=KEYPADTIMEOUT){

                                                              // If access granted, open 5 second window for pin pad commands.
        if(reader2Count >=4){
          if(reader2 !=0xB){                                  // Pin pad command can be any length, terminated with '#' on the keypad.
            if(keypadValue ==0){                              // This 0..9, A..F encoding works with many Wiegand-format keypad or reader 
              keypadValue = reader2;                          // plus keypad units.

            }
            else if(keypadValue !=0) {
              keypadValue = keypadValue <<4;
              keypadValue |= reader2;               
            }
            wiegand26.initReaderTwo();                         //Reset reader one and move on.
          } 
          else break;

        }

      }

        logkeypadCommand(2,keypadValue);
        runCommand(keypadValue);                              // Run any commands entered at the keypads.
        wiegand26.initReaderTwo();
      

   }
    wiegand26.initReaderTwo();                    
  } 


  /* Check physical sensors with 
   the logic below. Behavior is based on
   the current alarmArmed value.
   0=disarmed 
   1=armed
   2=
   3=
   4=door chime only (Unlock DOOR1, Check zone 0/chirp alarm if active)
   
   Modify the alarm sequence to meet your needs.
   */

  switch(alarmArmed) {


 case 0:
  {
    break;                                        // Alarm is not armed, do nothing.  
  }

    case 1:                                       // Alarm is armed
  {
                                              
                                                    
      if(alarmActivated==0){                       // If alarm is armed but not currently alarming, check sensor zones.

          if(pollAlarm(0) == 1 ){                   // If this zone is tripped, immediately set Alarm State to 2 (alarm delay).
              alarmState(2);                        // Also starts the delay timer    
              alarmDelay=millis();
              if(sensor[0]==false) {                // Only log and save if sensor activation is new.
               logalarmSensor(0);
               EEPROM.write(EEPROM_ALARM,0);        // Save the alarm sensor tripped to eeprom                                      
               sensor[0]=true;                      // Set value to not log this again                                                                        
              }
           } 
          if(pollAlarm(1) == 1 ){                  // If this zone is tripped, immediately set Alarm State to 1 (alarm immediate).
            alarmState(1);      
             if(sensor[1]==false) {                // Only log and save if sensor activation is new.
              logalarmSensor(1);
              EEPROM.write(EEPROM_ALARM,1);        // Save the alarm sensor tripped to eeprom                                     
              sensor[1]=true;                      // Set value to not log this again
             }  
          }
          if(pollAlarm(2) == 1 ){                  // If this zone is tripped, immediately set Alarm State to 1 (alarm immediate).
            alarmState(1);      
             if(sensor[2]==false) {                // Only log and save if sensor activation is new.
              logalarmSensor(2);
              EEPROM.write(EEPROM_ALARM,2);        // Save the alarm sensor tripped to eeprom                                     
              sensor[2]=true;                      // Set value to not log this again
             }    

           } 
           
          if(pollAlarm(3) == 1 ){                   // If this zone is tripped, immediately set Alarm State to 2 (alarm delay).
              alarmState(2);                        // Also starts the delay timer    
              alarmDelay=millis();
              if(sensor[3]==false) {                // Only log and save if sensor activation is new.
               logalarmSensor(3);
               EEPROM.write(EEPROM_ALARM,3);        // Save the alarm sensor tripped to eeprom                                      
               sensor[3]=true;                      // Set value to not log this again                                                                        
              }
           }    
                                                                                           
  
                                           
      }
   if(alarmActivated==1)  {                         // If alarm is actively going off (siren/strobe) for 10 min (6e5=10min)
    if(millis()-alarmSirenTimer >=3.6e6)            // Check for alarm interval expired and turn off if needed
     {
      digitalWrite(ALARMSIRENPIN,LOW);              // Turn on the chime instead  
      digitalWrite(ALARMSTROBEPIN,HIGH);     
     }
                           }  

   if(alarmActivated==2)  {                         // If alarm is activated on delay, take this action
    if(millis()-alarmDelay >=60000)                 // Turn on the siren once delay exceeds 60sec.
     {
      alarmState(1);                          
     }
                           }  
    
        
      break;
  
  }
  
  case 4: 
    {                                                // Door chime mode
      
      if((pollAlarm(3) !=0) && (doorChime==false)) {   // Only activate door chime once per opening
        chirpAlarm(3);                  
        logChime();
        doorChime=true;   
         }
      if(pollAlarm(3) ==0){
        doorChime=false;   }
        break;    
      
    }

  default: 
    {
      break;  
    }
  }
  
// Log all motion detector activations regardless of alarm armed state. Useful for "occupancy detection"

          if(pollAlarm(0) == 1 ){                  // If this zone is tripped, log the action only
          //  if(sensor[0]==false) 
          if((millis() - sensorDelay[0]) >=7500) {
           logalarmSensor(0);   
           sensorDelay[0]=millis();                                                                  
           sensor[0]=true;      }                 // Set value to not log this again for 7.5s              
           }

          if(pollAlarm(1) == 1 ){                  // If this zone is tripped, log the action only
         //   if(sensor[1]==false) 
          if((millis() - sensorDelay[1]) >=7500) {
           logalarmSensor(1);   
           sensorDelay[1]=millis();                                                            
           sensor[1]=true;                       // Set value to not log this again for 7.5s
          }           
         }
  } // End of loop()


void runCommand(long command) {         // Run any commands entered at the pin pad.

  switch(command) {                              


  case 0x1: 
    {                                     // If command = 1, deactivate alarm
      alarmState(0);                      // Set global alarm level variable
      armAlarm(0);
      chirpAlarm(1);
      break;  
    }

  case 0x2: 
    {                                       // If command =2, activate alarm with delay.

      doorUnlock(1);                        // Set global alarm level variable
      door1Locked=false;
      doorClosed=false;                      // 200 chirps = ~30 seconds delay

   if((pollAlarm(3) == 0) && (pollAlarm(2) == 0)) {                  // Do not arm the alarm if doors are open

     for(byte i=0; i<30; i++) {
         if((pollAlarm(3) !=0) && doorClosed==false) {             // Set door to be unlocked until alarm timeout or user exits
          lockall();    
          doorClosed=true; 
         }      
         digitalWrite(ALARMSTROBEPIN, HIGH);
         delay(500);
         digitalWrite(ALARMSTROBEPIN, LOW);
         delay(500);                        
      }
      chirpAlarm(2);
      armAlarm(1);                 
      lockall();                                                  // Lock all doors on exit
   }
  else {                                                          // Beep the alarm once and exit if attempt made to arm alarm with doors open
         digitalWrite(ALARMSTROBEPIN, HIGH);
         delay(500);
         digitalWrite(ALARMSTROBEPIN, LOW);
         delay(500);                        
         lockall();                                                  // Lock all doors anyway
       }
      break; 
    }
    
  case 0x3: 
    {

      doorLock(1);                       // Set door 2 to stay unlocked, and door 1 to be locked
      doorUnlock(2);
      door1Locked=true;
      door2Locked=false;
      chirpAlarm(3);   
      break;
    }

  case 0x4:                               // Set doors to remain open
    {
      armAlarm(4);
      doorUnlock(1);
      doorUnlock(2);
      door1Locked=false;
      door2Locked=false;
      chirpAlarm(4);   
      break;
    }
  case 0x5:                               // Relock all doors
    {
      lockall();
      chirpAlarm(5);   
      break;  
    }

  case 0x911: 
    {
      chirpAlarm(9);          // Emergency
      armAlarm(1);                   
      alarmState(1);
      break;  
    }

  case 0x20: 
    {                                   // If command = 20, do nothing
      break;
    }    
  default: 
    {       
      break;      
    }  
  }


}  


/* Alarm System Functions - Modify these as needed for your application. 
 Sensor zones may be polled with digital or analog pins. Unique reader2
 resistors can be used to check more zones from the analog pins.
 */

void alarmState(byte alarmLevel) {                    //Changes the alarm status based on this flow

  logalarmState(alarmLevel); 
  switch (alarmLevel) {                              
  case 0: 
    {                                                 // If alarmLevel == 0 turn off alarm.   
      digitalWrite(ALARMSIRENPIN, LOW);
      digitalWrite(ALARMSTROBEPIN, LOW);
      alarmActivated = alarmLevel;                    //Set global alarm level variable
      break;  
    }        
  case 1: 
    { 
      digitalWrite(ALARMSIRENPIN, HIGH);               // If alarmLevel == 1 turn on strobe lights and siren
  //    digitalWrite(ALARMSTROBEPIN, HIGH);            // Optionally activate yoru strobe/chome
      alarmSirenTimer=millis();
      alarmActivated = alarmLevel;                    //Set global alarm level variable
      logalarmTriggered();

      break;  
    }        

  case 2:                                        
    {
      digitalWrite(ALARMSTROBEPIN, HIGH);   
      alarmActivated = alarmLevel;
      break;    
    }

  case 3:                                        
    {

      alarmActivated = alarmLevel;
      break;    
    }
    /*
      case 4: {
     vaporize_intruders(STUN);
     break;
     }
     
     case 5: {
     vaporize_intruders(MAIM);
     }  etc. etc. etc.
     break;
     */

  default: 
    {                                            // Exceptional cases kill alarm outputs
      digitalWrite(ALARMSIRENPIN, LOW);          // Turn off siren and strobe
     // digitalWrite(ALARMSTROBEPIN, LOW);        
      break;
    } 


 

  }

      if(alarmActivated != EEPROM.read(EEPROM_ALARM)){    // Update eeprom value
         EEPROM.write(EEPROM_ALARM,alarmActivated); 
         }

}  //End of alarmState()

void chirpAlarm(byte chirps){            // Chirp the siren pin or strobe to indicate events.      
  for(byte i=0; i<chirps; i++) {
    digitalWrite(ALARMSTROBEPIN, HIGH);
    delay(100);
    digitalWrite(ALARMSTROBEPIN, LOW);
    delay(200);                              
  }    
}                                   

byte pollAlarm(byte input){

  // Return 1 if sensor shows < pre-defined voltage.
  delay(20);
  if(abs((analogRead(analogsensorPins[input])/4) - EEPROM.read(EEPROM_ALARMZONES+input)) >SENSORTHRESHOLD){
    return 1;

  }
  else return 0;
}

void trainAlarm(){                       // Train the system about the default states of the alarm pins.
  armAlarm(0);                           // Disarm alarm first
  alarmState(0);

  int temp[5]={0};
  int avg;

  for(int i=0; i<numAlarmPins; i++) {         

    for(int j=0; j<5;j++){                          
      temp[j]=analogRead(analogsensorPins[i]);
      delay(50);                                         // Give the readings time to settle
    }
    avg=((temp[0]+temp[1]+temp[2]+temp[3]+temp[4])/20);  // Average the results to get best values
    addToLog('t',avg);
    EEPROM.write((EEPROM_ALARMZONES+i),byte(avg));   //Save results to EEPROM
    avg=0;
  }

  logDate();
  //PROGMEMprintln(alarmtrainMessage);


}

void armAlarm(byte level){                       // Arm the alarm and set to level
  alarmArmed = level;
  logalarmArmed(level);

  sensor[0] = false;                             // Reset the sensor tripped values
  sensor[1] = false;
  sensor[2] = false;
  sensor[3] = false;

  if(level != EEPROM.read(EEPROM_ALARMARMED)){ 
    EEPROM.write(EEPROM_ALARMARMED,level); 
  }
}


/* Access System Functions - Modify these as needed for your application. 
 These function control lock/unlock and user lookup.
 */

int checkSuperuser(long input){       // Check to see if user is in the user list. If yes, return their index value.
int found=-1;
  for(int i=0; i<=numUsers; i++){   
    if(input == superUserList[i]){
      logDate();
      addToLog('Q',i);
      found=i;
      return found;    
    }
  }                   
  return found;             //If no, return -1
}


void doorUnlock(int input) {          //Send an unlock signal to the door and flash the Door LED
byte dp=1;
  if(input == 1) {
    dp=DOORPIN1; }
   else(dp=DOORPIN2);
  
  digitalWrite(dp, HIGH);
  addToLog('U',input);

}

void doorLock(int input) {          //Send a lock signal to the door and flash the Door LED
byte dp=1;
  if(input == 1) {
    dp=DOORPIN1; }
   else(dp=DOORPIN2);

  digitalWrite(dp, LOW);
  addToLog('L',input);

}
void unlockall() {
  doorUnlock(1);
  doorUnlock(2);
  alarmState(0);                                    
  armAlarm(4);
  door1Locked=false;
  door2Locked=false;
  chirpAlarm(3);    
  
  //PROGMEMprintln(doorsunlockedMessage);
}
void lockall() {                      //Lock down all doors. Can also be run periodically to safeguard system.
  doorLock(1);
  doorLock(2);
  door1Locked=true;
  door2Locked=true;
  //PROGMEMprintln(doorslockedMessage);

}


void logDate()
{
  ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  addToLog('H',hour);
  addToLog('M',minute);
  addToLog('E',second);
}


void logReboot() {                                  //Log system startup
  logDate();
    //PROGMEMprintln(rebootMessage);
}

void logChime() {
  //logDate();
    //PROGMEMprintln(doorChimeMessage);
}

void logTagPresent (long user, byte reader) {     //Log Tag Presented events
  //logDate();
  addToLog('R',user%divisor); 
  addToLog('r',user/divisor); 
}

void logAccessGranted(long user, byte reader) {     //Log Access events
  logDate();
  addToLog('G',user%divisor); 
  addToLog('g',user/divisor); 
}                                         

void logAccessDenied(long user, byte reader) {     //Log Access denied events
  logDate();
  addToLog('D',user%divisor); 
  addToLog('d',user/divisor); 
}   

void logkeypadCommand(byte user, long command){
  logDate();
  addToLog('C',user%divisor); 
  addToLog('c',user/divisor); 
}  




void logalarmSensor(byte zone) {     //Log Alarm zone events
  //logDate();
  addToLog('s',zone); 
}

void logalarmTriggered() {
  logDate();
  addToLog('T',0);
}

void logunLock(long user, byte door) {        //Log unlock events
  logDate();
  addToLog('U',user%divisor); 
  addToLog('u',user/divisor); 
}

void logalarmState(byte level) {        //Log unlock events
  //logDate();
  addToLog('m',level);
}

void logalarmArmed(byte level) {        //Log unlock events
  logDate();
  addToLog('A',level);
}

void logprivFail() {
  addToLog('F',2);
}

/*
void hardwareTest(long iterations)
{

  // Hardware testing routing. Performs a read of all digital inputs and
  // a write to each relay output. Also reads the analog value of each
  // alarm pin. Use for testing hardware. Wiegand26 readers should read 
  // "HIGH" or "1" when connected.
  

  pinMode(2,INPUT);
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(5,INPUT);

  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  for(long counter=1; counter<=iterations; counter++) {                                  // Do this number of times specified
    logDate();
    
    Serial.print("\n"); 
    Serial.println("Pass: "); 
    Serial.println(counter); 
    Serial.print("Input 2:");                    // Digital input testing
    Serial.println(digitalRead(2));
    Serial.print("Input 3:");
    Serial.println(digitalRead(3));
    Serial.print("Input 4:");
    Serial.println(digitalRead(4));
    Serial.print("Input 5:");
    Serial.println(digitalRead(5));
    Serial.print("Input A0:");                   // Analog input testing
    Serial.println(analogRead(0));
    Serial.print("Input A1:");
    Serial.println(analogRead(1));
    Serial.print("Input A2:");
    Serial.println(analogRead(2));
    Serial.print("Input A3:");
    Serial.println(analogRead(3));
    
    delay(5000);

    digitalWrite(6,HIGH);                         // Relay exercise routine
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);
    digitalWrite(9,HIGH);
    //Serial.println("Relays 0..3 on");
    delay(2000);
    digitalWrite(6,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    //Serial.println("Relays 0..3 off");

  }
}
*/

void clearUsers()    //Erases all users from EEPROM
{
  for(int i=EEPROM_FIRSTUSER; i<=EEPROM_LASTUSER; i++){
    EEPROM.write(i,0);  
    logDate();
    addToLog('Z',0);
  }
}

void addUser(int userNum, byte userMask, unsigned long tagNumber)       // Modifies a user an entry in the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to
  byte EEPROM_buffer[5] ={0};                                           // Buffer for creating the 4 byte values to write. Usermask is stored in byte 5.

  logDate();

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.
    addToLog('i',userNum);
  }
  else
  {
    EEPROM_buffer[0] = byte(tagNumber &  0xFFF);   // Fill the buffer with the values to write to bytes 0..4 
    EEPROM_buffer[1] = byte(tagNumber >> 8);
    EEPROM_buffer[2] = byte(tagNumber >> 16);
    EEPROM_buffer[3] = byte(tagNumber >> 24);
    EEPROM_buffer[4] = byte(userMask);

    for(int i=0; i<5; i++){
      EEPROM.write((offset+i), (EEPROM_buffer[i])); // Store the resulting value in 5 bytes of EEPROM.
    }

    addToLog('a',userNum);

  }
}

void deleteUser(int userNum)                                            // Deletes a user from the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to

  logDate();

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.

    addToLog('I',userNum);
  }
  else
  {
    for(int i=0; i<5; i++){
      EEPROM.write((offset+i), 0xFF); // Store the resulting value in 5 bytes of EEPROM.
                                                    // Starting at offset.
    }
    addToLog('d',userNum);
  }

}



int checkUser(unsigned long tagNumber)                                  // Check if a particular tag exists in the local database. Returns userMask if found.
{                                                                       // Users number 0..NUMUSERS
  // Find the first offset to check

  unsigned long EEPROM_buffer=0;                                         // Buffer for recreating tagNumber from the 4 stored bytes.
  int found=-1;
  
  logDate();


  for(int i=EEPROM_FIRSTUSER; i<=(EEPROM_LASTUSER-5); i=i+5){


    EEPROM_buffer=0;
    EEPROM_buffer=(EEPROM.read(i+3));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+2));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+1));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i));


    if((EEPROM_buffer == tagNumber) && (tagNumber !=0xFFFFFFFF) && (tagNumber !=0x0)) {    // Return a not found on blank (0xFFFFFFFF) entries 
      addToLog('c',((i-EEPROM_FIRSTUSER)/5));
      found = EEPROM.read(i+4);
      return found;
    }                             

  }
  addToLog('c',0);
  delay(1000);                                                            // Delay to prevent brute-force attacks on reader
  return found;                        
}


void dumpUser(EthernetClient client, byte usernum)                     // Return information ona particular entry in the local DB
{                                                                      // Users number 0..NUMUSERS


  unsigned long EEPROM_buffer=0;                                       // Buffer for recreating tagNumber from the 4 stored bytes.


  if((0<=usernum) && (usernum <=199)){

    int i=usernum*5+EEPROM_FIRSTUSER;

    EEPROM_buffer=0;
    EEPROM_buffer=(EEPROM.read(i+3));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+2));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i+1));
    EEPROM_buffer= EEPROM_buffer<<8;
    EEPROM_buffer=(EEPROM_buffer ^ EEPROM.read(i));

    client.print(((i-EEPROM_FIRSTUSER)/5),DEC);
    client.print("\t");
    client.print(EEPROM.read(i+4),DEC);
    client.print("\t");

    if(DEBUG==2){
      client.println(EEPROM_buffer,HEX);
                 }
     else {
           if(EEPROM_buffer != 0xFFFFFFFF) {
             client.println("********");
           }
     }
  
  }
  else client.println("Bad user number!");
}

boolean login(long input) {
  //logDate();
  if((consoleFail>=5) && (millis()-consolefailTimer<300000))  // Do not allow priv mode if more than 5 failed logins in 5 minute
  {  
    addToLog('F',1);
    return false;
  }
  else {
    if (input == PRIVPASSWORD)
    {
      consoleFail=0; 
      addToLog('S',0);
      privmodeEnabled=true;
      return true;
    }
    else {
      //addToLog('F',0);
      privmodeEnabled=false;    
      if(consoleFail==0) {                // Set the timeout for failed logins
        consolefailTimer=millis();
      }
      consoleFail++;                 // Increment the login failure counter
      return false;
    } 
  }
}

void printStatus(EthernetClient client) {
  client.println("<pre>");
  client.print("Alarm armed:");
  client.println(alarmArmed,DEC);
  client.print("Alarm activated:");
  client.println(alarmActivated,DEC);
  client.print("Alarm 3:");
  client.println(pollAlarm(3),DEC);
  client.print("Alarm 2:");
  client.println(pollAlarm(2),DEC);                  
  client.print("Door 1 locked:");
  client.println(door1Locked);                    
  client.print("Door 2 locked:");
  client.println(door2Locked); 
  client.println("</pre>"); 
}


void addToLog(char type, int data) {
  logKeys[logCursor] = type;
  logData[logCursor] = data;
  
  logCursor += 1;
  if(logCursor > sizeof(logKeys)) {
    logCursor = 0;
  }
}

void printLog(EthernetClient client) {
  client.println("<pre>");
  for(int i=0;i<sizeof(logKeys);i++) {
    client.print(logKeys[i]);
    client.print(": ");
    client.println(logData[i]);
  }
  client.println("</pre>");
}


void PROGMEMprintln(EthernetClient client, const prog_uchar str[])    // Function to retrieve strings from program memory
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++))){
    client.print(c);
  }
}

/* Wrapper functions for interrupt attachment
 Could be cleaned up in library?
 */
void callReader1Zero(){wiegand26.reader1Zero();}
void callReader1One(){wiegand26.reader1One();}
void callReader2Zero(){wiegand26.reader2Zero();}
void callReader2One(){wiegand26.reader2One();}
void callReader3Zero(){wiegand26.reader3Zero();}
void callReader3One(){wiegand26.reader3One();}


