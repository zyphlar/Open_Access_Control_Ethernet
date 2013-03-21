/*
 * Open Source RFID Access Controller - Ethernet Branch
 *
 * 3/21/2013 v0.05 (branch based on upstream 4/3/2011 v1.32)
 * Will Bradley - will@heatsynclabs.org
 * Short Tie - tie.short@gmail.com
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
 * U=unlocked door (1=door1, 2=door2)
 * Z=user db cleared (0)
 * z=log cleared (0)
 *   Log Format:
 *   H M E 
 */

#include <Wire.h>         // Needed for I2C Connection to the DS1307 date/time chip
#include <EEPROM.h>       // Needed for saving to non-voilatile memory on the Arduino.
#include <avr/pgmspace.h> // Allows data to be stored in FLASH instead of RAM


#include <Ethernet.h>     // Ethernet stuff
#include <SPI.h>          


#include <DS1307.h>       // DS1307 RTC Clock/Date/Time chip library
#include <WIEGAND26.h>    // Wiegand 26 reader format libary
#include <PCATTACH.h>     // Pcint.h implementation, allows for >2 software interupts.


//-------- begin user config section --------

#define DEBUG 2                         // Set to 2 for display of raw tag numbers in log files, 1 for only denied, 0 for never.               

#define adam   0xABCDE                  // Name and badge number in HEX. We are not using checksums or site ID, just the whole
#define bob   0xBCDEF                  // output string from the reader.
#define carl   0xA1B2C3
const long  superUserList[] = { adam, bob, carl};  // Super user table (cannot be changed by software)

#define PRIVPASSWORD 0x1234             // Console "priveleged mode" password

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192,168,1,177);

//-------- end user config section --------

#define DOORDELAY 5000                  // How long to open door lock once access is granted. (2500 = 2.5s)
#define SENSORTHRESHOLD 100             // Analog sensor change that will trigger an alarm (0..255)
#define NUM_SENSORS 4                   // The number of sensors

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

/* Definitions for Log types */
#define LOG_REBOOT		0
#define LOG_CHIME		1
#define LOG_TAG_PRESENT		2
#define LOG_ACCESS_GRANTED	3
#define LOG_ACCESS_DENIED	4
#define LOG_KEYPAD_COMMAND	5
#define LOG_ALARM_ZONE		6
#define LOG_ALARM_TRIGGERED	7
#define LOG_UNLOCK		8
#define LOG_LOCK		9
#define LOG_ALARM_STATE		10
#define LOG_ALARM_ARMED		11
#define LOG_PRIVELEDGE_FAILED	12
#define LOG_ALARM_TRAIN		13
#define LOG_SUPERUSER		14
#define LOG_HARDWARE_TEST	15
#define LOG_CLEAR_USERS		16
#define LOG_ADD_USER_FAIL	17
#define LOG_ADD_USER_SUCCESS	18
#define LOG_DELETE_USER_FAIL	19
#define LOG_DELETE_USER_SUCCESS	20
#define LOG_CHECK_USER		21
#define LOG_LOGIN_FAIL		22
#define LOG_LOGIN_SUCCESS	23
#define LOG_LOCKED_OUT_USER	24
#define LOG_CLEAR_LOG		25

/* Definitions for LOG_PRIVILEDGE_FAILED */
#define WRONG_PASSWORD		0
#define TOO_MANY_TRIES		1
#define NOT_LOGGED_IN		2

/* Defintions for LOG_LOCK */
#define DOOR1			1
#define DOOR2			2
#define BEDTIME			3

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
#define NUMDOORS (sizeof(doorPin)/sizeof(byte))  //TODO: NUMDOORS isn't used and doorPin isn't defined... remove?
#define numAlarmPins (sizeof(analogsensorPins)/sizeof(byte))

//Other global variables
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;     // Global RTC clock variables. Can be set using DS1307.getDate function.

byte alarmActivated = EEPROM.read(EEPROM_ALARM);                   // Read the last alarm state as saved in eeprom.
byte alarmArmed = EEPROM.read(EEPROM_ALARMARMED);                  // Alarm level variable (0..5, 0==OFF) 

boolean sensor[NUM_SENSORS]={false};                                         // Keep track of tripped sensors, do not log again until reset.
unsigned long sensorDelay[2]={0};                                  // Same as above, but sets a timer for 2 of them. Useful for logging
                                                                   // motion detector hits for "occupancy check" functions.

// Enable up to 3 door access readers.
volatile long reader1 = 0;
volatile int  reader1Count = 0;
volatile long reader2 = 0;
volatile int  reader2Count = 0;
int userMask=0;
boolean keypadGranted=0;                                       // Variable that is set for authenticated users to use keypad after login

//volatile long reader3 = 0;                                   // Uncomment if using a third reader.
//volatile int  reader3Count = 0;

unsigned long keypadTime = 0;                                  // Timeout counter for  reader with key pad
unsigned long keypadValue=0;


boolean privmodeEnabled = false;                               // Switch for enabling "priveleged" commands

// Log buffer
int logLevel=2;
char logKeys[40]={0};
int logData[40]={0};
int logCursor=0;

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
const prog_uchar help[]           PROGMEM  = {"<hr/>See source for command syntax."};  //<pre>Numbers must be padded.\n\n?e=0000 - enable priv (0 to logout)\n?s000 - show user\n?m000&p000&t00000000 - modify user(0-200) perm(0-255) tag(0-f)\n?a - list all\n?r000 - remove user\n?o1 ?o2 - open door 1/2\n?u ?u=1 ?u=2 - unlock all/1/2\n?l ?l=1 ?l=2 - lock all/1/2\n?1 - disarm\n?2 - arm\n?3 - train\n?9 - status\n?z - show log\n?y - clear log\n?w0000000000000 - show year-month-day-dayofweek-hour-min-sec\n?x - set year-month-day-dayofweek-hour-min-sec\n?v=0 ?v=1 ?v=2 ?v=3 - set logging to MostVerbose/Verbose/Quiet/MostQuiet</pre>"}; 
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






//  Serial.begin(57600);	               	       // Set up Serial output at 8,N,1,57600bps
  
  
  log(LOG_REBOOT, 0, 0);
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

          if(readString.indexOf("?") < 0) {
            PROGMEMprintln(client,title);
            PROGMEMprintln(client,help);
          }
          else {
            
          if(readString.indexOf("?e=") > 0 || readString.indexOf("&e=") > 0) { // login
            int offset = readString.indexOf("e=");
            char pass[5] = {readString[offset+2],readString[offset+3],readString[offset+4],readString[offset+5],'\0'};

            if(login(strtoul(pass,NULL,16))) {
              client.println("authok");  
            }
            else {
              client.println("authfail");
              break;  // we don't need extra errors below; we already know about the authfail.
            }
          }

          if(privmodeEnabled==true) {

          switch(readString[readString.indexOf("?")+1]){

          case 's': { // show user
              int offset = readString.indexOf("?s");
              char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};
            
              client.println("<pre>");
              client.print("UserNum:");
              client.print(" ");
              client.print("Usermask:");
              client.print(" ");
              client.println("TagNum:");
              dumpUser(client, atoi(usernum));
              client.println("</pre>");
              break;
          }
          case 'm': { // modify user #, permission #, tag # (?m000&p000&t00000000 must be zero-padded)
            int offset = readString.indexOf("?m");  // user, 3 chars
            int initialoffset = offset; // save for comparison
            char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};

            offset = readString.indexOf("&p");  // permissions mask, 3 chars
            char usermask[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};

            offset = readString.indexOf("&t");  // tag, 8 chars
            char usertag[9] = {readString[offset+2],readString[offset+3],readString[offset+4],readString[offset+5],
                               readString[offset+6],readString[offset+7],readString[offset+8],readString[offset+9],'\0'};
            
            if(offset-initialoffset == 10){
                client.println("<pre>");
                client.println("prev:");
                dumpUser(client, atoi(usernum));
                addUser(atoi(usernum), atoi(usermask), strtoul(usertag,NULL,16));
                client.println("cur:");
                dumpUser(client, atoi(usernum));
                client.println("</pre>");
            }
            else {
              client.println("err:query");
            }
            break;
          }
          case 'a': {  //list all users
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
              break;
          }
          case 'r': {  //remove user (?r000)
            int offset = readString.indexOf("?r");
            char usernum[4] = {readString[offset+2],readString[offset+3],readString[offset+4],'\0'};
            
              client.println("r");           
              client.println("<pre>");
              client.println("prev:");
              dumpUser(client,atoi(usernum));
              deleteUser(atoi(usernum));
              client.println("cur:");
              dumpUser(client,atoi(usernum));
              break;
          }
          case 'o': {  // open door ?o1 or ?o2
            int offset = readString.indexOf("?o");
  
              if(readString[offset+2] == '1'){  
                alarmState(0);                                       // Set to door chime only/open doors                                                                       
                armAlarm(4);
                doorUnlock(1);                                       // Open the door specified
                door1locktimer=millis();
                PROGMEMprintln(client,open1);
              }                    
              else{
                if(readString[offset+2] == '2'){  
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
              break;
          }
          case 'u': {  //unlock (?u or ?u=1 or ?u=2)
              int offset = readString.indexOf("?u="); // see if we're unlocking a specific door
              if(offset > 0) {
                if(readString[offset+3] == '1'){ 
                  doorUnlock(1);
                  alarmState(0);
                  armAlarm(4);
                  door1Locked=false;
                  chirpAlarm(3);
                  PROGMEMprintln(client,unlock1);
                }
                else {
                  if(readString[offset+3] == '2'){ 
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
              break;
          }
          case 'l': {   //lock (?l or ?l=1 or ?l=2)
              int offset = readString.indexOf("?l="); // see if we're unlocking a specific door
              if(offset > 0) {
                if(readString[offset+3] == '1'){ 
                  doorLock(1);
                  door1Locked=true;
                  chirpAlarm(3);
                }
                else if(readString[offset+3] == '2'){ 
                    doorLock(2);
                    door2Locked=true;
                    chirpAlarm(3);          
                }
                else {
                  lockall();
                  chirpAlarm(1); 
                  PROGMEMprintln(client,lockboth);
                }  
              }
              else {  // not unlocking a specific door; unlock all.
                PROGMEMprintln(client,lockboth);
                lockall();
              }
              printStatus(client);
              break;
          }
          case '1': {  // disarm
              armAlarm(0);
              alarmState(0);
              chirpAlarm(1);  
              printStatus(client);
              break;
          }
          case '2': { // arm
              chirpAlarm(20);        // 200 chirps = ~30 seconds delay
              armAlarm(1);                           
              printStatus(client);
              break;
          }
          case '3': { // train
              trainAlarm();
              printStatus(client);
              break;
          }
          case 'z': { // log
              printLog(client);
              break;
          }
          case 'y': { // clear log
              for(int i=0;i<sizeof(logKeys);i++) {
                logKeys[i] = 0;
                logData[i] = 0;
              }
              logCursor = 0;
              log(LOG_CLEAR_LOG, 0,0);
              
              client.println("y");
              break;
          }
          case 'w': { // Print out the date - "YYMMDDWHHmmSS"
            ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    

            client.print(year, DEC);
            client.print(month, DEC);
            client.print(dayOfMonth, DEC);
            client.print(dayOfWeek, DEC);
            client.print(hour, DEC);
            client.print(minute, DEC);
            client.print(second, DEC);
          }
          case 'x': { // Assign date/time format (?xYYMMDDWHHmmSS)
            int offset = readString.indexOf("?x");
            char year[3] = {readString[offset+2],readString[offset+3],'\0'};
            char mon[3] = {readString[offset+4],readString[offset+5],'\0'};
            char dom[3] = {readString[offset+6],readString[offset+7],'\0'};
            byte dow = {readString[offset+8]};
            char hour[3] = {readString[offset+9],readString[offset+10],'\0'};
            char minute[3] = {readString[offset+11],readString[offset+12],'\0'};
            char sec[3] = {readString[offset+13],readString[offset+14],'\0'};
            ds1307.setDateDs1307(atoi(sec),atoi(minute),atoi(hour),dow,atoi(dom),atoi(mon),atoi(year));         
  /*  Sets the date/time (needed once at commissioning)
   
   byte second,        // 0-59
   byte minute,        // 0-59
   byte hour,          // 1-23
   byte dayOfWeek,     // 1-7
   byte dayOfMonth,    // 1-28/29/30/31
   byte month,         // 1-12
   byte year);          // 0-99
   */
             break;
          }
          case 'v': { // Change the amount of data recorded to the log
             int offset = readString.indexOf("?v=");
             switch(readString[offset+2]){
              case '0':{
                logLevel = 0; // 0 - Most Verbose
                break;
              }
              case '1':{
                logLevel = 1; // 1 - Verbose
                break;
              }
              case '2':{
                logLevel = 2; // 2 - Quiet
                break;
              }
              case '3':{
                logLevel = 3; // 3 - Most Quiet
                break;
              }
              default: { }
             }
             break;
          }
          case '9': {
            printStatus(client);
          }
          default: {}
          } // End switch on query letter
          } // End Calls that require authentization
          /* Calls that do not require authentation */
          else{

          if(readString.indexOf("?9") > 0) { // status
            printStatus(client);
          }
          else{
            PROGMEMprintln(client,noauth);
            log(LOG_PRIVELEDGE_FAILED, 0, NOT_LOGGED_IN);
          }
          } // End calls that do not require authentication

          if(readString.indexOf("&e=") > 0) { // if e is passed as a second parameter, log out.
            login(strtoul("0000",NULL,16)); // 0000 = logout
          }
          } // End readString has query portion
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


  // Notes: RFID polling is interrupt driven, just test for the reader1Count value to climb to the bit length of the key
  // change reader1Count & reader1 et. al. to arrays for loop handling of multiple reader output events
  // later change them for interrupt handling as well!
  // currently hardcoded for a single reader unit

  /* This code checks a reader with a 26-bit keycard input. Use the second routine for readers with keypads.  
   * A 5-second window for commands is opened after each successful key access read.
   */

  if(reader1Count >= 26) {                           //  When tag presented to reader1 (No keypad on this reader)
    processTagAccess(reader1, 1);
  }
  
  if(reader2Count >= 26){                            // Tag presented to reader 2
    processTagAccess(reader2, 2);
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

	for(int i=0; i<NUM_SENSORS; i++) {        // For each of the Sensors Check their status

          if(pollAlarm(i) == 1 ){
              if(i == 0 || i == 3){                // If zone 0 or 3 are tripped, immediately set Alarm State to 2 (alarm delay).
                alarmState(2);                     // Also starts the delay timer    
                alarmDelay=millis();
              }
              else {
                alarmState(1);                     // Otherwise, immediately set Alarm State to 1 (alarm immediate).
              }
              if(sensor[i]==false) {                // Only log and save if sensor activation is new.
               log(LOG_ALARM_ZONE,0,i);
               EEPROM.write(EEPROM_ALARM,i);        // Save the alarm sensor tripped to eeprom                                      
               sensor[i]=true;                      // Set value to not log this again                                                                        
              }
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
        log(LOG_CHIME, 0, 0);
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
  
// Log all motion detector activations (currently 0,1) regardless of alarm armed state. Useful for "occupancy detection"

  for(int i=0; i<2; i++){
          if(pollAlarm(i) == 1 ){                  // If this zone is tripped, log the action only
          
          // If the sensor has triggered but not in the last 7.5s. This will not log continual movement, only new movement. 
          if(sensor[i]==false && ((millis() - sensorDelay[i]) >=7500 )){ 
            log(LOG_ALARM_ZONE,0,i);   
            sensorDelay[i]=millis();                                                                  
            sensor[i]=true; 
            }             
          }
          else if (pollAlarm(i) == 0){
            sensor[i]=false;
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

  log(LOG_ALARM_STATE, 0, alarmLevel); 
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
      log(LOG_ALARM_TRIGGERED, 0, 0);

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
    log(LOG_ALARM_TRAIN, avg,i);
    EEPROM.write((EEPROM_ALARMZONES+i),byte(avg));   //Save results to EEPROM
    avg=0;
  }



}

void armAlarm(byte level){                       // Arm the alarm and set to level
  alarmArmed = level;
  log(LOG_ALARM_ARMED,0,level);

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

int processTagAccess(long reader, int doorNum) {

    log(LOG_TAG_PRESENT,reader,doorNum);                 // Write log entry to serial port
    chirpAlarm(1);                                       // Chirp alarm to show that tag input done              
                                                         // CHECK TAG IN OUR LIST OF USERS. -1 = no match                                  
    keypadGranted=false;                                // Reset the keypad authorized variable

    userMask=checkUser(reader);    

    if(userMask>=0){    
      switch(userMask) {
 
   case 0:                         // No outside privs, do not log denied.
    {                              // authenticate and log only.
    log(LOG_ACCESS_GRANTED,reader, doorNum);
    break;
    }
      
   case 10:                         // Authenticating immediately locks up and arms alarm
    {                              // 
    log(LOG_ACCESS_GRANTED,reader, doorNum);
    runCommand(0x2);
    break;
    }
    
   case 20:                                               //Limited hours user
    {
    ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);    
    if((hour >=17) && (hour <=23)){
         alarmState(0);
         armAlarm(0);                                     //  Deactivate Alarm                   
         switch(doorNum){
           case 1:
             log(LOG_ACCESS_GRANTED,reader, 1);           // Log and unlock door
             door1locktimer=millis();
             doorUnlock(doorNum);
           break;
           case 2:
             log(LOG_ACCESS_GRANTED,reader, 2);           // Log and unlock door
             door2locktimer=millis();
             doorUnlock(doorNum);
           break;
         }
         keypadGranted=1;
    }
     break;
    }
  
   case 255:                                               // Locked out      
    {
     log(LOG_LOCKED_OUT_USER, reader,userMask);
     break;
    }
    
    default:  
    {            
         alarmState(0);
         armAlarm(0);                            //  Deactivate Alarm                          
         switch(doorNum){
           case 1:
             log(LOG_ACCESS_GRANTED,reader, 1);           // Log and unlock door
             door1locktimer=millis();
             doorUnlock(doorNum);
           break;
           case 2:
             log(LOG_ACCESS_GRANTED,reader, 2);           // Log and unlock door
             door2locktimer=millis();
             doorUnlock(doorNum);
           break;
         }
         
         keypadGranted=1;
         break;
    }
                 }                                      

  }
    else 
    {                                             
     if(checkSuperuser(reader) >= 0) {              // Check if a superuser, grant access.
         alarmState(0);
         armAlarm(0);                              //  Deactivate Alarm
         chirpAlarm(1);                          
         switch(doorNum){
           case 1:
             log(LOG_ACCESS_GRANTED,reader, 1);           // Log and unlock door
             door1locktimer=millis();
             doorUnlock(doorNum);
           break;
           case 2:
             log(LOG_ACCESS_GRANTED,reader, 2);           // Log and unlock door
             door2locktimer=millis();
             doorUnlock(doorNum);
           break;
         }
         keypadGranted=1;
                                      }
      else{                                
      log(LOG_ACCESS_DENIED,reader,doorNum);                 //  no tickee, no laundree
          }
    }
    
    switch(doorNum)
    {
      case 1:
      {
        wiegand26.initReaderOne();                   //  Reset for next tag scan
        break;
      }
      case 2:
      {
        wiegand26.initReaderTwo();                   //  Reset for next tag scan
        unsigned long keypadTime=0;                  //  Timeout counter for  reader with key pad
        long keypadValue=0;
        keypadTime=millis();
        break;
      }
      default:
      {}
    }  
                                         
   if(keypadGranted==1) 
    {
      while((millis() - keypadTime)  <=KEYPADTIMEOUT){

                                                     // If access granted, open 5 second window for pin pad commands.
        if(reader2Count >=4){
          if(reader !=0xB){                         // Pin pad command can be any length, terminated with '#' on the keypad.
            if(keypadValue ==0){                     // This 0..9, A..F encoding works with many Wiegand-format keypad or reader 
              keypadValue = reader;                 // plus keypad units.

            }
            else if(keypadValue !=0) {
              keypadValue = keypadValue <<4;
              keypadValue |= reader;               
            }
            wiegand26.initReaderTwo();               //Reset reader one and move on.
          } 
          else break;

        }

      }

        log(LOG_KEYPAD_COMMAND,keypadValue,doorNum);
        runCommand(keypadValue);                              // Run any commands entered at the keypads.
        switch(doorNum)
        {
          case 1:
          {
            wiegand26.initReaderOne();                   //  Reset for next tag scan
            break;
          }
          case 2:
          {
            wiegand26.initReaderTwo();
            break;
          }
          default:
          {}
        }  
      

   }
}

int checkSuperuser(long input){       // Check to see if user is in the user list. If yes, return their index value.
int found=-1;
  for(int i=0; i<=numUsers; i++){   
    if(input == superUserList[i]){
      log(LOG_SUPERUSER, input, i);
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
  log(LOG_UNLOCK, 0, input);
}

void doorLock(int input) {          //Send a lock signal to the door and flash the Door LED
byte dp=1;
  if(input == 1) {
    dp=DOORPIN1; }
   else(dp=DOORPIN2);

  digitalWrite(dp, LOW);
  log(LOG_LOCK, 0, input);
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


void log(byte Action, long LongInfo, byte ShortInfo)
{
  ds1307.getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  if(logLevel < 2) {
    addToLog('H',hour);
    addToLog('M',minute);
    addToLog('E',second);
  }

  switch(logLevel) {
   // Most Verbose Logging level.
  case 0: {
   switch(Action) {
    case LOG_CHIME: {		// Chime
     //PROGMEMprintln(doorChimeMessage);
    }
    case LOG_ALARM_ARMED: {		//log Alarm Armed - ShortInof is alarm level
     addToLog('A',ShortInfo);
     break;
    }
    case LOG_CHECK_USER: {
     addToLog('c', ShortInfo);
     break;
    }
    case LOG_ALARM_STATE: {		// Log Alarm State - ShortInfo is alarm level
     addToLog('m',ShortInfo);
     break;
    }
    case LOG_ALARM_ZONE: {		// Log Alarm zone events - ShortInfo is zone
     addToLog('s',ShortInfo); 
     break;
    }
    case LOG_HARDWARE_TEST: {
     break;
    }
    default: {}
    }
   }

   // Verbose Logging level.
   case 1: {
   switch(Action) {
    case LOG_SUPERUSER: {		// Log superuser card used - ShortInfo is the SuperUser number
       addToLog('Q',ShortInfo);
     break;
    }
    case LOG_TAG_PRESENT: {		// Log Tag Presented event - LongInfo is user, ShortInfo door number
     addToLog('R',LongInfo%divisor); 
     addToLog('r',LongInfo/divisor); 
     break;
    }
    case LOG_LOGIN_SUCCESS: {
     addToLog('S',0);
     break;
    }
    case LOG_CLEAR_LOG: {
     addToLog('z',0);
     break;
    }
    default: {}
    }
   }

   // Queit Logging level.
   case 2: {
   switch(Action) {
    case LOG_ACCESS_GRANTED: {		// Log Access Granted event - LongInfo is user, ShortInfo door number
     addToLog('G',LongInfo%divisor); 
     addToLog('g',LongInfo/divisor); 
     break;
    }
    case LOG_ACCESS_DENIED: {		// Log Access Denied event - LongInfo is user, ShortInfo door number
     addToLog('D',LongInfo%divisor); 
     addToLog('d',LongInfo/divisor); 
     break;
    }
    case LOG_KEYPAD_COMMAND: {		// Log Keypad command - logkeypadCommand(byte user, long command) 
     addToLog('C',LongInfo%divisor); 
     addToLog('c',LongInfo/divisor); 
     break;
    }
    case LOG_UNLOCK: {			// Log unlock event - LongInfo is user, ShortInfo is door
     addToLog('U',ShortInfo);  
     break;
    }
    case LOG_LOCK: {			// Log lock event - ShortInfo is door number
     addToLog('L',ShortInfo); 
     break;
    }
    case LOG_ALARM_TRAIN: {
     //PROGMEMprintln(alarmtrainMessage);
     addToLog('t',LongInfo/divisor);	// Log training of sensor values - LongInfo is sensor average, ShortInfo is sensor number
     break;
    }
    default: {}
    }
   }

   // Most Queit Logging level. These will always be logged.
   default:{
   switch(Action) {
    case LOG_REBOOT: {		// Reboot - Log system startup
     //PROGMEMprintln(rebootMessage);
     break;
    }
    case LOG_ALARM_TRIGGERED: {		// Log Alarm Triggered
     addToLog('T',0);
     break;
    }
    case LOG_PRIVELEDGE_FAILED: {
     addToLog('F',ShortInfo);
     break;
    }
    case LOG_CLEAR_USERS: {
     addToLog('Z',0);
     break;
    }
    case LOG_ADD_USER_FAIL: {
     addToLog('i',LongInfo);
     break;
    }
    case LOG_ADD_USER_SUCCESS: {
     addToLog('a',LongInfo);
     break;
    }
    case LOG_DELETE_USER_FAIL: {
     addToLog('I',LongInfo);
     break;
    }
    case LOG_DELETE_USER_SUCCESS: {
     addToLog('d',LongInfo);
     break;
    }
    case LOG_LOGIN_FAIL: {
     addToLog('F',1);
     break;
    }
    case LOG_LOCKED_OUT_USER: {		// log Locked out user attempted access, LongInfo user, ShortInfo usermask
     addToLog('f',ShortInfo);
     break;
    }
    default: {}
    }
   }
  }


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
    log(LOG_HARDWARE_TEST, 0 ,0);
    
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
    log(LOG_CLEAR_USERS, 0, 0);
  }
}

void addUser(int userNum, byte userMask, unsigned long tagNumber)       // Modifies a user an entry in the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to
  byte EEPROM_buffer[5] ={0};                                           // Buffer for creating the 4 byte values to write. Usermask is stored in byte 5.

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.
    log(LOG_ADD_USER_FAIL, userNum, 0);
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

    log(LOG_ADD_USER_SUCCESS, userNum, 0);

  }
}

void deleteUser(int userNum)                                            // Deletes a user from the local database.
{                                                                       // Users number 0..NUMUSERS
  int offset = (EEPROM_FIRSTUSER+(userNum*5));                          // Find the offset to write this user to

  if((userNum <0) || (userNum > NUMUSERS)) {                            // Do not write to invalid EEPROM addresses.

    log(LOG_DELETE_USER_FAIL, userNum, 0);
  }
  else
  {
    for(int i=0; i<5; i++){
      EEPROM.write((offset+i), 0xFF); // Store the resulting value in 5 bytes of EEPROM.
                                                    // Starting at offset.
    }
    log(LOG_DELETE_USER_SUCCESS, userNum, 0);
  }

}



int checkUser(unsigned long tagNumber)                                  // Check if a particular tag exists in the local database. Returns userMask if found.
{                                                                       // Users number 0..NUMUSERS
  // Find the first offset to check

  unsigned long EEPROM_buffer=0;                                         // Buffer for recreating tagNumber from the 4 stored bytes.
  int found=-1;
  
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
      log(LOG_CHECK_USER, 0, (i-EEPROM_FIRSTUSER)/5);
      found = EEPROM.read(i+4);
      return found;
    }                             

  }
  log(LOG_CHECK_USER, 0, 0);
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
  if((consoleFail>=5) && (millis()-consolefailTimer<300000))  // Do not allow priv mode if more than 5 failed logins in 5 minute
  {  
    log(LOG_LOGIN_FAIL,0,0);
    return false;
  }
  else {
    if (input == PRIVPASSWORD)
    {
      consoleFail=0; 
      log(LOG_LOGIN_SUCCESS, 0, 0);
      privmodeEnabled=true;
      return true;
    }
    else {
      //log(LOG_LOGIN_FAIL,0,0);
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


