//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used
//Example of variant with light exit signal with distant signal, with operating mode LED on pin 13 (internal LED)
//for Arduino Micro
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
//#define FIXMODE NORMALMODE //If this define is active, the operating mode is set, betrModeP is then
//not read and ignored. Possible values:
//NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A1;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A0;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
#define ENCODER_AKTIV//If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =   1;//Rotary encoder input for adjustment.
const byte encode2P     =   0;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
const int DccAddr          =  20;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   17;//Display operating status (normal/programming) (LED)

#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT//Alternating indicators with both LEDs on when starting
const byte iniTyp[]     =   {    FSTATIC,  FSERVO,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL };
const byte out1Pins[]   =   {       A2,         3,/*rt*/ 9,/*rt*/10,/*ge*/15,        5 };//output pins of the functions
const byte out2Pins[]   =   {       A3,        NC,/*gn*/14,/*ws*/ 8,/*gn*/16,        6 };
const byte out3Pins[]   =   {       NC,        NC,/*ge*/ 7,         NC,       NC,       NC };
 
const byte iniFmode[]     = {STATICMOD,  SERVOMOD,          0,          0,         0,  COILMOD };
const byte iniPar1[]      = {       50,        30,    0b01001,    0b10001,       0b01,       50 };
const byte iniPar2[]      = {       50,       150,    0b00010,    0b00110,       0b10,       50 };
const byte iniPar3[]      = {       50,         8,          5,          0,         19,        0 };
const byte iniPar4[]      = {        0,         0,    0b00101,          0,         0,        0,};//only for light signals!
//------------------------------------------------------------------------------------


