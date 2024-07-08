//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used
//Example of variant with 4 servos and static/flashing LED outputs
//for Arduino UNO/Nano/Micro
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
//#define FIXMODE NORMALMODE //If this define is active, the operating mode is set, betrModeP is then
//not read and ignored. Possible values:
//NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A5;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A4;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
#define ENCODER_AKTIV//If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =   A2;//Rotary encoder input for adjustment.
const byte encode2P     =   A3;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV//CV values ​​from V7.0 (10 CV per address)

const int DccAddr          =  20;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//with LocoNet interface this is the LocoNetId
//#define NOACK //Activate this line if there is no hardware to read CV
//(no Ack pin) The pin defined in Interfac.h is then used as OUTPUT
//set, but can be used for any functions in the table below
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   13;//Display operating status (normal/programming) (LED)

#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT//Alternating indicators with both LEDs on when starting
const byte iniTyp[]    =    {   FSERVO,   FSERVO,   FSERVO,   FSERVO,  FSTATIC,  FSTATIC,    FCOIL };
const byte out1Pins[]  =    {       A0,       A1,       11,       12,        7,        8,       10 }; 
const byte out2Pins[]  =    {        3,        5,       NC,       NC,        6,       NC,        9 };
const byte out3Pins[]  =    {       NC,       NC,       NC,       NC,       NC,       NC,       NC };

const byte iniCVx[10][sizeof(iniTyp)]  = {
/*iniFmode (CV120,130,..*/ { SAUTOOFF, SAUTOOFF, SAUTOOFF,        0,  BLKMODE,        0, CAUTOOFF },
/*iniPar1 (CV121,131,..*/  {        0,        0,        0,        0,       50,        0,       50 },
/*iniPar2 (CV122,132,..*/  {      180,      180,      180,      180,       50,        0,       50 },
/*iniPar3 (CV123,133,..*/  {        8,        8,        8,        0,      100,        0,        0 },
/*iniPar4 (CV124,134,..*/  {        0,         0,       0,        0,        0,        0,        0 },
/*iniPar5 (CV125,135,..*/  {       0,          0,       0,        0,        0,        0,        0 },
/*iniPar6 (CV126,136,..*/  {       0,          0,       0,        0,        0,        0,        0 },
/*iniPar7 (CV127,137,..*/  {       0,          0,       0,        0,        0,        0,        0 },
/*iniPar8 (CV128,138,..*/  {       0,          0,       0,        0,        0,        0,        0 },
/*iniState (CV129,139,..*/ {       0,          0,       0,        0,        0,        0,        0 }};//Status values
//------------------------------------------------------------------------------------
