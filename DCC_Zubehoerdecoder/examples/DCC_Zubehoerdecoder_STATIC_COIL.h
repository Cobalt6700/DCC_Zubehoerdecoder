//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder-LS-Nano.h" //Path to an alternative config file
#define KONFIG_FILE "TestKonf\DCC_ZubehoerdecoderV71.h"//Path to an alternative config file
#if !defined( KONFIG_FILE ) || defined ( EXEC_KONFIG )

//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used

#define IFC_SERIAL  Serial//If the define is active, commands can also be issued via the serial interface
#define SERIAL_BAUD 115200

//Example of variant with light exit signal with distant signal, with operating mode LED on pin 13 (internal LED)
//for Arduino Nano
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
//#define FIXMODE NORMALMODE //If this define is active, the operating mode is set, betrModeP is then
//not read and ignored. Possible values:
//NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A7;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A6;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
#define ENCODER_AKTIV//If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =   A5;//Rotary encoder input for adjustment.
const byte encode2P     =   A4;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV//CV values ​​from V7.0 (10 CV per address)

const int DccAddr           =  17;//DCC decoder address
const byte iniMode          = AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//with LocoNet interface this is the LocoNetId
//#define NOACK //Activate this line if there is no hardware to read CV
//(no Ack pin) The pin defined in Interfac.h is then used as OUTPUT
//set, but can be used for any functions in the table below
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   13;//Display operating status (normal/programming) (LED)

#define STATICRISE  (250/50 << 4)//Softled riseTime = 250 (max = 750)
#define COILMOD     NOPOSCHK|CAUTOOFF
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE//Alternating indicators with both LEDs on when starting
const byte iniTyp[]     =   {   FSTATIC,   FSTATIC3,   FCOIL };
const byte out1Pins[]   =   {        A0,/*rt*/ 9,        3 };//output pins of the functions
const byte out2Pins[]   =   {        A1,/*rt*/10,        5 };
const byte out3Pins[]   =   {         7,/*gn*/11,        6 };

const byte iniCVx[10][sizeof(iniTyp)]  = {
/*iniFmode (CV120,130,..*/ { STATICMOD,    0b00111,  COILMOD },
/*iniPar1 (CV121,131,..*/  {       100,         50,       50 },
/*iniPar2 (CV122,132,..*/  {       100,         25,       50 },
/*iniPar3 (CV123,133,..*/  {       200,    0b00101,        0 },
/*iniPar4 (CV124,134,..*/  {         0,        100,        0 }, 
/*iniPar5 (CV125,135,..*/  {         0,        200,        0 },
/*iniPar6 (CV126,136,..*/  {         0,    0b00101,        0 },
/*iniPar7 (CV127,137,..*/  {         0,        200,        0 },
/*iniPar8 (CV128,138,..*/  {         0,          0,        0 },
/*iniState (CV129,139,..*/ {         0,          0,        0 }};//Status values
//------------------------------------------------------------------------------------
#endif
