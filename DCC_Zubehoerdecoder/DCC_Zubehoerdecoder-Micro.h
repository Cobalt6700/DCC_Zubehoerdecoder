//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder-Micro1.h" //Path to an alternative config file
//#define KONFIG_FILE "TestKonf\DCC_ZubehoerdecoderV71-Micro.h" //Path to an alternative config file
//#define KONFIG_FILE "TestKonf\V71-Micro-Signal3.h" //Path to an alternative config file
//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder_STATIC_COIL.h" //Path to an alternative config file
#if !defined( KONFIG_FILE ) || defined ( EXEC_KONFIG )

//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used
//#define IFC_SERIAL Serial //If the define is active, commands can also be issued via the serial interface
#define SERIAL_BAUD 115200

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
const byte betrModeP    =   A5;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A4;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
#define ENCODER_AKTIV//If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =   A3;//Rotary encoder input for adjustment.
const byte encode2P     =   A2;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV//CV values ​​from V7.0 (10 CV per address)
const int DccAddr           = 317;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//with LocoNet interface this is the LocoNetId
//#define NOACK //Activate this line if there is no hardware to read CV
//(no Ack pin) The pin defined in Interfac.h is then used as OUTPUT
//set, but can be used for any functions in the table below
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   13;//Display operating status (normal/programming) (LED)

#define STATICRISE  (250/50 << 4)//Softled riseTime = 250
//#define COILMOD NOPOSCHK|CAUTOOFF
#define COILMOD     CSTATIC
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT//|sautoback
#define SERVO0MOD   SERVOMOD//Modbyte for follower servo (FSERVO0)
#define SERVOMOD2    SAUTOOFF|NOPOSCHK|SDIRECT |SAUTOBACK
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE//Alternating indicators with both LEDs on when starting
const byte iniTyp[]     =   {    FSERVO,  FSERVO0,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL ,      F2SERVO };
const byte out1Pins[]   =   {       A0,        A1,/*rt*/ 9,/*rt*/10,/*ge*/12,        0,          3 };//output pins of the functions
const byte out2Pins[]   =   {       16,        17,/*gn*/11,/*ws*/ 8,/*gn*/13,        15,         1 };
const byte out3Pins[]   =   {       NC,        NC,/*ge*/ 7,         NC,        NC,       NC,          5 };

const byte iniCVx[10][sizeof(iniTyp)]  = {
/*iniFmode (CV120,130,..*/ { SERVOMOD,0b11100100, NODARKLED,          0,         0,  COILMOD,  SERVOMOD  },
/*iniPar1 (CV121,131,..*/  {       30,       110,    0b01001,    0b10001,      0b01,       50,         30 },
/*iniPar2 (CV122,132,..*/  {       80,       160,    0b00010,    0b00110,      0b01,       50,         80 },
/*iniPar3 (CV123,133,..*/  {       8,          8,          5,          0,        16,        0,          8 },
/*iniPar4 (CV124,134,..*/  {       0,          0,    0b00101,          0,         0,        0,         20  }, 
/*iniPar5 (CV125,135,..*/  {       0,          0,    0b00000,    0b00000,         0,        0,  SERVOMOD2  },
/*iniPar6 (CV126,136,..*/  {       0,          0,          0,          0,      0b11,        0,         20  },
/*iniPar7 (CV127,137,..*/  {       0,          0,         60,          0,         0,        0,        120  },
/*iniPar8 (CV128,138,..*/  {       0,          0,         60,          0,         0,        0,         32  },
/*iniState (CV129,139,..*/ {       0,          0,          0,          0,         0,        0,         10  }//Status values
                            };
//------------------------------------------------------------------------------------
#endif
