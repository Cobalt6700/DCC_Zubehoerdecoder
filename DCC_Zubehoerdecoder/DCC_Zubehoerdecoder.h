//#define KONFIG_FILE "examples\DCC_Zubehoerdecoder_STATIC_COIL.h"  // Path to an alternative config file
//#define KONFIG_FILE "TestKonf\DCC_ZubehoerdecoderV712.h"  // Path to an alternative config file
#if !defined( KONFIG_FILE ) || defined ( EXEC_KONFIG )

// ----------------- DCC Accessory Decoder ---------------------------------------
// This file contains user-modifiable parameters to adapt the accessory decoder
// to the desired functionality and the used hardware.

//#define IFC_SERIAL  Serial  // If this define is active, commands can also be sent via the serial interface
#define SERIAL_BAUD 115200

// Example for variant with light exit signal with distant signal, with operational mode LED on pin 13 (internal LED)
// for Arduino Nano

//----------------------------------------------------------------
// Hardware-dependent constants (not changeable via CV)
//----------------------------------------------------------------

// Analog inputs: (On Nano and Mini versions, A7 and A6 can also be used here to free up more digital usable ports.
//                 On UNO, A7+A6 are not available!)
// #define FIXMODE NORMALMODE    // If this define is active, the operation mode is fixed, betrModeP will then
                        // not be read and will be ignored. Possible values:
                        // NORMALMODE, POMMODE, INIMODE, ADDRMODE
const byte betrModeP    =   A7;     // Analog input for determining the operating mode. Only read at program start!
const byte resModeP     =   A6;     // Reset CV values + servo center position

// Digital inputs (ports A0-A5 can also be used digitally): ---------

// Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE  // Properties of the rotary encoder (impulses per detent)
#define ENCODER_AKTIV       // If this line is commented out, the encoder is not used.
                            // The encoder ports are then ignored and can be used otherwise.
const byte encode1P     =   A5;     // Input for rotary encoder adjustment.
const byte encode2P     =   A4;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Operating values (changeable via CV) These data are only written to the CVs during initialization mode.
// Initialization mode can be activated via mode input or automatically if no valid values are in CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV       // CV values from V7.0 (10 CVs per address)

const int DccAddr           =  17;    // DCC decoder address
const byte iniMode          = AUTOADDR /*| ROCOADDR*/;  // Default operating mode (CV47)
const int  PomAddr          = 50;    // Address for programming on main ( CV48/49 )
                                    // with LocoNet interface, this is the LocoNetId
//#define NOACK                     // Activate this line if there is no hardware for CV readout
                                    // (no Ack pin) The pin defined in Interfac.h will then be set as OUTPUT,
                                    // but can be used for any function in the table below

// Outputs: Outputs marked with NC are not assigned to any port. This allows saving ports,
//          e.g., if no polarization relay is needed for a servo.
const byte modePin      =   13;     // Display operational state (Normal/Programming) (LED)

#define STATICRISE  (250/50 << 4) // Softled riseTime = 250 ( max = 750 )
#define COILMOD     NOPOSCHK|CAUTOOFF
#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT //|SAUTOBACK
#define SERVO0MOD   SERVOMOD    // Modbyte for follow-up servo (FSERVO0)
#define STATICMOD   CAUTOOFF|BLKSOFT|BLKSTRT|STATICRISE    // Alternating flasher with both LEDs on at startup            
const byte iniTyp[]     =   {    FSERVO,  FSERVO0,   FSIGNAL2,   FSIGNAL0,   FVORSIG,   FCOIL };
const byte out1Pins[]   =   {       A2,        A3,   /*rt*/ 9,   /*rt*/10,  /*ge*/A0,        5 };  // Output pins of the functions
const byte out2Pins[]   =   {        3,        12,   /*gn*/11,   /*ws*/ 8,  /*gn*/A1,        6 };
const byte out3Pins[]   =   {       NC,        NC,   /*ge*/ 7,         NC,        NC,       NC };

const byte iniCVx[10][sizeof(iniTyp)]  = {
/* iniFmode (CV120,130,..*/ { SERVOMOD,0b11100100,          0,          0,         0,  COILMOD },
/* iniPar1 (CV121,131,..*/  {       30,       110,    0b01000,    0b10001,      0b01,       50 },
/* iniPar2 (CV122,132,..*/  {       80,       160,    0b00010,    0b00110,      0b10,       50 },
/* iniPar3 (CV123,133,..*/  {       8,          8,          5,          0,        16,        0 },
/* iniPar4 (CV124,134,..*/  {       0,          0,    0b00101,          0,         0,        0 }, 
/* iniPar5 (CV125,135,..*/  {       0,          0,    0b01001,          0,         0,        0 },
/* iniPar6 (CV126,136,..*/  {       0,          0,          0,          0,         0,        0 },
/* iniPar7 (CV127,137,..*/  {       0,          0,         50,          0,         0,        0 },
/* iniPar8 (CV128,138,..*/  {       0,          0,        100,          0,         0,        0 },
/* iniState (CV129,139,..*/ {       0,          0,          0,          0,         0,        0 }}; // Status values
//------------------------------------------------------------------------------------
#endif
