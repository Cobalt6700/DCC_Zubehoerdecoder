/* Universal DCC - accessories code
***********************************************************************************
* Example file for L i c h t h a u p t s i g n a l e (without pre -signals on the same mast).
* The pins are designed for the Arduino Nano.
* 3- or 5-term main signals can be controlled.The number of
* Control signals depends on the number of LEDs:
* Without train influence relay, 6 3-term (3 LEDs) or 3 4-term signals (5/6 LEDs) can be controlled
* The pin assignments may only have to be adjusted.All other parameters can
* remain unchanged.
* The LEDs are actively controlled.Light signals designed for 16V must be via an inverting
* Uln2803 (or similar) can be controlled
* With the light signal decoder, no return channel is intended for reading out the CV values
***********************************************************************************
*/
// #define IFC_SERIAL 	Serial
// parameter that can be changed by the user to adapt the accessory code to the HW used
#define SERIAL_BAUD 115200

// Example of variant with a light outfing signal with a pre-signal, with operating fashion LED on PIN 13 (internal LED)

//----------------------------------------------------------------
// Hardware -dependent constant (not changing via CV)
//----------------------------------------------------------------

// inputs analog: (A7 and A6 can also be used here for nano and mini versions to more
// to get digitally usable ports.
// A7+A6 are not available for the UN!)))
const byte betrModeP    =   PIN_PA4;     // D5 Analog input for determining operating mode.Only at the
                                    // Read the program!
const byte resModeP     =   PIN_PA5;     // D3 Reset CV values ​​+ middle position servos

// Digital inputs (the ports A0-A5 can also be used digitally): ---------

// Turning code for servo adjustment ...........
#define ENCODER_DOUBLE  // Properties of the rotary code (impulses by resting)
//#define ENCODER_AKTIV       // If this line is commented on, the encoder is not used.
                            // The encoder ports are then ignored and can be otherwise
                            // be used.
const byte encode1P     =   NC;     // Entrance code for adjustment.
const byte encode2P     =   NC;
// ............................................
//-------------------------------------------------------------------------------------------------------
// Operating values ​​(changed via CV) This data is only written in the CV's in the initiation mode.
// The initiation mode can be activated by fashion or it is automatically active if none
// stand meaningful values ​​in the CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV       // CV-Werte ab V7.0 ( 10 CV per Adresse )

const int DccAddr          =  201;    // DCC-Decoderadresse
const byte iniMode          = 0x50  /* | AUTOADDR */ /* | ROCOADDR*/;  // default-Betriebsmodus ( CV47 )
const int  PomAddr          = 51;    // Adresse für die Pom-Programmierung ( CV48/49 )
// #define NOACK                     // Activate this line if there is no HW to read for CV
// (no ack pin) The PIN defined in interfac.h is then an output
// set, but can be used for any functions in the table below


// Ausgänge:  mit NC gekennzeichnete Ausgänge werden keinem Port zugeordnet. Damit können Ports gespart werden,
//            z.B. wenn bei einem Servo kein Polarisierungsrelais benötigt wird
const byte modePin      =   PIN_PA7;     // Anzeige Betriebszustand (Normal/Programmierung) (Led)

#ifdef MAX_LEDS
#undef MAX_LEDS
#define MAX_LEDS 3// default ist 16. Kann auf die tatsächlich benutzte Zahl reduziert werden, um RAM zu sparen.
                    // Pro Softled werden 19 Byte benötigt
#endif

#define OUT1 PIN_PC0
#define OUT2 PIN_PC1
#define OUT3 PIN_PC2
#define OUT4 PIN_PC3
#define OUT5 PIN_PA1
#define OUT6 PIN_PA2
#define OUT7 PIN_PA3
#define OUT8 PIN_PB0
#define OUT9 PIN_PB1

#define SERVOMOD    SAUTOOFF|NOPOSCHK|SDIRECT     

const byte iniTyp[]     =   { FSERVO,   FSERVO,   FSERVO,  FSIGNAL2,  FSIGNAL2,  FSIGNAL2,  FSERIAL };//FSTATIC };
const byte out1Pins[]   =   {   OUT1,     OUT2,     OUT3,      OUT4,      OUT6,      OUT8,       NC };// DIMMER };
const byte out2Pins[]   =   {     NC,       NC,       NC,      OUT5,      OUT7,      OUT9,       NC };//     NC };
const byte out3Pins[]   =   {     NC,       NC,       NC,        NC,        NC,        NC,       NC };//     NC }; 
                                                                                                                                  
const byte iniCVx[10][sizeof(iniTyp)]  = {
/* iniFmode (CV120,130,..*/ { SERVOMOD, SERVOMOD, SERVOMOD, 0b1000000, 0b1000000,  0b1000000, 0b0000001  },
/* iniPar1 (CV121,131,..*/  {       30,       30,       30,  0b000001,   0b000001,  0b000001,         1  },
/* iniPar2 (CV122,132,..*/  {      118,      118,      118,  0b000010,   0b000011,  0b000010,         0  },
/* iniPar3 (CV123,133,..*/  {      100,      100,      100,         0,          0,         0,       255  },
/* iniPar4 (CV124,134,..*/  {        0,        0,        0,         0,          0,         0,       124  }, 
/* iniPar5 (CV125,135,..*/  {        0,        0,        0,         0,          0,         0,         0  },
/* iniPar6 (CV126,136,..*/  {        0,        0,        0,         0,          0,         0,         0  },
/* iniPar7 (CV127,137,..*/  {        0,        0,        0,         0,          0,         0,         0  },
/* iniPar8 (CV128,138,..*/  {        0,        0,        0,         0,          0,         0,         0  },
/* iniState (CV129,139,..*/ {        0,        0,        0,         0,          0,         0,         0  }}; // Status-Werte
//------------------------------------------------------------------------------------

