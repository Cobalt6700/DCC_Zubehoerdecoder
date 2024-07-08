/*Universal DCC accessory decoder
****************************************************************************************************
*Example file for main light signals (without distant signals on the same mast).
*The pins are designed for the Arduino Nano.
*3 or 5 aspect main signals can be controlled. The number of
*Controllable signals depend on the number of LEDs:
*Without a train control relay, 6 3-term signals (3 LEDs) or 3 4-term signals (5/6 LEDs) can be controlled
*Only the pin assignments need to be adjusted if necessary. All other parameters can
*remain unchanged.
*The LEDs are actively controlled HIGH. Light signals designed for 16V must have an inverting
*ULN2803 (or similar) can be controlled
*The light signal decoder does not have a return channel for reading out the CV values
****************************************************************************************************
*/
//Parameters that can be changed by the user to adapt the accessory decoder to the HW used
//Example of variant with light exit signal with distant signal, with operating mode LED on pin 13 (internal LED)
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
const byte betrModeP    =   A7;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A6;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
//#define ENCODER_AKTIV //If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =   NC;//Rotary encoder input for adjustment.
const byte encode2P     =   NC;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
#define EXTENDED_CV//CV values ​​from V7.0 (10 CV per address)

const int DccAddr          =  17;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
#define NOACK//Activate this line if there is no HW to read CV
//(no Ack pin) The pin defined in Interfac.h is then used as OUTPUT
//set, but can be used for any functions in the table below
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   13;//Display operating status (normal/programming) (LED)

#define MAX_LEDS 16//default is 16. Can be reduced to the number actually used to save RAM.
//19 bytes are required per SoftLED
                   
const byte iniTyp[]     =   { FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0, FSIGNAL2, FSIGNAL0,  FSIGNAL2, FSIGNAL0 };
const byte out1Pins[]   =   {       A0,       NC,       A3,       NC,        3,       NC,        6,       NC,         9,       NC }; 
const byte out2Pins[]   =   {       A1,       NC,       A4,       NC,        4,       NC,        7,       NC,        10,       NC };
const byte out3Pins[]   =   {       A2,       NC,       A5,       NC,        5,       NC,        8,       NC,        11,       NC };
                                                                                                                                  
const byte iniCVx[10][sizeof(iniTyp)]  = {
/*iniFmode (CV120,130,..*/ {        0, 0b000100,        0, 0b000100,        0, 0b000100,        0, 0b000100,         0, 0b000100 },
/*iniPar1 (CV121,131,..*/  { 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001, 0b001001, 0b110001,  0b001001, 0b110001 },
/*iniPar2 (CV122,132,..*/  { 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110, 0b100010, 0b100110,  0b100010, 0b100110 },
/*iniPar3 (CV123,133,..*/  {        0,        0,        0,        0,        0,        0,        0,        0,         0,        0 },
/*iniPar4 (CV124,134,..*/  { 0b000101,        0, 0b000101,        0, 0b000101,        0, 0b000101,        0,  0b000101,        0 }, 
/*iniPar5 (CV125,135,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/*iniPar6 (CV126,136,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/*iniPar7 (CV127,137,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/*iniPar8 (CV128,138,..*/  {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 },
/*iniState (CV129,139,..*/ {       0,          0,       0,        0,        0,        0,        0,        0,         0,        0 }};//Status values
//------------------------------------------------------------------------------------

