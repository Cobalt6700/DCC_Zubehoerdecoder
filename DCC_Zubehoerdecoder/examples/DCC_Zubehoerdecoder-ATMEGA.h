//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used
//Example of variant with light signal and static/flashing LED outputs
//for Arduino UNO/Nano/Micro

#define ENCODER_DOUBLE//Properties of the rotary encoder (impulses per detent position)
//Parameters that can be changed by the user to adapt the accessory decoder to the HW used
//Example of variant with light exit signal with distant signal, with operating mode LED on pin 13 (internal LED)
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
const byte betrModeP    =   A5;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     =   A4;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
//#define ENCODER_AKTIV //If this line is commented out, the encoder will not be used.
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
const int DccAddr          =  17;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved,
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   13;//Display operating status (normal/programming) (LED)

#define MAX_LEDS 12//default is 16. Can be reduced to the number actually used to save RAM.
//19 bytes are required per SoftLED
                    
const byte iniTyp[]     =   {    FCOIL,   FSIGNAL2, FSIGNAL0,   FVORSIG,  FSIGNAL0,          FSTATIC };
const byte out1Pins[]   =   {       NC,          9,       12,        5,        8,               A0 };//output pins of the functions
const byte out2Pins[]   =   {       NC,         10,       NC,        6,       NC,                3 };
const byte out3Pins[]   =   {       NC,         11,       NC,        7,       NC,               NC };
 
const byte iniFmode[]     = { CAUTOOFF, LEDINVERT, 0b00000100,        0,        0,  BLKMODE|BLKSOFT };
const byte iniPar1[]      = {       50, 0b0000010, 0b00000100,   0b0101,   0b1001,               50 };
const byte iniPar2[]      = {       50, 0b0000001, 0b00001001,   0b1010,      255,               50 };
const byte iniPar3[]      = {        0,         4,          8,        8,        9,              100 };
const byte iniPar4[]      = {        0, 0b0000101,          0,        0,        0,                0,};//only for light signals!
//------------------------------------------------------------------------------------

