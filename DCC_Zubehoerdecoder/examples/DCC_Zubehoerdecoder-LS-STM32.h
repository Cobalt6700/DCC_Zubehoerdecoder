//-----------------DCC accessory encoder -----------------------------------------
//This file contains the user-changeable parameters to connect the accessory decoder
//to adapt the desired functionality and the HW used
//Example of a pure light signal decoder
//for STM32F103
///////////////////////////////////////////////////////////////////////
//
//Example for Maple-Mini (STM32F1 processor)
//
///////////////////////////////////////////////////////////////////////
//Parameters that can be changed by the user to adapt the accessory decoder to the HW used
//Example of variant with 4 servos + 3 static outputs, with operating mode LED on pin 13 (internal LED)
//----------------------------------------------------------------
//Hardware-dependent constant (cannot be changed via CV)
//----------------------------------------------------------------
//Analogue inputs: (For Nano and Mini versions, A7 and A6 can also be used here for more
//to free up digitally usable ports.
//A7+A6 are not available on the UNO! )
const byte betrModeP    = PA7;//Analog input to determine the operating mode. Will only be at
//Program start read in!
const byte resModeP     = PA6;//Reset CV values ​​+ center position servos
//Digital inputs (ports A0-A5 can also be used digitally): ---------
//Rotary encoder for servo adjustment ...........
//#define ENCODER_DOUBLE //Properties of the rotary encoder (impulses per detent position)
//#define ENCODER_AKTIV //If this line is commented out, the encoder will not be used.
//The encoder ports are then ignored and can be used otherwise
//be used.
const byte encode1P     =  NC;//Rotary encoder input for adjustment.
const byte encode2P     =  NC;
//..........................................
//-------------------------------------------------------------------------------------------------------
//Operating values ​​(can be changed via CV) These data are only written into the CVs in initiation mode.
//The initiation mode can be activated via the mode input or it is automatically active if none
//meaningful values ​​are in CV47.
//-------------------------------------------------------------------------------------------------------
const int DccAddr          =  17;//DCC decoder address
const byte iniMode          = 0x50 | AUTOADDR/*| ROCOADDR*/;//default operating mode (CV47)
const int  PomAddr          = 50;//Address for Pom programming (CV48/49)
//#define NOACK //Activate this line if there is no hardware to read out the CV
//(no Ack pin) The pin defined in Interfac.h is then used as OUTPUT
//set, but can be used for any functions in the table below
//Outputs: Outputs marked NC are not assigned to any port. This allows ports to be saved
//e.g. if a polarization relay is not required for a servo
const byte modePin      =   33;//Display operating status (normal/programming) (LED)

#define MAX_LEDS 12//default is 16. Can be reduced to the number actually used to save RAM.
//19 bytes are required per SoftLED

const byte iniTyp[]     =   { FSIGNAL2,   FSIGNAL2, FSIGNAL0,    FSIGNAL2, FSIGNAL0,  FSIGNAL0,   FVORSIG,  FSIGNAL0,   FSIGNAL2 };
const byte out1Pins[]   =   {      PA2,        PB3,      PB6,         PB7,     PA10,      PA14,      PC14,      PA1,        PA15 };//output pins of the functions
const byte out2Pins[]   =   {      PA3,        PB5,       NC,         PA8,     PC13,        NC,      PC15,       NC,        PB12 };
const byte out3Pins[]   =   {       NC,        NC ,       NC,         PA9,       NC,        NC,       PA0,       NC,          NC };
                                                                                     
const byte iniFmode[]     = {        0,         0, 0b00000000, 0b0000000, 0b00000000,0b00000000,        0,        0,  0b00000000 };
const byte iniPar1[]      = {   0b0001, 0b0000001, 0b00001010, 0b0001100, 0b00000110,0b01000000,   0b0101,   0b1001,      0b0001 };
const byte iniPar2[]      = {   0b0010, 0b0001000, 0b00000000, 0b0010000, 0b00010001,0b00000000,   0b1010,      255,      0b0010 };
const byte iniPar3[]      = {        0,         0,          0,         7,          0,         0,       15,       16,           0 };
const byte iniPar4[]      = {        0, 0b0000000,          0, 0b0000101,          0,         0,        0,        0,           0,};//only for light signals!
/*
const byte iniTyp[] = { FCOIL, FSIGNAL2, FSIGNAL0, FVORSIG, FSERVO, FSTATIC };
const byte out1Pins[] = { A2, 9, 12, 7, A1, 5 };  //output pins of the functions
const byte out2Pins[] = { A3, 10, NC, 8, 3, 6 };
const byte out3Pins[] = { NC, 11, NC, NC, NC, NC };

//Function-specific parameters. These parameters start at CV 50 and there are per function output
//5 CV values. The first 4 values ​​control the behavior and the following table are initial initiation values
//included for these CV's. The 5th value is for internal purposes and is not initiated here
//In the 'INIMode' operating mode, the mode and parx values ​​are taken from the following table at every start
//The table values ​​must be adapted to the type distribution (iniTyp, see above).
const byte iniFmode[] = { CAUTOOFF, LEDINVERT, 0b00000100, 0, 0, BLKMODE|BLKSOFT };
const byte iniPar1[] = { 50, 0b0000010, 0b00000100, 0b0001, 0, 50 };
const byte iniPar2[] = { 50, 0b0000001, 0b00001001, 0b0010, 180, 50 };
const byte iniPar3[] = { 0, 4, 8, 8, 8, 100 };
const byte iniPar4[] = { 0, 0b0000101, 0, 0, 0, 0,}; //only for light signals!

//-------------------------------------------------------------------------------------
/*The following values ​​serve as examples of sensible entries in the parameter table above.
//They are not used directly in the program!
//Default values ​​for servo output
const byte iniServoMode = SAUTOOFF;     //= (Mode) automatic pulse shutdown
const byte iniServoEven = 0;     //= Par1;
const byte iniServoAbzw = 180;   //= Par2;
const byte inispeed = 8;             //= Par3;

//Default values ​​for pulse output (double coil)
const byte iniCoilMode = CAUTOOFF;    //= (Mode) automatic pulse limitation switched on (0=OFF)
const byte iniCoilOn = 50;    //= (Par1) 500ms pulse
const byte iniCoilOff = 20;    //= (Par2) at least 2sec pause between 2 pulses

//Default values ​​for static/flashing outputs
//if 2 outputs are defined (out1Pins, out2Pins), they work in push-pull mode.
const byte iniStaticMode = BLINKMODE; //Bit0: Flashing/Static
//Bit1: When flashing, both LEDs start flashing alternately
//Bit2: with soft fading up/down (pins must be PWM capable)
const byte iniBlinkOn = 100;   //= (Par1) 1sec
const byte iniBlinkOff = 50 ;   //= (Par2) 0.5sec
*/
