/*Accessory decoder interface
**All interface-dependent (Loconet or DCC) program components are listed here
**summarized, and neutral calls for the functionalities in the sketch are provided.
*/

#ifndef INTERFACE_H
#define INTERFACE_H

#include <inttypes.h>
#include <Arduino.h>
#include "src/DebugDefs.h"

//#define LOCONET //If this is commented out, a DCC interface will be integrated
//################################################## ################################################## ##
#define NC 0xff//Port NC can be assigned to unused function outputs.
//Since the check for invalid pin numbers in the Arduino internal implementations depends on the processor
//is different, the sketch itself checks for NC and, if necessary, the Arduino function is not called.
//----------defines for LocoNet interface ----------------------------------------------------------
#ifdef LOCONET
    const uint8_t txPin        =   2;
//The received signal MUST be on pin 4 (for Micro/Leonardo) or pin 48 (for Mega).
    
    void ifc_init( uint8_t cvPomLow ) ;
    
//-----------defines for DCC interface -----------------------------------------------------------
#else
    #if defined ( ARDUINO_MAPLE_MINI )
//Definitions for the MapleMini (with STM32 processor)
        const byte dccPin       =   3;
        const byte ackPin       =   18;
    #elif defined (ARDUINO_GENERIC_STM32F103C)
//Definitions for generic STM32 boards
        const byte dccPin       =   PB0;
        const byte ackPin       =   PB4;
    #else
//Definitions for the 'standard' Arduinos (UNO, Nano, Mega, Micro, Leonardo)
        const uint8_t dccPin       =   2; //PIN_PA6
        const uint8_t ackPin       =   4; //PIN_PB5
    #endif
#endif

//-----------general definitions for both interfaces ----------------------------------------------
//Modes for progMode
extern byte progMode;
#define NORMALMODE  0
#define ADDRMODE    1//Wait for the 1st telegram to determine the first switch address (Prog LED flashes)
#define PROGMODE    2//Address received, POM programming possible (Prog LED active)
#define POMMODE     3//Allways-Pom Mode (no Prog-Led)
#define INIMODE     4//Like normal mode, but the standard CV values ​​and CV47-49 are used for each
//Start loaded from the defaults
#define INIALL      5//All CV's must be initiated
//..................Definition of CV addresses ................................. ....
#define CV_INIVAL     45//Valid flag
#define CV_MODEVAL    47//Initiation CV, bits 0..3 for operating modes
//Since the SV addresses (LocoNet) and the CV addresses (DCC) are offset by two from the respective libs
//are addressed, the EEPROM values ​​are invalid when changing the interface. That's why in this one
//In this case the EEPROM needs to be re-initiated. Because of the offset by 2, the valid flag is set to 2
//Memory cells are written so that the valid flag of the other interface is safely destroyed.
//VALID flag has been changed from V7 so that the CV's are initiated when changing from V6 <-> V7 (different distribution)
#ifdef LOCONET
    #define VALIDFLG  0xB0//If the interface changes, everything has to be re-initiated.
//Low Nibble must be 0 (due to MODEVAL bits)
#else
    #define VALIDFLG  0x60
#endif

#define CV_POMLOW     48//Address for POM programming
#define CV_POMHIGH    49
#define CV_ADRZAHL    50//Number of managed addresses (cannot be changed)
#define CV_INITYP     51//CV51 ff contain the function types of the addresses (cannot be changed)
#define CV_FUNCTION  120//Start the function configuration blocks
#define CV_BLKLEN     10//Length of a CV block (one block per address)
//The meaning is largely function specific

extern const byte modePin;
#define SET_PROGLED digitalWrite( modePin, HIGH )
#define CLR_PROGLED digitalWrite( modePin, LOW )

extern const uint8_t cvAccDecAddressLow;
extern const uint8_t cvAccDecAddressHigh;
extern const uint8_t cvVersionId;
extern const uint8_t cvManufactId;
extern const uint8_t cv29Config;
extern const uint8_t config29Value;
extern const uint8_t config29AddrMode;
extern const uint8_t manIdValue;

void ifc_notifyDccAccState( uint16_t Addr, uint8_t OutputAddr, uint8_t State );
void ifc_notifyCVAck ( void );
void ifc_notifyCVChange( uint16_t CvAddr, uint8_t Value );
void ifc_notifyCVResetFactoryDefault(void);
void ifc_notifyDccReset( uint8_t hardReset );

void ifc_init( uint8_t version, uint8_t progMode, uint8_t cvPomLow );

uint8_t ifc_getCV( uint16_t address );
void ifc_setCV( uint16_t address, uint8_t value );
uint16_t ifc_getAddr();
void ifc_process();

//====================== general help functions ========================== ========
//Hide unused (NC) ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode );
#else
void _pinMode( byte port, byte mode );
#endif

void _digitalWrite( byte port, byte state ) ;



#endif
