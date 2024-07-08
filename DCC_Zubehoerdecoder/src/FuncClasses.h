/*DIY accessory decoder
*
*Classes for the individual functions FSERVO, FCOIL, FSIGNAL, FSTATIC
*A suitable object must be instantiated for each function defined in the config file.
*Instantiation must be done in setup() with 'new'.
*/
#include <MobaTools.h>
#include "../Interface.h"

#define PPWA 3//Number of pins per switch address
//Offset of the CV address for the function-specific CV values
enum :byte { MODE=0, PAR1, PAR2, PAR3, PAR4, PAR5, PAR6, PAR7, PAR8, STATE} ;

//If the classes are not used in conjunction with the nmradcc Lib,
//Access to the configuration variables can be adjusted here
//(if necessary also direct EEPROM access)
#define NMRA
#ifdef NMRA
    #define getParam( parAdr ) ifc_getCV( _cvAdr+(parAdr) )
    #define setParam( parAdr, value ) ifc_setCV( _cvAdr+(parAdr), value )
    #define setState( value )  ifc_setCV( _cvAdr+STATE, value )
#else
//Access to the config parameters directly via EEPROM:
//(not possible with LocoNet Interface)
    #include <EEPROM.h>
    #define getParam( parAdr ) EEPROM.read( _cvAdr+(parAdr) )
    #define setParam( parAdr, value ) EEPROM.update( _cvAdr+(parAdr), value )
    #define setState( value )  EEPROM.update( _cvAdr+STATE, value )
#endif
//====================== general help functions ========================== ========
//Hide unused (NC) ports
/*moved to interface.h
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode );
#else
void _pinMode( byte port, byte mode );
#endif

void _digitalWrite( byte port, byte state ) ;
*/
//========================= Function classes ======================= =================
//----------------------FCOIL -----------------------------------------
//Flags for CV 'MODE'
#define CAUTOOFF 0x01//The pulse duration is limited internally
#define CINVERT  0x02//Output pin 3 inverted (if active)
#define CSTATIC  0x04//Pin1/2 can be switched independently (both can also be active)
//no shutdown via timer, ON/OFF only via DCC
#define NOPOSCHK 0x08//The outputs also respond to a command if the current
//Position is not changed.
 
 class Fcoil {
     public:
    Fcoil( int cvAdr, uint8_t out1P[] );
    void process();
    void set( uint8_t sollWert, uint8_t outState );//received new switching command

    
    private:
    MoToTimer _pulseT;
    struct {
        bool pulseON   :1 ;//Flag whether pause timer is running
        bool sollOut   :1;
        bool sollCoil  :1;//Coil to be controlled
        bool sollAct   :1;//Setpoint has not yet been adopted
        bool istCoil   :1;
    }_flags;
    #define GERADE  0x0//Bit 0
    #define ABZW    0x1//Bit 0
    
    uint16_t _cvAdr = 0;//Address of the CV block with the function parameters
    uint8_t *_outP;//Output pins

 };

//----------------------FSTATIC ----------------------------------------------------------
//Flags for CV 'MODE':
#define BLKMODE 0x01//Outputs flash
#define BLKSTRT 0x02//Start with both outputs ON
#define FSTAINV 0x02//Invert pin in extended mode
#define BLKSOFT 0x04//Outputs as soft LEDs
#define MODEOFFS   3//CV's per pin in extended mode
 class Fstatic {
//Static or flashing control of LEDs
    public:
    Fstatic( int cvAdr, uint8_t ledP[], bool extended=false  );
    void process( );
    void set( bool sollWert );//received new switching command

    private:
    void _setLedPin( uint8_t ledI, bool sollWert );
    MoToTimer _pulseT[3];
	
    uint16_t _cvAdr;//Address of the CV block with the function parameters
    SoftLed *_ledS[3] = { NULL, NULL, NULL };//Softled objects
    uint8_t *_ledP;//Pins of the LEDs
	uint8_t _pinStat = 0;//bit-coded pin status
    struct {
        bool blkOn :1;//flashing LED is ON
        bool isOn  :1;//Function is switched on
		bool extended :1;//true= extended mode with max 3 pins
    } _flags;       
    
 };

//-----------------------FSERVO -------------------------------------------
//Flags for CV 'MODE'
#define SAUTOOFF 0x01//Pulses are switched off after reaching the end position
#define SDIRECT  0x02//The servo also responds to a switching command during movement
#define NOPOSCHK 0x08//The outputs also respond to a command if the current
//Position is not changed.
#define SAUTOBACK 0x04//Servo automatically returns to its initial position (after time has elapsed)
#define SAUTOTIME 2000//Default value (if state parameter = 0)
#define F2OFFSET    5//Offset for the parameters of the 2nd servo on an address

class Fservo {
    public:
    Fservo( int cvAdr, uint8_t pins[], uint8_t posZahl, int8_t modeOffs=MODE );
    void process();
    void set( uint8_t sollWert );//received new switching command
	bool isMoving ();//Query whether servo is moving
	uint8_t getPos();//current position number of the servo
    uint8_t getCvPos();//Determine the CV value for the current position
	void adjust( uint8_t mode, uint8_t value );//Change servo parameters
		#define ADJPOS		0//for end-age adjustment: value = new position
		#define ADJPOSEND	1//Adopt new end position in CV
		#define ADJSPEED	2//Change servo speed
    void center( uint8_t mode );
		#define ABSOLUT	 0//Set servo to 90°
		#define RELATIVE 1//Center position between the programmed end points
        
    private:
//offset for the CV's of the position values
    static const uint8_t posOffset[6];
    MoToServo  _weicheS;
    MoToTimer  _autoTime;//for automatic reversing
    uint16_t _cvAdr = 0;//Address of the CV block with the function parameters
    int8_t   _modeOffs = 0;//Offset of the CV address for the mode byte. Normally this is 0
//With the servo combination for 3 gr. Form signals but -CV_BLKLEN for the 2nd servo,
//because it also accesses the ModeByte of the 1st servo.
    int8_t  _parOffs = 0;//Offset of the CV address for the position and speed parameters. At 2
//Servos on one address contain the parameters of the 2nd servo starting at PAR4
//The offset is passed to the constructor via modeOffs:
//modeOffs < 0: Mode offset for 2nd servo to subsequent address
//modeOffs > 0: Parameter offset for 2nd servo at the same address
    uint8_t *_outP;//Array with pins of the outputs
		#define SERVOP	0
		#define REL1P	1
		#define REL2P	2
    uint8_t _posZahl;//Number of positions that can be approached (2 or 4)
    uint8_t _sollPos;//Target position of the servo (0...3)
    uint8_t _istPos;//is the position of the servo
    struct {
        bool relOn    :1 ;//Initial state of the relays
        bool moving   :1 ;//Servo in motion
        bool sollAct  :1 ;//Setpoint has not yet been adopted
    } _flags;
 
 };
//-----------------------SIGNAL -------------------------------------------
//Constant for light signal function
//CV MODE flags:
#define LEDINVERT 0x80//SIGNAL: Invert Softled outputs (bit 7 of the mode byte of SIGNAL 2/3)
#define NODARKLED 0x40//When the image changes, LEDs that are active in both images are not switched off
const byte  LSMODE=0,                 BILD1=1,              BILD2=2, VORSIG=3,  DARKMASK = 4,//Parameter 1.Address
                                     BLINK1=5,             BLINK2=6, 
                                     BLINKTAKT1 = 7,       BLINKTAKT2 = 8,
            SOFTMASK2 = 0+CV_BLKLEN,  BILD3=1+CV_BLKLEN,    BILD4=2+CV_BLKLEN,//Parameter 2. Address
                                     BLINK3=5+CV_BLKLEN,   BLINK4=6+CV_BLKLEN, 
            SOFTMASK3=0+(2*CV_BLKLEN),BILD5=1+(2*CV_BLKLEN),BILD6=2+(2*CV_BLKLEN),//Parameter 3. Address
                                     BLINK5=5+(2*CV_BLKLEN),BLINK6=5+(2*CV_BLKLEN);
            
#define SIG_DARK_TIME   400//Time between darkening and showing the new signal image
#define SIG_RISETIME    400//Fade up/fade out time
 
 class Fsignal {
    public:
    Fsignal( int cvAdr, uint8_t pins[], uint8_t adrAnz, Fsignal** vorSig );
//addrAnz is the number of addresses that the signal occupies. This results in
//also the number of CV parameters and the number of pins
    void process();//must be called regularly in loop()
    void set( uint8_t sollWert );//get new signal command
    void setDark  ( bool darkFlg );//'true' switches the signal off (dark), 'false' switches it on
    
    private:
    void    _clrSignal (byte);//Switch off SoftLeds -Bits that are set in the parameter are not switched off
    void    _setSignal ();//switch on the current signal image
    void    _setSignalStatic ();//switch on the current signal image (static LEDs)
    void    _setSignalBlink ();//switch current signal image (blionkede LEDs)
    uint8_t _getHsMask ();//Determine mask for hard/soft switching of all outputs
    uint8_t _getSigMask( uint8_t ) ;//Determine the bit mask of the outputs for the current signal image
//the static mask is returned (for testing on 0xff)
    
    Fsignal **_vorSig;//Pointer to distant signal on the same mast
    MoToTimer darkT;//Dark time when blending between signal images
    MoToTimer _blinkT;//Timer for flashing signal images
    uint16_t _cvAdr = 0;//Address of the CV block with the function parameters
    uint8_t  _pinAnz;//Number of assigned output pins: 3(PPWA) per CV block
    uint8_t *_outP;//Array with pins of the outputs
    SoftLed **_sigLed;//Pointer to array of Softled objects
    struct {
        byte state   :2;//Status of the internal state machine
        byte sigBild :3;//current signal image corresponding to the last setpoint
        byte dark    :1;//Signal is currently darkened
        byte isVorSig:1;//the object is a distant signal
        byte blinkTakt:1;//Status of the flashing cycle (starts with true)
    } _fktStatus;//internal statuses
    
    struct {//LED masks for the current signal image
        byte staticLed;//statically lit LEDs
        byte blnkStd;//flashing LEDs /start with ON
        byte blnkInv;//flashing LEDs /start with OFF
    } _sigMask;
    
//current signal state
    #define SIG_WAIT        0//Wait for signal commands
    #define SIG_NEW         2//show new signal image

 
 
 };
