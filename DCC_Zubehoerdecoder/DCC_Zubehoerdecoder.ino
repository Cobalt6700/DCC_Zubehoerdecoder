/*Universal DCC decoder for switches and (light) signals
Version 7.0.0 -The functionalities are defined as classes
The class objects are only instantiated in setup depending on the configuration

Characteristics:
Several (successive) accessory addresses can be controlled
The maximum number of addresses essentially depends on the number of digital outputs
(max 16 servos can be configured)
1. Address adjustable via programming

3 outputs /accessory address
Adjustable functionality:
-Servo with switching relay for turnout polarization
-Logical coupling of 2 servos for 3-aspect form signals
-1 servo via 2 addresses to control 4 positions
-2 servos via one address (parallel control of servos)
-Impulse function for servos (automatic return to initial position,
e.g. for uncouplers)
-Double coil drives
-static outputs
-flashing outputs
-Light signal functions

The functionality and IO assignment is determined via tables in the h-file.
The configuration of the individual functions is done via CV programming.
For example, for servo outputs the end positions can be set using a CV value; for light signals this is
Assignment of the output states to the signal state can be configured.
*/
#define DCC_DECODER_VERSION_ID 0x71

// CONFIG FILE DEFINES HERE

#include "src/FuncClasses.h"
#ifdef __AVR_MEGA__
#include <avr/wdt.h>    //for soft reset (via watchdog)
#endif

//------------------------------------------//


#ifdef __STM32F1__
//is now defined in core: #define digitalPinToInterrupt(x) x
    #define MODISTEP    4096/6//Limit values ​​at the analog input of the operating modes
#else
    #define MODISTEP    1024/6
#endif
#define uint_t unsigned int


//Limit values ​​of the analog input for the respective operating modes (entire range 0...1024):
#define ISNORMAL    MODISTEP*5//> 853 is considered normal operation
#define ISPOM       MODISTEP*3//<853, >512 Allways Pom
#define ISOPEN      MODISTEP//<512, >170 IniMode: initiate all function parameters at startup
#define ISPROG      0//<170 programming mode (address recognition)
//Possible function types per output.
#define FOFF        0//Function output switched off
#define FSERVO      1//Standard servo output
#define FCOIL       2//Magnetic items
#define FSTATIC     3//The output is switched on or off statically/flashing
#define FSIGNAL0    4//Follow-up address for signals
#define FSIGNAL2    5//1. Signal address
#define FVORSIG     6//1. distant signal address
#define FSERVO0     7//Follow-up address for 2 coupled servos
#define F2SERVO     8//Class for controlling 2 servos via one address
#define FSTATIC3    9//3 outputs can be switched on or off statically/flashing
#define FMAX        9  

//--------------------------------------
//Flags for iniMode:
#define AUTOADDR    1//Automatic address recognition after initial initiation or when programming mode is active
#define ROCOADDR    2//0: Output address 4 is switch address 1
//1: Output address 0 is switch address 1
//------------------------------------------------------
//------------------Incorporating the configuration file ------------------------
#ifdef __STM32F1__
  #include "DCC_Zubehoerdecoder-STM32.h"
  #ifndef KONFIG_FILE
  #pragma message "\n\rbenutztes Konfig-File: DCC_Zubehoerdecoder-STM32.h"
  #endif
#elif defined(__AVR_ATmega32U4__)
  #include "DCC_Zubehoerdecoder-Micro.h"
  #ifndef KONFIG_FILE
  #pragma message "\n\rbenutztes Konfig-File: DCC_Zubehoerdecoder-Micro.h"
  #endif
#else
  #include "DCC_Zubehoerdecoder.h"
  #ifndef KONFIG_FILE
  #pragma message "\n\rbenutztes Konfiig-File DCC_Zubehoerdecoder.h"
  #endif
#endif
#ifdef KONFIG_FILE
  #define STRING2(x) #x
  #define STRING(x) "\n\rBenutztes Konfig-File: "  STRING2(x)
  #pragma message (STRING(KONFIG_FILE))
  #define EXEC_KONFIG 1
  #include KONFIG_FILE
#endif
//-------------------------------------------------------------------------------
//-------------------------------------------
const byte weichenZahl = sizeof(iniTyp);

#define cvParAdr(wIx,par)      (uint16_t)CV_FUNCTION+CV_BLKLEN*(wIx)+(par)
#define getCvPar(wIx,par)   ifc_getCV( cvParAdr(wIx,par) )
#define cvEAdr(wIx,epar)    CV_EXTDATA+epar+CV_ERWLEN*wIx  
#define getCvExtPar(wIx,epar) ifc_getCV( cvEAdr(wIx,epar) )

//CV default values ​​of the standard addresses:
struct CVPair {
  uint16_t  CV;
  uint8_t   Value;
};
CVPair FactoryDefaultCVs [] =
{
  {cvAccDecAddressLow, DccAddr%256},
  {cvAccDecAddressHigh, DccAddr/256},
  {cvVersionId, DCC_DECODER_VERSION_ID},
  {cvManufactId, manIdValue},
  {cv29Config, config29Value},
};

//----------------------Variable -------------------------------------------------
byte opMode;//Bit 0..3 from modeVal
byte rocoOffs;//4 for ROCO addressing, 0 otherwise
//byte isOutputAddr=1;            //Flag whether output addressing from 6.1 is always Ouput address
word weichenAddr;//Address of the 1st switch (of the entire block)
byte ioPins[PPWA*weichenZahl];//all defined IO's in a linear array
//Structure for 2 servos at one address (F2SERVO)
typedef struct {
    Fservo  *servo1;
    byte    pins1[3];//Pin assignment for 1st servo
    Fservo  *servo2;
    byte    pins2[3];//Pin assignment for 2nd servo
} F2Servo_t;

//Pointer to the function objects
union {//There is an array for each class, but they are on top of each other because they are per switch address
//only one object is possible
    F2Servo_t  *twoServo[weichenZahl];
    Fservo  *servo[weichenZahl];
    Fcoil   *coil[weichenZahl];    
    Fstatic *stat[weichenZahl];   
    Fsignal *sig[weichenZahl];
}Fptr;    

Fservo *AdjServo = NULL ;//Pointer to servo to be adjusted
//Type identifier for connected addresses (addresses with subsequent entries for servos or light signals
//This identifier is only ever entered at the basic address
enum combine_t:byte { NOCOM,//no following address available, default value
                      SERVO4POS,//Servo with 4 positions
                      SERVO_DOUBLE,//2 connected servos
                      SIGNAL2ADR,//Light signal with 4 signal images
                      SIGNAL3ADR };//Light signal with 6 signal images
combine_t adressTyp[weichenZahl];
;
byte progMode;//Flag whether decoder is in programming mode
//--------Encoder evaluation -----Adjustment of the servo end position -----------------------------
# ifdef ENCODER_AKTIV
//The last received switch position can be adjusted using an encoder.
//The values ​​are saved as soon as another switch position is received.
byte adjWix;//Switch index, which is currently is influenced by the encoder.
byte adjPos;//Position of the servo influenced by the encoder
byte adjPulse;//Servo position currently set via encoder
#define NO_ADJ 255//Value of adjPulse as long as no change has occurred
#endif
bool localCV;//local change to a CV (callback NotifyCV is then not executed)
//----Library objects ----
MoToTimer AckImpuls;
MoToTimer ledTimer;//to flash the programming LED
#ifdef LOCONET
//With the Loconet interface, a reset is carried out 2 seconds after the 'Pom' address is changed, whereby
//the Pom address is adopted as the Loconet ID. The time delay is required for low and high
//Bytes can be written before the reset is carried out. The time starts when one of the two
//byte is written.
bool chgLoconetId = false;
MoToTimer idLoconet;
#endif

//^^^^^^^^^^^^^^^^^^^^^^^^ End of definitions ^^^^^^^^^^^^^^^^^^^^^^^ ^^^^^
//################################################## #########################

void setup() {
    boolean iniFlg = false;
#ifdef __STM32F1__
   disableDebugPorts();//Enable JTAG and SW ports
#endif
    #ifdef FIXMODE
    progMode = FIXMODE;
    int temp = -1;
    #else
//Read operating mode
    int temp = analogRead( betrModeP );
    if ( temp > ISNORMAL ) {
//Normal operation
        progMode = NORMALMODE;
    } else if ( temp > ISPOM ) {
//PoM always active
        progMode = POMMODE;
    } else if ( temp > ISOPEN ) {
//IniMode -Basic CV's are always initiated
        progMode = INIMODE;
    } else {
//Programming mode, automatic address recognition
        progMode = ADDRMODE;
    }
    #endif
    
    _pinMode( modePin, OUTPUT );
    #if (defined DEBUG) || (defined IFC_SERIAL)
    #ifndef SERIAL_BAUD
    #define SERIAL_BAUD 115200
    #endif
    Serial.begin(SERIAL_BAUD);//Debugging and/or serial command interface
        #if defined(__STM32F1__) || defined(__AVR_ATmega32U4__) 
//on STM32/ATmega32u4: wait until USB is active (maximum 6sec)
        {  unsigned long wait=millis()+6000;
           while ( !Serial && (millis()<wait) );
        }
        #endif
    #endif
    #if (defined DEBUG)
    #warning "Debugging ist aktiv"
        #ifdef LOCONET
           DB_PRINT(  ">>>>>>>>>> Neustart: (SV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
        #else
           DB_PRINT(  ">>>>>>>>>> Neustart: (CV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
        #endif    

     Serial.print( "Betr:" ); Serial.print(temp);Serial.print(" -> Mode=" );
      switch ( progMode ) {
        case NORMALMODE:
          Serial.println( "Std" );
          break;
        case ADDRMODE:
          Serial.println( "Addr" );
          break;
        case POMMODE:
          Serial.println( "Pom" );
          break;
        case INIMODE:
          Serial.println( "Ini" );
          break;
        default:
          Serial.println( "??" );
          
      }
    #endif//End #ifdef DEBUG

//--------------------------------------
//Initiate CV's
//Check whether the function bytes starting from CV_INITYP are correct
    if ( ifc_getCV( CV_ADRZAHL ) != weichenZahl )  {
        iniFlg = true;
    } else {
        for ( byte i=0; i<weichenZahl; i++ ) {
            if ( ifc_getCV( CV_INITYP+i ) != iniTyp[i] ) { 
                iniFlg = true;
            }
        }
    }
//In V7 no ValidFlg is checked, only whether the HW-specific CV's correspond to the config values
//if ( (ifc_getCV( CV_MODEVAL )&0xf0) != VALIDFLG || ifc_getCV(CV_INIVAL) != VALIDFLG || analogRead(resModeP) < 100 || (ifc_getCV(cvVersionId) < 0x70 ) ) {
    if ( iniFlg || analogRead(resModeP) < 100 || (ifc_getCV(cvVersionId) < 0x70 ) ) {
//There is no correct value in modeVal or ManufactId (or resModeP is set to 0),
//initiate everything with the default values
//If a 'factory reset' is received via DCC, modeVal will be reset, which will happen the next time
//Start leads to initiate.
//
        DB_PRINT( "Init-ALL!!");
        iniCv( INIALL );
    } else if ( progMode == INIMODE ) {
        iniCv( INIMODE );
    }
//---------------------------------
//Initiate interface
    ifc_init( DCC_DECODER_VERSION_ID, progMode, CV_POMLOW );
    
//Read operating mode
    opMode = ifc_getCV( CV_MODEVAL) &0x0f;
    rocoOffs = ( opMode & ROCOADDR ) ? 4 : 0;
    DB_PRINT( "opMode=%d , rocoOffs=%d", opMode, rocoOffs );

//Encoder init
    IniEncoder();

//---End of basic initiation ---------------------------------
    
    setWeichenAddr();//1. Calculate switch address
        
    #ifdef DEBUG
    byte *heap_start = new( byte );
    #endif
//---Save the defined Io's in a linear array. The corresponding array section
//is passed to the function objects as a pointer, which creates an array with 'their' pin numbers
//receive
    for ( byte wIx = 0; wIx<weichenZahl; wIx++ ){
        ioPins[wIx*PPWA] = out1Pins[ wIx ];
        ioPins[wIx*PPWA+1] = out2Pins[ wIx ];
        ioPins[wIx*PPWA+2] = out3Pins[ wIx ];
    }
//---Instantiate function objects according to the configuration ------------------
    for ( byte wIx=0; wIx<weichenZahl; wIx++ ) {
//Instantiate and initiate function objects
        byte vsIx = 0;//Preset the distant signal index on the mast to 0 (no distant signal).
        adressTyp[wIx] = NOCOM;//Default is no follow-on address
        switch (iniTyp[wIx] )  {
          case FSERVO:
//Check whether servo combination (following type = FSERVO0)
            if ( wIx+1<weichenZahl && iniTyp[wIx+1] == FSERVO0 ){
//check whether FSERVO0 has the same connection pin (=servo with several positions)
                if ( out1Pins[wIx] != out1Pins[wIx+1] ) {
//no, 2 servos with combinatorial control
//the 2nd servo accesses the mod byte of the 1st servo, as the mod byte
//of the 2nd servo contains the position combinatorics
                    Fptr.servo[wIx] = new Fservo( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], 2 );
                    Fptr.servo[wIx+1] = new Fservo( cvParAdr(wIx+1,0) , &ioPins[(wIx+1)*PPWA], 2, -CV_BLKLEN );
                    adressTyp[wIx] = SERVO_DOUBLE;
                } else {
//Servo with 4 positions
                    adressTyp[wIx] = SERVO4POS;
                    Fptr.servo[wIx] = new Fservo( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], 4 );
                }
            } else {
//standard servo
                Fptr.servo[wIx] = new Fservo( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], 2 );
            }
            break;
          case F2SERVO://2 servos on one address
            Fptr.twoServo[wIx] = new F2Servo_t;//Set up structure for the 2 servos
//Write pin numbers into the struct.
            Fptr.twoServo[wIx]->pins1[0] = out1Pins[wIx];//Pin basic servo
            Fptr.twoServo[wIx]->pins1[1] = out2Pins[wIx];//Pin center relay for basic servo
            Fptr.twoServo[wIx]->pins1[2] = NC;//no 2 relays possible
            Fptr.twoServo[wIx]->pins2[0] = out3Pins[wIx];//Pin 2. Servo
            Fptr.twoServo[wIx]->pins2[1] = NC;//no relays at
            Fptr.twoServo[wIx]->pins2[2] = NC;//2nd servo possible
//Create servo objects
            Fptr.twoServo[wIx]->servo1 = new Fservo( cvParAdr(wIx,0), Fptr.twoServo[wIx]->pins1, 2 );
            Fptr.twoServo[wIx]->servo2 = new Fservo( cvParAdr(wIx,0), Fptr.twoServo[wIx]->pins2, 2, F2OFFSET );//with offset for parameters
            break;
          case FCOIL:
            Fptr.coil[wIx] = new Fcoil( cvParAdr(wIx,0) , &ioPins[wIx*PPWA] );
            break;
          case FSTATIC:
          case FSTATIC3:
            Fptr.stat[wIx] = new Fstatic( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], iniTyp[wIx]==FSTATIC3 );
            break;
          case FSIGNAL2:
//Check whether a distant signal on the mast needs to be darkened
            vsIx = getCvPar(wIx,PAR3);
            if( !(vsIx >= 1 && vsIx <= weichenZahl) ) {
//no valid distant signal index found
                vsIx = 0;
            }
            [[fallthrough]];
//the rest of the processing is the same for signals and distant signals
          case FVORSIG:
            {//Determine the number of output pins (PPWA*following addresses).
                byte pinZahl = PPWA; 
                if ( wIx+1<weichenZahl && iniTyp[wIx+1] == FSIGNAL0 ){
                    pinZahl+=PPWA;
                    if ( wIx+2<weichenZahl && iniTyp[wIx+2] == FSIGNAL0 ) {
                        pinZahl+=PPWA;
                        adressTyp[wIx] = SIGNAL3ADR;//Light signal with 3 addresses
                    } else {
                        adressTyp[wIx] = SIGNAL2ADR;//Light signal with 2 addresses
                    }
                }
                DBSG_PRINT("Signal %d, PinMax=%d, Vsindex: %d",wIx+1, pinZahl, vsIx );
                if ( vsIx == 0 ) {
                    Fptr.sig[wIx] = new Fsignal( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], pinZahl, NULL );
                } else {
                    Fptr.sig[wIx] = new Fsignal( cvParAdr(wIx,0) , &ioPins[wIx*PPWA], pinZahl, &Fptr.sig[vsIx-1] );
                }
            }
            break;
          default://also FSIGNAL0, FSERVO0
//Servo and signal sequence types are skipped here if necessary
            ;
        }//End switch function types
    }//End loop over all functions

    DBprintCV();//output all CV values ​​in debug mode
#ifdef DEBUG
    byte *heap_end = new( byte );
#ifdef __AVR_MEGA__
//Output memory occupancy for test (in the case of the heap, the addresses are related to the RAM
//the offset of 256 bytes of IO addresses in the ATMega 328 is deducted
    DB_PRINT(">> Setup-Ende >> Heap: Start=0x%x (%d), End=0x%x (%d)", (int)&__heap_start-256, (int)__malloc_heap_start-256, (int)__brkval-256,(int)__brkval-256);
#endif
    DB_PRINT(">>HEAP>> Start=0x%x, End=0x%x, Size=%d", heap_start, heap_end, heap_end-heap_start );

#endif
}



////////////////////////////////////////////////////////////////

void loop() {
    #ifdef DEBUG
/*static unsigned long startMicros = micros();;
static int loopCnt = 0;
loopCnt++;
if ( micros() -startMicros > 1000000L ) {
//output every second of loop duration
Serial.print( "Loop duration(µs):"); Serial.println( 1000000L/(long)loopCnt );
startMicros = micros();
loopCnt = 0;
}*/
    #endif
    
    #ifdef IFC_SERIAL
    dccSim();//Simulation of DCC telegrams
    #endif
    
    getEncoder();//Evaluate the rotary encoder and adjust the servo position if necessary
    ifc_process();//Here the received telegrams are analyzed and the setpoint is set
    #ifdef DEBUG
//Reset flag CV for CV output (flag CV is 1st CV behind the CV block for the output configuration)
    if( ifc_getCV( cvParAdr(weichenZahl,MODE)) != 0xff ) ifc_setCV( cvParAdr(weichenZahl,MODE) , 0xff );
    #endif
    
//Control outputs
    for ( byte i=0; i<weichenZahl; i++ ) {
        switch ( iniTyp[i]  ) {
          case FSERVO://Control servo outputs ------------------------------------------------
            Fptr.servo[i]->process();
            if ( adressTyp[i] == SERVO_DOUBLE ) {
//Control follower servo
                Fptr.servo[i+1]->process();
            }
            break;
          case F2SERVO://2 servos on one address
            Fptr.twoServo[i]->servo1->process();
            Fptr.twoServo[i]->servo2->process();
            break;
          case FCOIL://Double coil drives ------------------------------------------------------
//if ( dccSoll[i] != SOLL_INVALID ) DBCL_PRINT( "SollCoil=%d", dccSoll[i] );
           Fptr.coil[i]->process();
            break;
          case FSTATIC://Switch the output on/off statically -------------------------------------
          case FSTATIC3:
            Fptr.stat[i]->process();
            break;
          case FVORSIG:
          case FSIGNAL2:
            Fptr.sig[i]->process();
            break;
          case FSIGNAL0:
          case FSERVO0: 
//Skip signal sequence types
            ;
        }//-End Switch Function Types-------------------------------------------
    }//End of loop via the functions (switches)---------------


    #ifndef LOCONET
        #ifndef NOACK
//only DCC: switch off ack pulse--------------
        if ( !AckImpuls.running() ) _digitalWrite( ackPin, LOW );
        #endif
    #else
//only with Loconet: check whether Loconet ID should be changed
    if ( chgLoconetId && !idLoconet.running() ) {
        chgLoconetId = false;
        ifc_init( CV_POMLOW );    }
    #endif
    
//Programming LED flashes in programming mode until an address is received
    if ( ! ledTimer.running() && progMode == ADDRMODE) {
        ledTimer.setTime( 500 );
        if ( digitalRead( modePin ) ) 
            CLR_PROGLED;
        else
            SET_PROGLED;
    }
}//end loop
//--------------Setting the functional setpoints -----------------------
//This subprogram is called when a function object is switched
//shall be. e.g. signal image on the light signal, or switch points.
void setPosition( byte wIx, byte sollWert, byte state = 0 ) {
//for most functions only 0/1 makes sense for the setpoint. At light signals
//with several signal images, higher values ​​also make sense (up to 0...5 for 6
//signal images)
//state is only evaluated by FCOIL
    DB_PRINT("Set wIx=%d, soll=%d, state=%d", wIx, sollWert, state);
    switch ( iniTyp[wIx] ) {
        case FSERVO:
//check whether there are two servos to be controlled in combination
          if ( adressTyp[wIx] == SERVO_DOUBLE ) {
//yes, determine positions from the modbyte of the 2nd servo.
            byte pos = ifc_getCV( CV_FUNCTION + CV_BLKLEN*(wIx+1) ) >> ( sollWert*2 );
            DBSV_PRINT("Stellbyte=%02X",pos);
            Fptr.servo[wIx]->set( pos&1 );
            pos >>= 1;
            Fptr.servo[wIx+1]->set( pos&1 );
          } else {
//Standard servo or multi-position servo
            Fptr.servo[wIx]->set( sollWert );
          }
          break;
        case F2SERVO://2 servos on one address
          Fptr.twoServo[wIx]->servo1->set( sollWert );
          Fptr.twoServo[wIx]->servo2->set( sollWert );
          break;
        case FSTATIC:
        case FSTATIC3:
          Fptr.stat[wIx]->set( sollWert );
          break;
        case FCOIL:
          Fptr.coil[wIx]->set( sollWert, state );
          break;
        case FSIGNAL2:
        case FVORSIG:
          Fptr.sig[wIx]->set( sollWert );
          break;
    }
}
//////////////////////////////////////////////////////////////
//Subprograms called by the DCC or LocoNet Library:
//------------------------------------------------
//The following function is called by ifc_process() when a switch telegram has been received
void ifc_notifyDccAccState( uint16_t Addr, uint8_t OutputAddr, uint8_t State ){
//Calculate switch address
    byte i,dccSoll,dccState;
    uint16_t wAddr = Addr+rocoOffs;//Roco counts from 0, all others leave the first 4 switch addresses blank
//In programming mode, the first telegram received determines the first switch address
    if ( progMode == ADDRMODE ) {
//Calculate and save address
//Decoder is always in output address mode
        weichenAddr = wAddr;
        ifc_setCV( cvAccDecAddressLow, wAddr%256 );
        ifc_setCV( cvAccDecAddressHigh, wAddr/256 );
        progMode = PROGMODE;
        SET_PROGLED;
       DB_PRINT( "New: 1.Turnout addr: %d ", weichenAddr );
    }
//Test whether your own switch address
    DB_PRINT( "Switch address: %d , Exit: %d, State: %d", wAddr, OutputAddr, State );
//Check whether address is in the decoder area
    if ( wAddr >= weichenAddr && wAddr < (weichenAddr + weichenZahl) ) {
//is own address, set target value
        byte Ix = wAddr-weichenAddr;
        dccSoll =  OutputAddr & 0x1;
        dccState = State;
        
       
//DB_PRINT( "Switch %d, Index %d, Target %d, Actual %d", wAddr, Ix, dccSoll[Ix], fktStatus[Ix] );
//For servo and signal addresses, the target state may need to be adjusted: For the first
//Subsequent address from 0/1 to 2/3, with the 2nd subsequent address to 4/5 (signals can have up to 6
//have target states). In addition, this changed target state must be sent to the basic address
//be handed over.
        if ( iniTyp[Ix] == FSIGNAL0 || iniTyp[Ix] == FSERVO0 ) {
//it is a subsequent address
            dccSoll += 2;
            Ix--;//Set index to base address
            if ( iniTyp[Ix] == FSIGNAL0 ) {
//is 2nd subsequent address
                 dccSoll += 2;
                 Ix--;
            }
        }
//Check whether a servo is currently being adjusted and, if necessary, stop the adjustment
        ChkAdjEncode( Ix, dccSoll );

//set the addressed address
        setPosition( Ix, dccSoll, dccState );
    }
//Check whether distant signal must be switched via main signal address
    for ( i = 0; i < weichenZahl; i++ ) {
        uint16_t vsAdr;
        if ( iniTyp[i] == FVORSIG  ) {
//Determine the address of the associated main signal
            vsAdr = getCvPar(i, PAR3) + 256 * getCvPar(i, PAR4);
            if ( vsAdr == wAddr ) {
                DBSG_PRINT( "Vorsig0 %d, Index %d, Soll %d", wAddr, i, OutputAddr & 0x1 );
                setPosition( i, OutputAddr & 0x1 );
                break;//Cancel loop run, it can only be one signal address
            } else {
//Check subsequent addresses (for multi-term distant signals).
                if ( i+1 < weichenZahl && iniTyp[i+1] == FSIGNAL0 ) {
//1. Compare subsequent address
                    if ( vsAdr+1 == wAddr ) { 
//Match found, set new signal image
                        DBSG_PRINT( "Vorsig1 %d, Index %d, Soll %d", wAddr, i, (OutputAddr & 0x1)+2  );
                        setPosition( i, (OutputAddr & 0x1)+2 );
                    } else {
                        if ( i+2 < weichenZahl && iniTyp[i+2] == FSIGNAL0 ) {
//2. Compare subsequent address
                            if ( vsAdr+2 == wAddr ) { 
//Match found, set new signal image
                                DBSG_PRINT( "Vorsig2 %d, Index %d, Soll %d", wAddr, i, (OutputAddr & 0x1)+4  );
                                setPosition( i, (OutputAddr & 0x1)+4 );
                            }
                        }
                    }
                }
            }
        }
    }
}
//---------------------------------------------------
#ifndef LOCONET
//is called when the control center reads a CV. A 60mA current pulse is generated
void ifc_notifyCVAck ( void ) {
    #ifndef NOACK
//Start Ack pulse
//DB_PRINT( "Ack Pulse" );
    AckImpuls.setTime( 6 );
    _digitalWrite( ackPin, HIGH );
    #endif
}
#endif
//-----------------------------------------------------
//Subfunction for CVChange: Check changes to servo CVs
void chkServoCv( Fservo *servoP, uint8_t Value, int8_t parNr, int8_t sollOffs ) {
//parNr= 0: Pos 0, 1:Pos1, 2:Speed
//which parameter of the servo was changed?
    if ( parNr >= 0 && parNr < 3 ) {
//it is an end position or speed value
        if ( parNr == 2  ) {
//the speed of the servo was changed
            if ( sollOffs == 0 ) {
//with 4-position servo at the 2nd address (shouldOffs == 2) no speed value.
                DBSV_PRINT( "Servo %4x, Par/Ofs.%d/%d , Speed. %d neu einstellen", (unsigned int)servoP, parNr, sollOffs, Value );
                servoP->adjust( ADJSPEED, Value );
            }
        }  else if (  parNr+sollOffs == servoP->getPos() ){
//This is the current position of the servo,
//Reposition servo
           DBSV_PRINT( "Servo %4x, Par/Ofs.%d/%d , akt Pos. %d justieren", (unsigned int)servoP, parNr, sollOffs, Value );
           servoP->adjust( ADJPOS, Value );
        } else  {
//is not the current position of the servo, change the servo
           DBSV_PRINT( "Servo %4x, Par/Ofs.%d/%d , auf Pos. %d umstellen", (unsigned int)servoP, parNr, sollOffs, Value );
           servoP->set( parNr+sollOffs );
        }
    }
  
}
//Called after a CV value has been changed
void ifc_notifyCVChange( uint16_t CvAddr, uint8_t Value ) {
    if ( !localCV ) {
//CV was changed via nmraDCC. If this is an active servo position, then the servo position
//adjust accordingly, also if the speed has been changed.
//first calculate which index the CV address belongs to.
        localCV = true;//no re-call if CV's are changed within this function
        int8_t wIx = (CvAddr - CV_FUNCTION) / CV_BLKLEN ;
        int8_t parIx = (CvAddr - CV_FUNCTION) % CV_BLKLEN ;
        DB_PRINT( "neu: CV%d=%d ( Index = %d, Parameter = %d )", CvAddr, Value, wIx, parIx  );
        if ( wIx >= 0 && wIx < weichenZahl ) {
//it is a parameter CV
//check whether the output controls a servo:
            switch ( iniTyp[wIx] ) {
              case F2SERVO:
//connected servos -check which servo the CV belongs to
                if (  parIx >= 5 ) {
//2nd servo
                    chkServoCv( Fptr.twoServo[wIx]->servo2, Value, parIx-PAR1-5, 0 );
                  } else {
//1.Servo
                    chkServoCv( Fptr.twoServo[wIx]->servo1, Value, parIx-PAR1, 0 );
                  }
                break;;
              case FSERVO:
                chkServoCv( Fptr.servo[wIx],Value, parIx-PAR1, 0 );
                break;
              case FSERVO0:
//it is a CV of the servo sequence address
                if ( Fptr.servo[wIx] == NULL ) {
//The address does not have its own servo assigned -> is the servo of the previous index with 4 positions
                  chkServoCv( Fptr.servo[wIx-1], Value, parIx-PAR1, 2 );
                } else {
//is the 2nd servo of connected servos (with their own CV's for position and speed)
                  chkServoCv( Fptr.servo[wIx], Value, parIx-PAR1, 0 );
                }
                break;
            }
        }
        #ifdef DEBUG
//check whether the CV address BEHIND the switch addresses has been changed. If yes,
//output all CV values ​​and set the value back to 0xff
            if ( CvAddr ==  cvParAdr(weichenZahl,MODE) && Value !=0xff ) {
                DBprintCV();
            }
        #endif

//Check whether read-only CV has been changed. If so, reset
        if ( CvAddr == cv29Config ) {
//CV29 must not be changed -> set to default
          ifc_setCV( cv29Config, config29Value );//== Accessory decoder with output addressing
        }
        if ( CvAddr == CV_ADRZAHL ) {
            ifc_setCV( CV_ADRZAHL, weichenZahl );
        }
        for( byte i=0; i< weichenZahl; i++ ) {
            if ( CvAddr == (uint16_t)(CV_INITYP+i) ) {
                ifc_setCV( (CV_INITYP+i), iniTyp[i] );
            }
        }
//check whether the switch address has been changed.
//This can be done by changing the decoder address.
//If the decoder address is changed, the MSB (CV9) must be changed first. With the
//Changing the LSB (CV1) will then recalculate the switch address
//if (CvAddr == cvAccDecAddressLow || CvAddr == cvAccDecAddressHigh) setWeichenAddr();
        if (CvAddr ==  cvAccDecAddressLow ) setWeichenAddr();

        #ifdef LOCONET
//Check whether pom address has been changed. If yes, start reset timer
        if ( CvAddr == CV_POMLOW || CvAddr == CV_POMHIGH ) {
            chgLoconetId = true;
            idLoconet.setTime( 2000 );
        }
        #endif
        localCV=false;
    }
}    
//-----------------------------------------------------
void ifc_notifyCVResetFactoryDefault(void) {
//Reset to default values ​​and restart
    localCV = true;//switches off write protection for CV_ADRZAHL
    ifc_setCV( CV_ADRZAHL, 255 );//This completely initiates after reset.
    delay( 20 );
    DB_PRINT( "Reset: AdrZahl=0x%2x", ifc_getCV( CV_ADRZAHL ) );
    delay(500);
    softReset();
}
//------------------------------------------------------
void ifc_notifyDccReset( uint8_t hardReset ) {
#ifdef DEBUG
//if ( hardReset > 0 )//DB_PRINT("Reset received, Value: %d", hardReset);
//is sent when CV is read
#endif
}
//--------------------------------------------------------
/////////////////////////////////////////////////////////////////////////
//Initiate the CV values
void iniCv( byte mode ) {
    localCV= true;//no evaluation in ifc_notifyCVchange
//Standard CV's
    DB_PRINT("iniCV: %d", mode );
    for ( byte i=0; i<(sizeof(FactoryDefaultCVs) / sizeof(CVPair)); i++ ) {
            ifc_setCV( FactoryDefaultCVs[i].CV, FactoryDefaultCVs[i].Value);
    }
//Decoder-specific CV's
    
//general CV's
    ifc_setCV( (int) CV_POMLOW, PomAddr%256 );
    ifc_setCV( (int) CV_POMHIGH, PomAddr/256 );
    ifc_setCV( (int) CV_INIVAL, VALIDFLG );
    ifc_setCV( (int) CV_MODEVAL, VALIDFLG | (iniMode&0xf) );
    ifc_setCV( (int) CV_ADRZAHL, weichenZahl );
//Function-specific CV's
    for ( byte i = 0; i<weichenZahl; i++ ) {
        DB_PRINT("fktSpezCv: %d,Typ=%d", i, iniTyp[i] );
//Save function types
        ifc_setCV( (int) (CV_INITYP+i), iniTyp[i] );
//Save parameters
        #ifdef EXTENDED_CV//initial CV values ​​as 2-dim. array
//native structure from version 7.0
            for ( byte pIx = 0; pIx < CV_BLKLEN; pIx++ ) {
//Do not initiate status value
                if ( pIx != STATE ) ifc_setCV( cvParAdr( i, pIx ), iniCVx[pIx][i] );     
            }
            if ( mode == INIALL ) {
//With INIALL also initiate all status values
                ifc_setCV( cvParAdr(i,STATE ), iniCVx[CV_BLKLEN-1][i] );
            } 
        #else//Config file in V6 format
            ifc_setCV( cvParAdr(i,MODE), iniFmode[i] );
            ifc_setCV( cvParAdr(i,PAR1), iniPar1[i] );
            ifc_setCV( cvParAdr(i,PAR2), iniPar2[i] );
            ifc_setCV( cvParAdr(i,PAR3), iniPar3[i] );
            ifc_setCV( cvParAdr(i,PAR4), iniPar4[i] );//in V6: light signal parameters or status
//delete remaining CSv (5...8).
            for ( byte pIx = 5; pIx < (CV_BLKLEN-1); pIx++ ) {
                ifc_setCV( cvParAdr( i, pIx ), 0 );
            }
            
            if ( mode == INIALL ) {
//With INIALL also initiate all status values
                ifc_setCV( cvParAdr(i,STATE ), iniPar4[i] );
            } 
        #endif
    }
    localCV=false;
}
//-------------------------------------------------------
//Subprograms for servo adjustment with rotary encoder
void IniEncoder( void ) {
    #ifdef ENCODER_AKTIV
//Initiate encoder
    _pinMode( encode1P, INPUT_PULLUP );
    _pinMode( encode2P, INPUT_PULLUP );
    AdjServo = NULL;
    adjPulse = NO_ADJ;
    #endif
}
   
void getEncoder(  ) {
    #ifdef ENCODER_AKTIV
    
    static bool dirState, taktState;
    static int lastCnt,jogCount = 0;
    static enum {IDLE0, TAKTHIGH, TAKTLOW } encoderState;
    static boolean resModeState = HIGH;//Edge detection center button
//Encoder state machine
    switch ( encoderState ) {
      case IDLE0://Initial position, wait for status change at encode1P
        if ( digitalRead( encode1P )!= dirState ) {
            bool temp = digitalRead( encode2P );
            if (  temp != taktState ) {
//There was an additional clock edge, reversal of direction!
//Serial.println( "++++double clock" );
                jogCount = lastCnt;
            }
//Clock input active again
            encoderState = digitalRead( encode2P )? TAKTHIGH : TAKTLOW;
        }
        break;
      case TAKTHIGH:
//wait for a negative edge at the clock input
        if ( ! digitalRead( encode2P )  ) {
//For switches with double detents, generate a pulse here too
            taktState = LOW;
            dirState = digitalRead( encode1P );
            lastCnt = jogCount;
            # ifdef ENCODER_DOUBLE
            if ( dirState )
                jogCount++;
            else jogCount--;
            #endif
            encoderState = IDLE0;
        } 
        break;
      case TAKTLOW:
//wait for pos. Edge at the clock input
        if ( digitalRead( encode2P )  ) {
            taktState = HIGH;
//Determine direction
            dirState = digitalRead( encode1P );
            lastCnt = jogCount;
            if ( dirState )
                jogCount--;
            else jogCount++;
            encoderState = IDLE0;
        } 
    }
    #ifdef DEBUG
//Output encoder counter
    if ( jogCount != 0 ) {
        DBSV_PRINT( "Encoder: %d", jogCount );        
    }
    #endif

    if ( AdjServo != NULL && !AdjServo->isMoving() ) {
//There is a servo currently being adjusted that is not
//just moved
        if ( jogCount != 0 ) {
//Rotate encoder was moved
            if ( adjPulse == NO_ADJ ) {
//is the first adjustment pulse, set the servo to the current adjustment position
//and read current position from CV
                AdjServo->set( adjPos );
                adjPulse = AdjServo->getCvPos();
            }
            if ( (jogCount>0 && adjPulse<180) || (jogCount<0 && adjPulse>0) )
                adjPulse += jogCount;//adjPulse only in the range 0...180
            AdjServo->adjust( ADJPOS, adjPulse );
        } else if ( analogRead( resModeP ) < 200 && resModeState == HIGH ) {
//Center position button pressed
            resModeState = LOW;
            if ( iniTyp[adjWix] == F2SERVO ) {
//with a double servo, the servo to be adjusted is switched here
                DBSV_PRINT("Servo wechseln");
                if ( adjPulse != NO_ADJ ) {
//It was adjusted, save the current value
                    localCV = true;//no processing in callback routine NotifyCvChange
                    AdjServo->adjust(ADJPOSEND,adjPulse);
                    adjPulse = NO_ADJ;
                    localCV = false;
                }
//switch to another servo
                if ( AdjServo == Fptr.twoServo[adjWix]->servo1 ) {
                    AdjServo = Fptr.twoServo[adjWix]->servo2;
                } else {
                    AdjServo = Fptr.twoServo[adjWix]->servo1;
                }
            } else {
//otherwise the servo is moved to the middle position
                DBSV_PRINT("Servo center");
                AdjServo->center(ABSOLUT);
            }
        }
        if ( analogRead( resModeP ) > 700 ) resModeState = HIGH;
    }
    jogCount = 0;
    #endif
}

void ChkAdjEncode( byte WIndex, byte dccSoll ){
    #ifdef ENCODER_AKTIV
//Check whether the following address of two connected servos has been addressed
    if ( adressTyp[WIndex] == SERVO_DOUBLE && dccSoll > 1 ) {
//Subsequent address was addressed, set index to this servo
        WIndex++;
        dccSoll -= 2;
    }
//after receiving a switch address, it is checked whether a previous adjustment has been saved
//must. The received switch address is saved as a new adjustment address if it is a
//Servo drive acts.
    if ( adjPulse != NO_ADJ ) {
//It was adjusted, test whether it needs to be saved (servo change)
        if ( Fptr.servo[WIndex] != AdjServo || adjPos != dccSoll ) {
//Switch was switched or another switch was activated -> save adjustment
            localCV = true;//no processing in callback routine NotifyCvChange
            AdjServo->adjust(ADJPOSEND,adjPulse);
            adjPulse = NO_ADJ;
            adjWix = NO_ADJ;
            AdjServo = NULL;
            localCV = false;
        }
    }
    if ( iniTyp[ WIndex ] == FSERVO || iniTyp[ WIndex] == FSERVO0 ) {
//the new address refers to a servo
        AdjServo = Fptr.servo[WIndex];
        adjPos = dccSoll;
    } else if ( iniTyp[ WIndex ] == F2SERVO ) {
//new address is a double servo, start with servo 1
        AdjServo = Fptr.twoServo[WIndex]->servo1;
        adjPos = dccSoll;
        adjWix = WIndex;
    } else {
        AdjServo = NULL;
    }
    #endif
    #ifdef __AVR_MEGA__
    DB_PRINT("chkAdj-Freemem %d", freeMemory() );
    #endif
}




//////////////////////////////////////////////////////////////////////////
//General subprograms
void setWeichenAddr(void) {
//Decoder is always in output address mode
        weichenAddr = ifc_getAddr( );
/*else
softAddr = (ifc_getAddr( )-1)*4 +1 + rocoOffs ;*/
    DB_PRINT("setWadr: getAdr=%d, wAdr=%d", ifc_getAddr(), weichenAddr );
}
//--------------------------------------------------------
void softReset(void){
    DB_PRINT( "RESET" );
    delay(1000);
    #ifdef __AVR_MEGA__
//asm volatile ("jmp 0");
    cli();//irq's off
    wdt_enable(WDTO_15MS);//wd on,15ms
    while(1);//loop break;
    #endif
    #ifdef __STM32F1__
    nvic_sys_reset();//System reset of the STM32 CPU
    #endif
}
//-----------------------------------------------------------
#ifdef IFC_SERIAL
/*Simulation of DCC commands via serial interface

"ac 14 1 0" > Accessory address 14, setpoint 1 stat 0
"cr 50" > Read cv address 50
"cw 50 3" > Write Cv address 50 with value 3

The same callbacks are called as from the nmraDCC Lib
*/
void dccSim ( void ) {
    static char rcvBuf[40];//also as a send buffer for results
    static byte rcvIx=0;//Index in the receive buffer
    #define IFC_PRINTF( x, ... ) { sprintf_P( rcvBuf, (const char*) F( x ), ##__VA_ARGS__ ) ; IFC_SERIAL.println( rcvBuf ); }
//If data is available, read it into the receive buffer. End character is LF or CR
    char *token;
    int adr =0;//received address
    byte soll,state;
    byte dataAnz = IFC_SERIAL.available();
    if ( dataAnz > 0 ) {
        IFC_SERIAL.readBytes( &rcvBuf[rcvIx], dataAnz );
        rcvIx += dataAnz;
        if ( rcvBuf[rcvIx-1] == 10 || rcvBuf[rcvIx-1] == 13 ) {
            rcvBuf[rcvIx-1] = ' ';
            rcvBuf[rcvIx] = 0;
//receive complete line -> evaluate
            token = strtok( rcvBuf, " ,");
            if ( strcmp( token, "ac" ) == 0 ) {
//accessory command
                adr = atoi( strtok( NULL, " ," ) );
                soll = atoi( strtok( NULL, " ," ) );
                state = atoi( strtok(NULL, " ,") );
                DB_PRINT("Sim: AC,%d,%d,%d",adr,soll,state);
                ifc_notifyDccAccState( adr, soll, state );
            }
            if ( strcmp( token, "cr" ) == 0 ) {
//Read CV
                adr = atoi( strtok( NULL, " ," ) );
                soll = ifc_getCV( adr );
                IFC_PRINTF("CV%d = %d, 0x%02x",adr,soll,soll);
            }
            if ( strcmp( token, "cw" ) == 0 ) {
//Write CV
                adr = atoi( strtok( NULL, " ," ) );
                soll = strtol( strtok( NULL, " ," ),NULL,0 );
                if ( adr == 8 ) {
//Writing to CV8 triggers a reset according to the DCC standard
//Factory settings off (= values ​​from config file)
                    ifc_notifyCVResetFactoryDefault();
                } else {
                    ifc_setCV( adr, soll );
                    IFC_PRINTF("CV%d = %d, 0x%02x",adr, ifc_getCV(adr), ifc_getCV(adr) );
                }
            }
            if ( strcmp( token, "ca" ) == 0 ) {
//Output all essential CVs (only if debugging is active)
                DBprintCV();
            }
//Reset receive buffer
        rcvIx = 0;
        }
    }
}
#endif
#ifdef DEBUG
#ifdef __AVR_MEGA__
int freeMemory() {
//the RAM addresses start at 256 (0x100). They lie underneath
//IO addresses.
    int free_memory;
//DB_PRINT( "&fremem=%d, &heapstrt=%d, heapStrt=%d, brkVal=%d",&free_memory, &__heap_start, __heap_start, __brkval);
    if ((int)__brkval == 0) {
        free_memory = ((int)&free_memory) - ((int)&__heap_start);
    } else {
        free_memory = ((int)&free_memory) - ((int)__brkval);
    }
    return free_memory;
}
#else
int freeMemory() {
    return 0;
}
#endif

void DBprintCV(void) {
//output the entire CV memory used for debug purposes
//Standard addresses
   DB_PRINT( "--------- Debug-Ausgabe CV-Werte ---------" );
   DB_PRINT( "Version: %x, ManufactId: %d", ifc_getCV( cvVersionId ), ifc_getCV( cvManufactId ) );
    
//Decoder configuration global
   #ifdef LOCONET
   DB_PRINT(  "Initval (SV45/47) : 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
    DB_PRINT( "Konfig   (SV29)   : 0x%X", ifc_getCV( cv29Config ) );
    DB_PRINT( "Adresse:(SV17/18) : %d", ifc_getCV( cvAccDecAddressLow )+ifc_getCV( cvAccDecAddressHigh )*256);
    DB_PRINT( "LoconetId(SV48/49): %d"   , ifc_getCV( CV_POMLOW) + 256* ifc_getCV( CV_POMHIGH ) );
   #else
   DB_PRINT(  "Initwert (CV45/47): 0x%x 0x%x ", ifc_getCV( CV_INIVAL ), ifc_getCV( CV_MODEVAL ) );
    DB_PRINT( "Konfig   (CV29)   : 0x%X", ifc_getCV( cv29Config ) );
    DB_PRINT( "Adresse:(CV1/9)   : %d", ifc_getCV( cvAccDecAddressLow )+ifc_getCV( cvAccDecAddressHigh )*256);
    DB_PRINT( "PoM-Adr.(CV48/49) : %d"   , ifc_getCV( CV_POMLOW) + 256* ifc_getCV( CV_POMHIGH ) );
   #endif    
//Output configuration
   DB_PRINT_( "Wadr | Typ | CV's  | Mode| Par1| Par2| Par3| Par4|" );
   DB_PRINT ( " Par5| Par6| Par7| Par8| Status |" );
    for( byte i=0; i<weichenZahl; i++ ) {
       DB_PRINT_( "%4d |%2d/%1d |%3d-%3d| %3d | %3d | %3d | %3d | %3d " , 
                weichenAddr+i, iniTyp[i],adressTyp[i],
                cvParAdr(i,MODE),  cvParAdr(i,STATE),
                getCvPar(i,MODE),
                getCvPar(i,PAR1),
                getCvPar(i,PAR2),
                getCvPar(i,PAR3),
                getCvPar(i,PAR4) );
       DB_PRINT( "| %3d | %3d | %3d | %3d | %3d " , 
                getCvPar(i,PAR5),
                getCvPar(i,PAR6),
                getCvPar(i,PAR7),
                getCvPar(i,PAR8),
                getCvPar(i,STATE) );
    }
    
}
#else
void DBprintCV(void) {
    
}
#endif
