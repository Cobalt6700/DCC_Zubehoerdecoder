/* DIY Zubeh�rdecoder
 *
 * Klassen f�r die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * F�r jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 */
#include "Globals.h"
// Offset der CV-Adresse bei den Funktionsspezifschen CV-Werten
const byte MODE=0, PAR1=1, PAR2=2, PAR3=3, STATE=4 ;
const byte MODE2=5, PAR21=6, PAR22=7, PAR23=8, PAR24=9 ;  // 2. CV-Satz bei Lichtsignalen (Folgeeintrag)

//======================  allgemeine Hilfsfunktionen ==================================
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode );
#else
void _pinMode( byte port, byte mode );
#endif

void _digitalWrite( byte port, byte state ) ;

//========================= Funktionsklassen  =========================================
// Flags f�r CV 'MODE'
#define SAUTOOFF 0x01   // FSERVO: Impulse werden nach erreichen der Endlage abgeschaltet
#define SDIRECT  0x02   // FSERVO: Der Servo reagiert auch w�hrend der Bewegung auf einen Umschaltbefehl
#define NOPOSCHK 0x08   // FSERV: Die Ausg�nge reagieren auch auf einen Befehl, wenn die aktuelle
                        // Postion nicht ver�ndert wird.

class Fservo {
     public:
    Fservo( int cvAdr, uint8_t pins[] );
    void chkState( uint8_t *sollWert );
	bool isMoving ();					// Abfrage ob Servo in Bewegung
	uint8_t getPos();					// aktuellen Status der Weiche ermitteln (GERADE/ABZW)
	void adjust( uint8_t mode, uint8_t value );	// Servoparamter �ndern
		#define ADJPOS		0			// f�r Endagenjustierung: value = neue Position
		#define ADJPOSEND	1			// neue Endlage in CV �bernehmen
		#define ADJSPEED	2			// Servogeschwindigkeit �ndern
    void center( uint8_t mode );
		#define ABSOLUT	 0 		// Servo auf 90� stellen
		#define RELATIVE 1		// Mittelstellung zwischen den programmierten Endpunkten
        
    private:
    Servo8  _weicheS;
    uint16_t _cvAdr = 0;            // Adresse des CV-Blocks mit den Funktionsparametern
    bool _relOut;                   // Ausgangszustand der Relais
    uint8_t *_outP;           		// Array mit Pins der Ausg�nge
		#define SERVOP	0
		#define REL1P	1
		#define REL2P	2
    uint8_t _fktStatus = 0;         // interner Status: rechts/links
		#define GERADE  0x0         // Bit 0
		#define ABZW    0x1         // Bit 0
        #define MOVING  0x8         // 'Moving'-Bit

 
 };
//---------------------- FCOIL -------------------------------------------
// Flags f�r CV 'MODE'
#define CAUTOOFF 0x01   // Die Impulsdauer wird intern begrenzt
#define NOPOSCHK 0x08   // Die Ausg�nge reagieren auch auf einen Befehl, wenn die aktuelle
                        // Postion nicht ver�ndert wird.
 
 class Fcoil {
     public:
    Fcoil( int cvAdr, uint8_t out1P[] );
    void chkState( uint8_t *sollWert, uint8_t outState );
    
    private:
    EggTimer _pulseT;
    uint8_t _pulseON;            // Flag ob Pausentimer l�uft
    
    uint16_t _cvAdr = 0;            // Adresse des CV-Blocks mit den Funktionsparametern
    uint8_t *_outP;           // Pins der Ausgange
    uint8_t _fktStatus = 0;         // interner Status: rechts/links
    #define GERADE  0x0             // Bit 0
    #define ABZW    0x1             // Bit 0

 };

//------------------------FSTATIC -------------------------------------------- 
// Flags f�r CV 'MODE':
#define BLKMODE 0x01    // Ausg�nge blinken
#define BLKSTRT 0x02    // Starten mit beide Ausg�ngen EIN
#define BLKSOFT 0x04    // Ausg�nge als Softleds


 class Fstatic {
    // statisches oder blinkendes Ansteuern von Led's
    public:
    Fstatic( int cvAdr, uint8_t ledP[]  );
    void chkState( uint8_t *sollWert );
    
    private:
    void _setLedPin( uint8_t ledI, uint8_t sollWert );
    EggTimer pulseT;
    
    uint16_t _cvAdr;            // Adresse des CV-Blocks mit den Funktionsparametern
    SoftLed *_ledS[2] = { NULL, NULL };      // Softled-Objekte
    uint8_t *_ledP;           // Pins der Leds
    uint8_t _fktStatus;         // interner Status: 0= aus, 1= aktiv
    const byte BLKON = 0x4 ;    // Bit 2 nur f�r Static, blinkende Led ist EIN
    
 };

//------------------------ FSIGNAL -------------------------------------------- 
//Konstante f�r Lichtsignalfunktion
#define SIG_DARK_TIME   300     // Zeit zwischen Dunkelschalten und Aufblenden des neuen Signalbilds
#define SIG_RISETIME    500     // Auf/Abblendezeit
// Flags f�r CV MODE:
#define LEDINVERT 0x80  // FSIGNAL: SoftledAusg�nge invertieren (Bit 7 des Modebyte von FSIGNAL2/3)
 
 class Fsignal {
    public:
    Fsignal( int cvAdr, uint8_t pins[], uint8_t adrAnz, Fsignal** vorSig );
		// adrAnz ist die Zahl der Adressen, die das Signal belegt. Daraus ergibt sich
		// auch die Zahl der CV-Parameter und die Zahl der Pins
    void chkState( uint8_t *sollWert ); // muss im loop() regelm��ig aufgerufen werden
    void newSetPoint( int8_t sollWert );   // neuen Signalbefehl erhalten
    void setDark  ( bool darkFlg );     // 'true' schaltet das Signal aus(dunkel), false 'ein'
    
    private:
    void _clrSignal ();             // SoftLed's ausschalten
    void _setSignal ();             // aktuelles Signalbild einschalten
    uint8_t  _getSigMask( uint8_t ) ;        // Bitmaske der Ausg�nge f�r aktuelles Signalbild bestimmen
    
    Fsignal **_vorSig;               // Pointer auf Vorsignal am gleichen Mast
    EggTimer darkT;                 // Dunkelzeit beim �berblenden zwischen Signalbildern
    uint16_t _cvAdr = 0;            // Adresse des CV-Blocks mit den Funktionsparametern
    uint8_t  _adrAnz;               // Zahl der Adressen, die dieses Signal belegt
    uint8_t *_outP;           		// Array mit Pins der Ausg�nge
    SoftLed **_sigLed;              // Array f�r die Softled Objekte
    struct status_t {
        byte state  :2;             // Status der internen State-Machine
        byte sigBild:3;             // aktuelles Signalbild entsprechend letztem sollwert
        byte dark   :1;             // Signal ist aktuell Dunkelgeschaltet
        byte isVorSig :1;           // das Objekt ist ein Vorsignal
    } _fktStatus;         // interner Status
    
    // aktueller Signalzustand 
    #define SIG_WAIT        0    // Warte auf Signalbefehle
    #define SIG_DARK        1    // aktuelles Signalbild dunkelschalten ( nur 'soft' Ausg�nge )
    #define SIG_NEW         2    // neues Signalbild aufblenden

 
 
 };
