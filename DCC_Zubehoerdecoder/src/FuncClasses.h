/* DIY Zubeh�rdecoder
 *
 * Klassen f�r die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * F�r jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 */
#include "Globals.h"

class Fservo {
 
 };
 
 class Fcoil {
 
 };
 
#define BLKMODE 0x01    // FSTATIC: Ausg�nge blinken
#define BLKSTRT 0x02    // FSTATIC: Starten mit beide Ausg�ngen EIN
#define BLKSOFT 0x04    // FSTATIC: Ausg�nge als Softleds


 class Fstatic {
    // statisches oder blinkendes Ansteuern von Led's
    private:
    void _setLedPin( uint8_t ledI, uint8_t sollWert );
    EggTimer pulseT;
    
    const byte BLKON = 0x4 ;            // Bit 2 nur f�r Static, blinkende Led ist EIN
    uint16_t _cvAdr;     // Adresse des CV-Blocks mit den Funktionsparametern
    const byte MODE=0, PAR1=1, PAR2=2, PAR3=3, STATE=4 ;
    SoftLed *_ledS[2] = { NULL, NULL };      // Softled-Objekte
    uint8_t _ledP[2];       // Pins der Leds
    uint8_t _fktStatus;  // interner Status: 0= aus, 1= aktiv
    
    public:
    Fstatic( uint16_t cvAdr, uint8_t led1, uint8_t led2 );
    void chkState( uint8_t sollWert );
    
 };
 
 class FSignal {
 
 };
