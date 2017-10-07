/* DIY Zubeh�rdecoder
 *
 * Klassen f�r die einzelnen Funktionen FSERVO, FCOIL, FSIGNAL, FSTATIC
 * F�r jede im Konfig-File definierte Funktion muss ein passendes Objekt instanziiert werden.
 * Die Instanziierung muss im setup() mit 'new' erfolgen.
 * Jeder Klasse muss bei der Instanziierung die Basis CV-Adresse des Blocks mit den Konfigurationswerten
 * �bergeben werden, sowie ein Array mit den Pin-Nummern der Ausgangsports.
 * Bei Lichtsignalen zus�tzlich die Zahl der Ausgangspins (max 8) und ein Pointer auf den Objektpointer 
 * des Vorsignals am gleichen Mast (oder NULL, wenn kein Vorsignal am Mast vorhanden ist, oder es sich 
 * schon um ein Vorsignal handelt )
 * Alle Klassen haben mindestens 2 �ffentliche Methoden:
 * 'process' bearbeitet interne Abl�ufe und muss im loop regelm��ig aufgerufen werden
 * 'set'     gibt einen Schaltbefehl an das Objekt ( z.B. Weiche links/rechts )
 * weitere Methoden sind klassenspezifisch.
 */

 #include "FuncClasses.h"



//####################  allgemeine Hilfsfunktionen ##############################
// Ausblenden der nicht belegten (NC) Ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode ) {
#else
void _pinMode( byte port, byte mode ) {
#endif
    if ( port != NC ) pinMode( port,  mode );
}

void _digitalWrite( byte port, byte state ) {
    if( port != NC ) digitalWrite( port, state );
}
//#########################  Klassendefinitionen #################################

//----------------------- FCOIL ---------------------------------------------
// Ansteuerung von Doppelspulenantrieben

Fcoil::Fcoil( int cvAdr, uint8_t out1P[] ) {
    // Konstruktor f�r Doppelspulenantriebe
    _cvAdr = cvAdr;
    _outP = out1P;
    DB_PRINT( "Fcoil CV=%d, OutPins %d,%d ", _cvAdr, _outP[0], _outP[1] );
    
    for ( byte i=0; i<2; i++ ) {
        pinMode( _outP[i], OUTPUT );
        digitalWrite( _outP[i], LOW );
    }
    _flags.pulseON = false;
    _flags.sollCoil = getParam( STATE )&1;
    _flags.istCoil = !_flags.sollCoil;
    _flags.sollOut = 0;
    _flags.sollAct = true;
}
//..............    
void Fcoil::set( uint8_t dccSoll, uint8_t dccState ) {
    // neuen Schaltbefehl empfangen
    _flags.sollCoil = dccSoll;
    _flags.sollOut = dccState;
    _flags.sollAct = true;
}

//..............    
void Fcoil::process() {
        // Pulseausg�gne einschalten
    if (  !_flags.pulseON && !_pulseT.running() &&  _flags.sollAct ) {
        // Aktionen am Ausgang nur wenn kein aktiver Impuls und der Pausentimer nicht l�uft
        DB_PRINT(" Ist=%d, Soll=%d ", _flags.istCoil, _flags.sollCoil );
        if ( ( _flags.istCoil != _flags.sollCoil || (getParam( MODE) & NOPOSCHK) )
                && _flags.sollOut ) {
            // Weiche soll geschaltet werden
            DB_PRINT(" State=%d",  _flags.sollOut );
            if ( (_flags.sollCoil & 1) == 0 ) {
                // Out1 aktiv setzen
                digitalWrite( _outP[0], HIGH );
                digitalWrite( _outP[1], LOW );
                //DB_PRINT( "Pin%d HIGH, Pin%d LOW", _outP[0], _outP[1] );
            } else {
                // Out2 aktiv setzen
                digitalWrite( _outP[1], HIGH );
                digitalWrite( _outP[0], LOW );
                //DB_PRINT( "Pin%d LOW, Pin%d HIGH", _outP[0], _outP[1] );
            }
            _flags.pulseON = true;
            if ( getParam( PAR1) > 0 ) _pulseT.setTime( getParam( PAR1) * 10 );
            _flags.istCoil = _flags.sollCoil;
            setState( _flags.sollCoil );
        }
        _flags.sollAct = false; // Sollwert wurde bearbeitet.
    }
    
    // Pulsausg�nge ausschalten
    if ( _flags.pulseON ) {
        // pr�fen ab Impuls abgeschaltet werden muss
        if ( !(getParam( MODE) & CAUTOOFF) && _flags.sollAct && ! _flags.sollOut ) {
            // ein Abschalttelegramm wurde empfangen
            _flags.pulseON = false;
            _pulseT.setTime( 0 );     // Timer abschalten, falls er l�uft
            _flags.sollAct = false;  // Empfangenes Telegramm wurde bearbeitet.
        }
        // Timerabschaltung
        if ( (getParam( PAR1) > 0) && ! _pulseT.running()  ) {
            //  Timerabschaltung ist aktiv und Timer ist abgelaufen
            //DB_PRINT( "Pin%d LOW, Pin%d LOW", _outP[0], _outP[1] );
            _flags.pulseON = false;
        }
        
        if ( _flags.pulseON == false ) {
            digitalWrite( _outP[0], LOW );
            digitalWrite( _outP[1], LOW );
            // Timer f�r Pulspause setzen
            if ( getParam( PAR2) > 0 ) _pulseT.setTime( getParam( PAR2) * 10 );
        }
        
    }

}


//------------------------FSTATIC -------------------------------------------- 
// Ansteuerung von statischen/blinkenden Leds

Fstatic::Fstatic( int cvAdr, uint8_t ledP[] ) {
    // Konstruktor der Klasse f�r statisches Leuchten bzw. blinken
    _cvAdr = cvAdr;
    _ledP = ledP;
    DB_PRINT( "Fstatic CV=%d, LedPis %d,%d ", _cvAdr, _ledP[0], _ledP[1] );
    // Modi der Ausgangsports
    if ( getParam( MODE ) & BLKSOFT ) {
        // Ausgangsports als Softleds einrichten
        for ( byte i=0; i<2; i++ ) {
            if ( _ledP[i] != NC ) {
                _ledS[i] = new SoftLed;
                byte att, rise, writ;
                att=_ledS[i]->attach( _ledP[i] );
                _ledS[i]->riseTime( 500 );
                _ledS[i]->write( OFF, LINEAR );
               DB_PRINT( "Softled, pin %d, Att=%d", _ledP[i], att );
            }
        }
    } else {
        if ( _ledP[0] != NC ) pinMode( _ledP[0], OUTPUT );
        if ( _ledP[1] != NC ) pinMode( _ledP[1], OUTPUT );
        DB_PRINT( "Hardled, pins %d,%d ", _ledP[0], _ledP[1] );
    }
    // Grundstellung der Ausgangsports
    _flags.isOn = !getParam( STATE );
    set( getParam( STATE ) );
    /*
    if ( getParam( MODE ) & BLKMODE ) {
        // aktuellen Blinkstatus ber�cksichtigen
        _setLedPin(0, LOW );
        _setLedPin(1, LOW );
    } else {
        // statische Ausgabe
        _setLedPin(0, _flags.isOn );
        _setLedPin(1, !_flags.isOn );
    }*/
}

//..............    
void Fstatic::set( bool sollOn ) {
    // Funktion ein/ausschalten
    if ( sollOn != _flags.isOn ) {
        _setLedPin(0, sollOn );
        if ( getParam( MODE) & BLKMODE ) {
            _setLedPin(1, (getParam( MODE ) & BLKSTRT)&& sollOn ); 
        } else {                   
            _setLedPin(1, !sollOn );                    
        }
        DB_PRINT( "Fkt=%d, Soll=%d, Ist=%d", _cvAdr, sollOn, _flags.isOn );
        _flags.isOn = sollOn;
        setState( _flags.isOn );
        if ( _flags.isOn && ( getParam( MODE ) & BLKMODE ) ) {
            // Funktion wird eingeschaltet und Blinkmode ist aktiv -> Timer setzen
            _pulseT.setTime( getParam( PAR3)*10 );
            DB_PRINT( "BlkEin %d/%d, Strt=%x", getParam( PAR1) , getParam( PAR2), (getParam( MODE) & BLKSTRT)  );
            _flags.blkOn = true;
        }
    }
}

//..............    
void Fstatic::process( ) {
    // Blinken der Leds steuern
    if ( _flags.isOn && ( getParam( MODE ) & BLKMODE ) ) {
        // bei aktivem Blinken die Timer abfragen/setzen
        if ( !_pulseT.running() ) {
            // Timer abgelaufen, Led-Status wechseln
            if ( _flags.blkOn ) {
                // Led ausschalten
                _setLedPin(0, LOW );
                _setLedPin(1, HIGH );
                _flags.blkOn = false;
                _pulseT.setTime( getParam( PAR2 )*10 );
            } else {
                // Led einschalten
                _setLedPin(0, HIGH );
                _setLedPin(1, LOW );
                _flags.blkOn = true;
                _pulseT.setTime( getParam( PAR1 )*10 );
            }
        }
    }
}

//..............    
void Fstatic::_setLedPin( uint8_t ledI, uint8_t sollWert ) {
    // den LED ausgang 1/2 setzen. je nach Konfiguration als Softled oder hart
    if ( ledI <2 ) {
        if ( _ledS[ledI] != NULL ) _ledS[ledI]->write( sollWert);
        else if ( _ledP[ledI] != NC ) digitalWrite( _ledP[ledI], sollWert );
    }
}

//----------------------------- FSERVO --------------------------------------------
// Ansteuerung von Servo-Antrieben

Fservo::Fservo( int cvAdr, uint8_t pins[] ) {
    // Konstruktor der ServoKlasse
    _outP = pins;
    _cvAdr = cvAdr;
    // Weichenservo einrichten
    if ( _outP[SERVOP] != NC ) {
        _weicheS.attach( _outP[SERVOP], getParam( MODE ) & SAUTOOFF );
        _weicheS.setSpeed( getParam( PAR3 ) );
    }
    _pinMode( _outP[REL1P], OUTPUT );
    _pinMode( _outP[REL2P], OUTPUT );
    // Servowerte und Relaisausgang initiieren und ausgeben
    _flags.isAbzw = getParam( STATE );
    _flags.relOn = _flags.isAbzw;
    _digitalWrite( _outP[REL1P], _flags.relOn );
    _digitalWrite( _outP[REL2P], !_flags.relOn );
    
    if ( _flags.isAbzw ) {
        _weicheS.write( getParam( PAR2 ) );
    } else {
        _weicheS.write( getParam( PAR1 ) );
    }
   
}

//..............    
void Fservo::set( bool sollAbzw ) {
    // Befehl 'servo stellen' erhalten
    _flags.sollAbzw = sollAbzw;
    _flags.sollAct = true;
}
//..............    
void Fservo::process() {
    // Umstellvorgang kontrollieren
    // Diese Methode muss in jedem loop() Durchlauf aufgerufen werden

    if ( _flags.moving ) {
        // Weiche wird gerade ungestellt, Schaltpunkt Relais und Bewegungsende �berwachen
        if ( _flags.sollAbzw != _flags.isAbzw && (getParam( MODE) & SDIRECT) ) {
            // Es wurde die Servoposition umgeschalten und das Flag SDIRECT ist
            // gesetzt: Bewegung abbrechen und Moving-Bit l�schen.
            // Im n�chsten loop-Durchlauf wird dann auf die neue Position reagiert
            _weicheS.write( _weicheS.read() );
            _flags.moving = false;; 
        }
        if ( _weicheS.moving() < 50 ) _flags.relOn = _flags.isAbzw;
        if ( _weicheS.moving() == 0 ) {
            // Bewegung abgeschlossen, 'MOVING'-Bit l�schen und Lage in CV speichern
            _flags.moving = false; 
            _flags.sollAct = false;
            setState( _flags.isAbzw );
            /*if ( getParam( MODE ) & NOPOSCHK ) {
                // Soll auf 'ung�ltig' stellen, damit auch neue Telegramme mit gleicher
                // Position erkannt werden (ausser es wurde schon ver�ndert )
                if ( _flags.sollAbzw == _fktStatus ) _flags.sollAbzw = SOLL_INVALID;
                DB_PRINT( "dccSoll=%d", _flags.sollAbzw );
            }*/
        }
    } else if ( _flags.sollAct  && (_flags.sollAbzw != _flags.isAbzw || (getParam( MODE) & NOPOSCHK))  ) {
        // Weiche muss umgestellt werden
        DB_PRINT( "Weiche stellen, Ist=%d,Soll=%d", _flags.isAbzw, _flags.sollAbzw );
        _flags.isAbzw = _flags.sollAbzw;    // Istwert auf Sollwert 
        _flags.moving = true;               // und MOVING-Flagt setzen.
        if ( _flags.sollAbzw  ) {
            _weicheS.write( getParam( PAR2) );
        } else {
            _weicheS.write( getParam( PAR1) );
        }
    }
    // Relaisausg�nge setzen
    if ( _outP[REL2P] == NC ) {
        // Variante mit einem Relais, wird in Bewegungsmitte umgeschaltet
        _digitalWrite( _outP[REL1P], _flags.relOn );
    } else {
        // Variante mit 2 Relais, w�hrend der Bewegung beide Relais abschalten
        if ( _flags.moving ) {
            _digitalWrite( _outP[REL1P], OFF );
            _digitalWrite( _outP[REL2P], OFF );
        } else {
            // im Stillstand des Servos entsprechend relaisout schalten
            _digitalWrite( _outP[REL1P], _flags.relOn );
            _digitalWrite( _outP[REL2P], !_flags.relOn );
        }
    }
    
   
}
//..............    
bool Fservo::isMoving () {
    // Abfrage ob servo in Bewegung
    return _weicheS.moving() > 0;
}
//..............    
uint8_t Fservo::getPos(){
    // aktuelle Position des Servos ermitteln
    return _flags.isAbzw;
}
//..............    
void Fservo::adjust( uint8_t mode, uint8_t value ) {
    // Servoparameter �ndern
    switch ( mode ) {
      case ADJPOSEND:
        // Justierungswert im CV der aktuellen Position speichern
        if ( _flags.isAbzw ) setParam( PAR2, value );
        else setParam( PAR1, value );
        // kein break, da weichenservo auch noch auf diese Position gestellt wird.    
      case ADJPOS:
        _weicheS.write( value );
      case ADJSPEED:
        setParam( PAR3, value );
        _weicheS.setSpeed( value );
      break;
    }
}
//..............    
void Fservo::center( uint8_t mode ){ 
    // Servo auf Mittelstellung bringen
    if ( mode == ABSOLUT) {
        // absolute Mittelstellung (90�)
        _weicheS.write(90);
    } else if ( mode == RELATIVE ) {
        _weicheS.write( getParam( PAR1)/2 + getParam( PAR2)/2 );
    }
}
    


//---------------------------- FSIGNAL --------------------------------------------
// Ansteuerung von Lichtsignalen

Fsignal::Fsignal( int cvAdr, uint8_t pins[], uint8_t pinAnz, Fsignal** vorSig ){
    // Konstruktor f�r den Signaldecoder mit 1 bis 3 Adressen
    // Signale werden immer mit dem Grundzustand initiiert ( = HP0 oder Hp00 )
    
    _cvAdr  = cvAdr;
    _pinAnz = min( 8, pinAnz );
    _vorSig = vorSig;   // == NULL wenn kein Vorsignal am Mast
    // Zahl der zugeordneten Ausgangsports (maximal 8 genutzt)
    _outP = pins;
    _sigLed = new SoftLed*[_pinAnz] ;
    _fktStatus.sigBild=0x7; // ung�ltiges Signalbild
    _fktStatus.state = SIG_WAIT;
    _fktStatus.dark = false;
     
    DB_PRINT( "Fsignal CV=%d, Pins %d", _cvAdr, _pinAnz);
    //Modi der Ausg�nge setzen ( 3 Ausg�nge je Adresse ) (Soft/Hard)
    for ( byte pIx=0; pIx < _pinAnz ; pIx++ ) {
        // Modi der Ausg�nge setzen
        byte sigMode = _getHsMask(); // Bitcodierung harte/weiche Ledumschaltung
		_sigLed[pIx] = NULL;		 // Softled-Pointer mit NULL initiieren
        if ( _outP[pIx] != NC ) {
            // Die CV_s verwalten die pins Adressbezogen ( 1 CV f�r 3 Pins )
            // sigMode enth�lt bitcodiert die Info ob harte/weiche Umschaltung
           //DB_PRINT( "SigMode=%02x, Index= %d, pin=%d, ", sigMode, sigO,outPin);
            if ( sigMode & (1<<pIx) ) {
                // Bit gesetzt -> harte Umschaltung
                _pinMode(_outP[pIx], OUTPUT );
                _sigLed[pIx] = NULL;
            } else {
                // Bit = 0 -> Softled
                byte att, rise, writ; // nur f�r Testzwecke ( DB_PRINT )
                _sigLed[pIx] = new SoftLed;
                att=_sigLed[pIx]->attach( _outP[pIx] , getParam( LSMODE ) & LEDINVERT );
                _sigLed[pIx]->riseTime( SIG_RISETIME );
                _sigLed[pIx]->write( OFF, BULB );
                DB_PRINT( "Softled, pin %d, Att=%d", _outP[pIx], att );
            }
            //DB_PRINT( "portTyp[%d][%d] = %d" , sigO&1, wIx+(sigO>>1), portTyp[sigO&1][wIx+(sigO>>1)] );
		}
    }
    _fktStatus.sigBild = 1;
    set( 0 );
    DB_PRINT( "Konstruktor %d", _cvAdr );
}


//----- �ffentliche Methoden -----------------------------------------------------------
// Signal dunkelschalten ( genutzt f�r Vorsignale am Mast eines Hauptsignals )
void Fsignal::setDark( bool darkFlg ) {
    // Ist das Flag 'true' wird das Signal dunkelgeschaltet
    if ( darkFlg ) {
        // Signal dunkelschalten
        DB_PRINT("setDark",0);
        _clrSignal();
        _fktStatus.dark = true;
    } else {
        // Aktuelles Signalbild wieder einschalten
        _fktStatus.dark = false;
        DB_PRINT("clrDark",0);
        _setSignal();
    }
}

//..............    
// Signalbild umschalten
void Fsignal::set( uint8_t sollWert ) {
    DB_PRINT( "setSignal, CV%d, Soll=%d, Ist=%d", _cvAdr, sollWert,  _fktStatus.sigBild )
    if (  _fktStatus.sigBild != sollWert ) {
        // Sollzustand hat sich ver�ndert, p�fen ob erlaubter Zustand
        if (  _getSigMask( sollWert) == 0xff )  {
            // Sollzustand hat Signalmaske 0xff -> diesen Zustand ignorieren
            // Sollzustand zur�cksetzen
        } else {
            // G�ltiger Zustand, �bernehmen, Flag setzen und Timer aufziehen
            _fktStatus.sigBild =  sollWert;
            _fktStatus.state   = SIG_NEW;
            darkT.setTime( SIG_DARK_TIME ) ;
            _clrSignal(); // aktuelles Signalbild dunkelschalten
            DB_PRINT("Ende set %d", _cvAdr );
        }
    }
    
}
//..............    
// Umschalten des Signalbilds steuern ( muss im loop() aufgerufen werden )
void Fsignal::process() {
 
    switch ( _fktStatus.state ) {
      case SIG_WAIT:  
        // warten auf Zustands�nderung am Signal
        // wurde 'set' aufgerufen, und das Signalbild muss umgeschaltet werden, so 
        // wird die Statemachine in der 'set' Methode weitergeschaltet
        break;
      case SIG_NEW:
        // Wenn Timer abgelaufen, neues Signalbild aufschalten
        if ( ! darkT.running() ) {
            // ist abgelaufen: neues Signalbild
            _fktStatus.state = SIG_WAIT; // 
            _setSignal(); // aktuelles Signalbild einschalten
            // Dunkelschaltung am Vorsignal setzen (nur bei Hauptsignalen)
            if ( _vorSig != NULL && *_vorSig != NULL  ) {
                byte darkStates = getParam( DARKMASK );
                (*_vorSig)->setDark( darkStates & (1<< _fktStatus.sigBild) );
                DB_PRINT( "darkStates=0x%02x, Signalbild=%d" , darkStates, _fktStatus.sigBild );
            }
        }
        break;
        default:
        ;
    }

}

//-------------- private Methoden -----------------------------------------------
// Maske Hard/Soft f�r alle Pins bestimmen 
uint8_t Fsignal::_getHsMask(){
    // die Bits stehen jeweils in 3er Gruppen bei den Parametern je Adresse
    // Maximal k�nnen 8 Ausg�nge verwaltet werden
    uint8_t hsMask = getParam( LSMODE ) & 0x7;
    if ( _pinAnz > PPWA ) hsMask |= (getParam( SOFTMASK2 ) & 0x7) << 3; 
    if ( _pinAnz > 2*PPWA ) hsMask |= (getParam( SOFTMASK3 ) & 0x3) << 6;     
    return hsMask;
}

//..............    
// Ausgangsmaske f�r Signalbild sigState bestimmen
uint8_t  Fsignal::_getSigMask( uint8_t sigState ) {
    // sState: Signalzustand
    static int parOffs[] = { 1,2,6,7,11,12,16,17 } ; // max 4 Adressen vorgesehen
    //DB_PRINT("Fsignal-Freemem %d", freeMemory() );
   return getParam( parOffs[sigState] );
}

//..............    
// alle Signalausg�nge entsprechend dem derzeitigen Signalzustand setzen
void Fsignal::_setSignal ( ) {
    //byte sigZustand; // aktueller Signalzustand, abgeleitet aus den Weichenzust�nden
    byte sigOutMsk;  // Bitmaske der Ausgangsports (Bit=1:Ausgang setzen, Bit=0 Ausgang r�cksetzen
                     // Diese Maske steht f�r jeden Signalzustand in entsprechenden CV-Paramtern:
                     // CV51+offs    Bitmuster der Ausg�nge f�r Befehl 1.Adresse 0 (rot)
                     // CV52+offs    Bitmuster der Ausg�nge f�r Befehl 1.Adresse 1 (gr�n)
                     // CV56+offs    Bitmuster der Ausg�nge f�r Befehl 2.Adresse 0 (rot)
                     // CV57+offs    Bitmuster der Ausg�nge f�r Befehl 2.Adresse 1 (gr�n)
                     // die folgenden CV's sind nur relevant bei FSIGNAL3 (3 Adressen, 8 Zust�nde 6 Ausg�nge)
                     // CV61+offs    Bitmuster der Ausg�nge f�r Befehl 3.Adresse 0 (rot)
                     // CV62+offs    Bitmuster der Ausg�nge f�r Befehl 3.Adresse 1 (gr�n)
                     // offs= wIx*5
                     //
    if ( !_fktStatus.dark ) {
        // Signal ist nicht dunkelgeschaltet, Signalbild aufblenden
        DB_PRINT( "Sig %d EIN (%d)", _cvAdr, _fktStatus.sigBild );
        // das aktuelle Signalbild steht in _fktStatus.sigBild
        // Ausgangszust�nde entsprechend Signalzustand bestimmen (CV-Wert)
        sigOutMsk = _getSigMask( _fktStatus.sigBild ) ;
        // Die Ausg�nge entsprechend dem aktuellen Signalbild setzen
        for ( byte i=0; i< _pinAnz ; i++ ) {
            if ( _sigLed[i] == NULL ) {
                // Standard-Ausgang
                digitalWrite( _outP[i], sigOutMsk&1 );
            } else {
                // Softled-Ausgang
                _sigLed[i]->write( sigOutMsk&1, LINEAR );
            }
            sigOutMsk = sigOutMsk >> 1;
        }
        //DB_PRINT( " Signal %d, Status=0x%02x, Ausg�nge: 0x%02x ", wIx, sigZustand, Dcc.getCV( CVBaseAdr[sigZustand] + CVoffs)  );
    }
}
 
//..............    
// alle Signallampen ausschalten ( beim �berblenden zwischen Signalbildern )
void Fsignal::_clrSignal () {
    // alle 'Soft'Leds des Signals ausschalten
    DB_PRINT( "Sig %d AUS", _cvAdr);
    // nur 'soft' Ausgangszust�nde l�schen
    for ( byte pIx=0; pIx< _pinAnz ; pIx++ ) {
        if ( _sigLed[pIx] != NULL ) _sigLed[pIx]->write( OFF, BULB ); 
    }
    // am Haupsignal gegebenenfalls auch das Vorsignal dunkelschalten
    if ( _vorSig != NULL && *_vorSig != NULL ) {
        DB_PRINT("Vorsig dunkelschalten",0);
        (*_vorSig)->setDark( true );
    } 
}
