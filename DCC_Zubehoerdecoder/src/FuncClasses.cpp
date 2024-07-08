/*DIY accessory decoder
*
*Classes for the individual functions FSERVO, FCOIL, FSIGNAL, FSTATIC
*A suitable object must be instantiated for each function defined in the config file.
*Instantiation must be done in setup() with 'new'.
*Each class must have the base CV address of the block with the configuration values ​​when instantiated
*are passed, as well as an array with the pin numbers of the output ports.
*For light signals, the number of output pins (max 8) and a pointer to the object pointer are also included
*of the distant signal on the same mast (or ZERO if there is no distant signal on the mast, or it
*is already a distant signal)
*All classes have at least 2 public methods:
*'process' processes internal processes and must be called regularly in the loop
*'set' gives a switching command to the object (e.g. switch left/right)
*other methods are class-specific.
*/

#include "FuncClasses.h"


//#################### general help functions ############################ ##
//Hide unused (NC) ports
#ifdef __STM32F1__
void _pinMode( byte port, WiringPinMode mode ) {
#else
void _pinMode( byte port, byte mode ) {
#endif
    if ( port != NC ) pinMode( port,  mode );
}

void _digitalWrite( byte port, byte state ) {
    if( port != NC ) {
		digitalWrite( port, state );
//DBSV_PRINT("DW: Pin%d=%d", port,state );
	}
}
//######################### Class definitions ######################## #########

//----------------------FCOIL -------------------------------------------
//Control of double coil drives

Fcoil::Fcoil( int cvAdr, uint8_t out1P[] ) {
//Dual coil drive constructor
    _cvAdr = cvAdr;
    _outP = out1P;
    DBCL_PRINT( "Fcoil CV=%d, OutPins %d,%d,%d ", _cvAdr, _outP[0], _outP[1], _outP[2] );
    
    for ( byte i=0; i<3; i++ ) {
        _pinMode( _outP[i], OUTPUT );
        _digitalWrite( _outP[i], LOW );
    }
	if ( getParam( MODE) & CSTATIC ) {	
//set outputs in STATIC mode
		byte lastState = getParam( STATE );
		if ( getParam( MODE) & CINVERT ) {
			_digitalWrite( _outP[0], !(lastState & 1) );
			_digitalWrite( _outP[1], !(lastState & 2) );
		} else {
			_digitalWrite( _outP[0], lastState & 1 );
			_digitalWrite( _outP[1], lastState & 2 );
		}
		_flags.pulseON =0;
		_flags.sollCoil = 0;
		_flags.istCoil = 0;
		_flags.sollOut = 0;
		_flags.sollAct = 0;
	} else {
//set flags for processing in process()
		_flags.pulseON = false;
		_flags.sollCoil = getParam( STATE )&1;
		_flags.istCoil = !_flags.sollCoil;
		_flags.sollOut = 0;
		_flags.sollAct = true;
	}
}
//..............
void Fcoil::set( uint8_t dccSoll, uint8_t dccState ) {
//receive new switching command
    _flags.sollCoil = dccSoll;
    _flags.sollOut = dccState;
    _flags.sollAct = true;
	if ( getParam( MODE) & CSTATIC ) {
//in STATIC mode the outputs are switched directly
		if ( getParam( MODE) & CINVERT ) {
			digitalWrite( _outP[dccSoll], !dccState  );
			setState( !digitalRead(_outP[0]) | ((!digitalRead(_outP[1]))<<1) );
		} else {
			digitalWrite( _outP[dccSoll], dccState  );
			setState( digitalRead(_outP[0]) | (digitalRead(_outP[1])<<1) );
		}
	} else {
//Switching the outputs in process()
		_flags.sollCoil = dccSoll;
		_flags.sollOut = dccState;
		_flags.sollAct = true;
	}
//If the 3rd pin is defined, output the target state (inverted if CINVERT bit is set).
	_digitalWrite( _outP[2], (getParam(MODE) & CINVERT)? !_flags.sollCoil : _flags.sollCoil );
    DBCL_PRINT( "SetFcoil OutPins %d,%d,%d ", digitalRead(_outP[0]), digitalRead(_outP[1]), digitalRead(_outP[2]) );
}

//..............
void Fcoil::process() {
//In STATIC mode no processing via timer, ON/OFF directly in set method
	if ( getParam( MODE) & CSTATIC) return;
	
//Switch on pulse outputs
    if (  !_flags.pulseON && !_pulseT.running() &&  _flags.sollAct ) {
//Actions at the output only if there is no active pulse and the pause timer is not running
        DBCL_PRINT(" Ist=%d, Soll=%d ", _flags.istCoil, _flags.sollCoil );
        if ( ( _flags.istCoil != _flags.sollCoil || (getParam( MODE) & NOPOSCHK) )
                && _flags.sollOut ) {
//Switch should be switched
            DBCL_PRINT(" State=%d",  _flags.sollOut );
            if ( (_flags.sollCoil & 1) == 0 ) {
//Set Out1 active
                _digitalWrite( _outP[0], HIGH );
                if ( _outP[0] != _outP[1] ) _digitalWrite( _outP[1], LOW );
//DBCL_PRINT( "Pin%d HIGH, Pin%d LOW", _outP[0], _outP[1] );
            } else {
//Set Out2 active
                _digitalWrite( _outP[1], HIGH );
                if ( _outP[0] != _outP[1] ) _digitalWrite( _outP[0], LOW );
//DBCL_PRINT( "Pin%d LOW, Pin%d HIGH", _outP[0], _outP[1] );
            }
            _flags.pulseON = true;
            if ( getParam( PAR1) > 0 ) _pulseT.setTime( getParam( PAR1) * 10 );
            _flags.istCoil = _flags.sollCoil;
            setState( _flags.sollCoil );
        }
        _flags.sollAct = false;//Setpoint was edited.
    }
    
//Switch off pulse outputs
    if ( _flags.pulseON ) {
//check when pulse has to be switched off
        if ( !(getParam( MODE) & CAUTOOFF) && _flags.sollAct && ! _flags.sollOut ) {
//a shutdown telegram was received
            _flags.pulseON = false;
            _pulseT.setTime( 0 );//Turn off the timer if it is running
            _flags.sollAct = false;//Received telegram was processed.
        }
//Timer shutdown
        if ( (getParam( PAR1) > 0) && ! _pulseT.running()  ) {
//Timer shutdown is active and timer has expired
//DBCL_PRINT( "Pin%d LOW, Pin%d LOW", _outP[0], _outP[1] );
            _flags.pulseON = false;
        }
        
        if ( _flags.pulseON == false ) {
            _digitalWrite( _outP[0], LOW );
            _digitalWrite( _outP[1], LOW );
//Set timer for pulse pause
            if ( getParam( PAR2) > 0 ) _pulseT.setTime( getParam( PAR2) * 10 );
        }
        
    }

}


//----------------------FSTATIC ----------------------------------------------------------
//Control of static/flashing LEDs

Fstatic::Fstatic( int cvAdr, uint8_t ledP[], bool extended ) {
//Constructor of the class for static glow or flashing
	byte modeOffs = 0;//In normal mode there is only one mode byte
    _cvAdr = cvAdr;
    _ledP = ledP;
	_flags.extended = extended;
	if ( extended ) {
		modeOffs = 3;//In extended mode, each pin has its own mode byte
	}

    DBST_PRINT( "Fstatic,%d CV=%d, LedPins %d,%d, %d ", extended, _cvAdr, _ledP[0], _ledP[1], _ledP[2] );
//Set up output ports
	for ( byte pNr=0; pNr < 3; pNr++ ) {
		byte modeIx = MODE+(pNr*modeOffs);
//The 3rd pin can only be a SoftLed in extended mode
		if ( (getParam( modeIx ) & BLKSOFT) && (extended || pNr != 2 ) ) {
//Set up output ports as softleds
            if ( _ledP[pNr] != NC ) {
                _ledS[pNr] = new SoftLed;
                byte att;
                int rise;
                att=_ledS[pNr]->attach( _ledP[pNr] );
                rise = (getParam(modeIx) >> 4) * 100;
                if ( rise == 0 ) rise = 500;//default value
                _ledS[pNr]->riseTime( rise );
                _ledS[pNr]->write( OFF, LINEAR );
                DBST_PRINT( "Softled, pin %d, Att=%d", _ledP[pNr], att );
            }
		} else {
        _pinMode( _ledP[pNr], OUTPUT );
        DBST_PRINT( "Hardled, pin %d ", _ledP[pNr] );
		}
	}
//Basic position of the output ports
    _flags.isOn = !getParam( STATE );
    _flags.blkOn = false;//Flashing starts with OFF
    set( getParam( STATE ) );
}

//..............
void Fstatic::set( bool sollOn ) {
//Switch function on/off
    if ( sollOn != _flags.isOn ) {
		if ( _flags.extended ) {
//extended mode (FSTATIC3): switch 3 pins off/on. Start timer in flashing mode
			for ( byte pNr=0; pNr<3; pNr++ ) {
				byte modeIx = MODE+(pNr*MODEOFFS);
				_setLedPin(pNr, sollOn );
				DBST_PRINT( "Soll=%d, Mode=%02x", sollOn, getParam( modeIx ) );
				if ( sollOn && (getParam( modeIx ) & BLKMODE) && getParam( modeIx+1) > 0 ) {
//Power on and flashing mode: start timer
					_pulseT[pNr].setTime( getParam( modeIx+1)*10 );
					DBST_PRINT( "BlinkTime %d = %d", pNr, getParam( modeIx+1)*10 );
				}
			}
		} else {
//normal mode (FSTATIC)
			_digitalWrite( _ledP[2], sollOn );//If necessary, switch the 3rd pin statically
			_setLedPin(0, sollOn );
			if ( getParam( MODE) & BLKMODE ) {
				_setLedPin(1, (getParam( MODE ) & BLKSTRT)&& sollOn ); 
			} else {                   
				_setLedPin(1, !sollOn );                    
			}
			DBST_PRINT( "Fkt=%d, Soll=%d, Ist=%d", _cvAdr, sollOn, _flags.isOn );
			if ( sollOn && ( getParam( MODE ) & BLKMODE ) ) {
//Function is switched on and blinking mode is active -> set timer
				_pulseT[0].setTime( getParam( PAR3)*10 );
				DBST_PRINT( "BlkEin %d/%d, Strt=%x", getParam( PAR1) , getParam( PAR2), (getParam( MODE) & BLKSTRT)  );
				_flags.blkOn = true;
			}
		}
		_flags.isOn = sollOn;
		setState( _flags.isOn );
	}
}

//..............
void Fstatic::process( ) {
//Control the flashing of the LEDs
    if ( _flags.isOn ) {
//only flashes in the active status
	    if ( _flags.extended ) {
			for ( byte ledI=0; ledI<3; ledI++ ) {
				byte modeIx = MODE+(ledI*MODEOFFS);
				if ( (getParam( modeIx ) & BLKMODE) && getParam( modeIx+1 ) > 0 && !_pulseT[ledI].running() ) {
//Timer expired, change LED status
					if ( bitRead( _pinStat, ledI ) ) {
//Turn off LED
						_setLedPin(ledI, LOW );
						if ( getParam( modeIx+2 ) > 0 ) _pulseT[ledI].setTime( getParam( modeIx+2 )*10 );
						else _pulseT[ledI].setTime( getParam( modeIx+1 )*10 );
					} else { 
//Turn on LED
						_setLedPin(ledI, HIGH );
						_pulseT[ledI].setTime( getParam( modeIx+1 )*10 );
					}
				}
			}
		} else if ( getParam( MODE ) & BLKMODE ) {
//Query/set the timers when flashing is active
			if ( !_pulseT[0].running() ) {
//Timer expired, change LED status
				if ( _flags.blkOn ) {
//Turn off LED
					_setLedPin(0, LOW );
					_setLedPin(1, HIGH );
					_flags.blkOn = false;
					_pulseT[0].setTime( getParam( PAR2 )*10 );
				} else {
//Turn on LED
					_setLedPin(0, HIGH );
					_setLedPin(1, LOW );
					_flags.blkOn = true;
					_pulseT[0].setTime( getParam( PAR1 )*10 );
				}
			}
		}
    }
}

//..............
void Fstatic::_setLedPin( uint8_t ledI, bool sollWert ) {
//set the LED output. depending on the configuration as soft LED or hard
	bool outState = sollWert;
    if ( ledI < 3 ) {
//In extended mode, invert output value if necessary
		if ( _flags.extended && ( FSTAINV & getParam( MODE+(MODEOFFS*ledI) ) ) ) outState = !outState; 
        if ( _ledS[ledI] != NULL ) _ledS[ledI]->write( outState);
        else _digitalWrite( _ledP[ledI], outState );
		bitWrite( _pinStat, ledI, sollWert );
		DBST_PRINT( "Pin: %d, Val: %d, pStat=%02x", _ledP[ledI], outState, _pinStat );
    }
}

//-----------------------------FSERVO -------------------------------------------
//Control of servo drives
//offset for the CV's of the position values
const uint8_t Fservo::posOffset[6]={PAR1,PAR2,PAR1+CV_BLKLEN,PAR2+CV_BLKLEN,PAR1+2*CV_BLKLEN,PAR2+2*CV_BLKLEN } ;   
const uint8_t _relIx[] = {1,2,4,5};//Index of relay pins in the pin array

Fservo::Fservo( int cvAdr, uint8_t pins[], uint8_t posZahl, int8_t modeOffs ) {
//Constructor of the Servo class
    _outP = pins;
    _posZahl = posZahl;
    _cvAdr = cvAdr;
    _modeOffs=modeOffs;
    _parOffs=0;
    if ( modeOffs > 0 ) {
        _parOffs = modeOffs;//Is 2nd servo on primary address (F2SERVO)
    }
//Set up turnout servo
    if ( _outP[SERVOP] != NC ) {
        _weicheS.attach( _outP[SERVOP], getParam( _modeOffs ) & SAUTOOFF );
        _weicheS.setSpeed( getParam( PAR3+_parOffs ) );
    }
    for ( uint8_t i = 0; i<posZahl; i++ ) {
        _pinMode( _outP[_relIx[i]], OUTPUT );
        _digitalWrite( _outP[_relIx[i]], LOW );//necessary for STM32/PA15 (this port is HIGH according to pinMode)
    }
//Initiate and output servo values ​​and relay output
    if ( getParam(_modeOffs) & SAUTOBACK )  _istPos = 0;
    else                                    _istPos = getParam( STATE );
    _sollPos = _istPos ;
    _flags.relOn = _istPos;
//11.8.23 (V7.1.2) Relays are always switched in .process
/*if ( posNumber > 2 ) {
//4-position servo: Always set relay according to current position
_digitalWrite( _outP[_relIx[_istPos]], ON );
} else {
//Servo with 2 positions (center switching or 2 relays with position switching)
_digitalWrite( _outP[REL1P], _flags.relOn );
_digitalWrite( _outP[REL2P], !_flags.relOn );
}*/
    _flags.sollAct = false;
    _flags.moving = false;
    _weicheS.write( getParam( posOffset[_istPos]+_parOffs ) );
    DBSV_PRINT("ServoObj@%04x, Pins=%d %d %d %d %d %d , cvAdr=%d, modeOffs=%d", (uint32_t)this , _outP[0],_outP[1],_outP[2],_outP[3],_outP[4],_outP[5], _cvAdr, _modeOffs);
    DBSV_PRINT("ModeByte=%02x", getParam( _modeOffs ) ) ;
   
}

//..............
void Fservo::set( uint8_t newPos ) {
//Command 'set servo' received
    if ( newPos >= _posZahl ) newPos = _posZahl-1;//maximum number of position values
    _sollPos = newPos;
    DBSV_PRINT( "Servo.set(%d): new:%d, soll:%d, max:%d", _outP[0], newPos, _sollPos, _posZahl );
    _flags.sollAct = true;
}
//..............
void Fservo::process() {
//Check the changeover process
//This method must be called in every loop() iteration
    if ( _autoTime.expired() && _sollPos ) {
//Servo is in working position and time has expired: move back
        _sollPos = 0;
        _flags.sollAct = true;
        DBSV_PRINT( "(%04lx) ServoTimer abgelaufen, _istlAbz=%d",(uint32_t)this, _istPos  );
     }   
    
    if ( _flags.moving ) {
//Switch is currently not set, monitor switching point relay and end of movement
        if ( _sollPos != _istPos && (getParam( _modeOffs) & SDIRECT) ) {
//The servo position has been switched and the flag is SDIRECT
//set: cancel movement and delete moving bit.
//The next loop run then reacts to the new position
            _weicheS.write( _weicheS.read() );
            _flags.moving = false;; 
        }
        if ( _weicheS.moving() < 50 ) _flags.relOn = _istPos;
        if ( _weicheS.moving() == 0 ) {
//Movement completed, clear 'MOVING' bit and save position to CV
            _flags.moving = false; 
            _flags.sollAct = false;
            if ( getParam( _modeOffs ) & SAUTOBACK ) {
                if ( _istPos ) {
//Start the timer for reversing in the working position
                    if ( getParam(PAR4+_parOffs) <= 1 ) _autoTime.setTime(SAUTOTIME);
                    else                        _autoTime.setTime( getParam(PAR4+_parOffs) * 100 );
                    DBSV_PRINT("(%04lx) Timer gestartet, Zeit=%d",(uint32_t)this, _autoTime.getTime() );
                }
            } else {
//Save current location without highway
                setState( _istPos );
            }
/*if ( getParam( _modeOffs ) & NOPOSCHK ) {
//Should be set to 'invalid' so that new telegrams with the same
//Position can be recognized (unless it has already been changed)
if ( _sollPos == _fktStatus ) _sollPos = SOLL_INVALID;
DBSV_PRINT( "dccSoll=%d", _sollPos );
}*/
        }
    } else if ( _flags.sollAct  && (_sollPos != _istPos || (getParam( _modeOffs) & NOPOSCHK))  ) {
//Switch needs to be changed
        _istPos = _sollPos;//Actual value to setpoint
        _flags.moving = true;//and set MOVING flag.
        DBSV_PRINT("Servo-write=%d", getParam( posOffset[_sollPos]+_parOffs ) );
        _weicheS.write( getParam( posOffset[_sollPos]+_parOffs ) );
    }
//Set relay outputs
    if ( _outP[REL2P] == NC && _posZahl == 2  ) {
//Variant with a relay, is switched in the middle of the movement
        _digitalWrite( _outP[REL1P], _flags.relOn );
    } else {
//Variant with 2 or 4 relays, switch off all relays during movement
        if ( _flags.moving ) {
            for ( byte i=0; i<_posZahl; i++ ) {
                _digitalWrite( _outP[_relIx[i]], OFF );
            }
        } else {
//Switch relay out accordingly when the servo is at a standstill
            _digitalWrite( _outP[_relIx[_sollPos]], ON );
        }
    }
}
//..............
bool Fservo::isMoving () {
//Query whether servo is moving
    return _weicheS.moving() > 0;
}
//..............
uint8_t Fservo::getPos(){
//determine the current position of the servo
    return _istPos;
}
//..............
uint8_t Fservo::getCvPos(){
//Determine the CV value for the current position of the servo
    return getParam( posOffset[_sollPos]+_parOffs );
}
//..............
void Fservo::adjust( uint8_t mode, uint8_t value ) {
//Change servo parameters
    DBSV_PRINT( "adjust: mode=%d, val=%d", mode, value );
    switch ( mode ) {
      case ADJPOSEND:
//Save adjustment value in the CV of the current position
        setParam( posOffset[_istPos]+_parOffs, value );
//no break, because the soft servo is also set to this position.
        [[fallthrough]];
      case ADJPOS:
        _weicheS.write( value );
        break;
      case ADJSPEED:
        setParam( PAR3+_parOffs, value );
        _weicheS.setSpeed( value );
        DBSV_PRINT("AdjSpeed, %04X = %d ( CV %d ) ", (uint16_t)this, value, PAR3+_parOffs ); 
        break;
    }
}
//..............
void Fservo::center( uint8_t mode ){ 
//Bring servo to center position
    if ( mode == ABSOLUT) {
//absolute center position (90°)
        _weicheS.write(90);
    } else if ( mode == RELATIVE ) {
        _weicheS.write( getParam( PAR1)/2 + getParam( PAR2)/2 );
    }
}
    


//----------------------------FSIGNAL -------------------------------------------
//Control of light signals

Fsignal::Fsignal( int cvAdr, uint8_t pins[], uint8_t pinAnz, Fsignal** vorSig ){
//Constructor for the signal decoder with 1 to 3 addresses
//Signals are always initiated with the basic state ( = HP0 or Hp00 )
    
    _cvAdr  = cvAdr;
    _pinAnz = min( 8, pinAnz );
    _vorSig = vorSig;//== ZERO if no distant signal on the mast
//Number of assigned output ports (maximum 8 used)
    _outP = pins;
    _sigLed = new SoftLed*[_pinAnz] ;
    _fktStatus.sigBild=0x7;//invalid signal image
    _fktStatus.state = SIG_WAIT;
    _fktStatus.dark = false;
     
    DBSG_PRINT( "Fsignal CV=%d, Pins %d", _cvAdr, _pinAnz);
//Set modes of the outputs (3 outputs per address) (Soft/Hard)
    for ( byte pIx=0; pIx < _pinAnz ; pIx++ ) {
//Set output modes
        byte sigMode = _getHsMask();//Bit coding hard/soft LED switching
		_sigLed[pIx] = NULL;//Initiate softled pointer with NULL
        if ( _outP[pIx] != NC ) {
//The CV_s manage the pins address-related (1 CV for 3 pins)
//sigMode contains bit-coded information about whether hard/soft switching
//DBSG_PRINT( "SigMode=%02x, Index= %d, pin=%d, ", sigMode, sigO,outPin);
            if ( sigMode & (1<<pIx) ) {
//Bit set -> hard switchover
                _pinMode(_outP[pIx], OUTPUT );
                _sigLed[pIx] = NULL;
            } else {
//Bit = 0 -> Softled
                byte att;//only for testing purposes (DBSG_PRINT)
                _sigLed[pIx] = new SoftLed;
                att=_sigLed[pIx]->attach( _outP[pIx] , getParam( LSMODE ) & LEDINVERT );
                _sigLed[pIx]->riseTime( SIG_RISETIME );
                _sigLed[pIx]->write( OFF, BULB );
                DBSG_PRINT( "Softled, pin %d, Att=%d", _outP[pIx], att );
            }
//DBSG_PRINT( "portTyp[%d][%d] = %d" , sigO&1, wIx+(sigO>>1), portTyp[sigO&1][wIx+(sigO>>1)] );
		}
    }
    _fktStatus.sigBild = 1;
    set( 0 );
    DBSG_PRINT( "Konstruktor %d", _cvAdr );
}


//-----public methods -----------------------------------------------------------
//Switch signal dark (used for distant signals on the mast of a main signal)
void Fsignal::setDark( bool darkFlg ) {
//If the flag is 'true', the signal is switched off
    if ( darkFlg ) {
//Switch signal dark
        DBSG_PRINT("setDark");
        _clrSignal(0);
        _fktStatus.dark = true;
    } else {
//Switch current signal image back on
        _fktStatus.dark = false;
        DBSG_PRINT("clrDark");
        _setSignal();
    }
}

//..............
//Switch signal image
void Fsignal::set( uint8_t sollWert ) {
    DBSG_PRINT( "setSignal, CV%d, Soll=%d, Ist=%d", _cvAdr, sollWert,  _fktStatus.sigBild );
    if (  _fktStatus.sigBild != sollWert ) {
//Target status has changed, check whether the status is permitted
        if (  _getSigMask( sollWert) == 0xff )  {
//Target state has signal mask 0xff -> ignore this state
//Reset target state
            DBSG_PRINT("SigMask(soll) = %02X", _getSigMask( sollWert) );
        } else {
//Valid state, apply, set flag and raise timer
            _fktStatus.sigBild =  sollWert;
            _fktStatus.state   = SIG_NEW;
            _getSigMask( _fktStatus.sigBild ) ;
            DBSG_PRINT( "SigMSk Stat=%02X, Blnk=%02X, Inv=%02X", _sigMask.staticLed, _sigMask.blnkStd, _sigMask.blnkInv );
            darkT.setTime( SIG_DARK_TIME ) ;
//Switch everything dark in the current signal image that is not lit in the new signal image
            _clrSignal(_sigMask.staticLed|_sigMask.blnkStd|_sigMask.blnkInv); 
//clr signal(sig mask.static led);
            DBSG_PRINT("Ende set %d", _cvAdr );
        }
    }
    
}
//..............
//Control switching of the signal image (must be called in loop())
void Fsignal::process() {
 
    switch ( _fktStatus.state ) {
      case SIG_WAIT:  
//wait for the signal to change status
//'set' was called and the signal image needs to be switched, like this
//the state machine is advanced in the 'set' method
        
//switch the flashing LEDs when the signal images are flashing
        if ( _blinkT.expired() ) {
            _fktStatus.blinkTakt = !_fktStatus.blinkTakt;
            _setSignalBlink ( );
        }
        break;
      case SIG_NEW:
//When timer expires, switch on new signal image
        if ( ! darkT.running() ) {
//has expired: new signal image
            _fktStatus.state = SIG_WAIT;//
            _setSignal();//switch on the current signal image
           
//Set dark switch on the distant signal (only for main signals)
            if ( _vorSig != NULL && *_vorSig != NULL  ) {
                byte darkStates = getParam( DARKMASK );
                (*_vorSig)->setDark( darkStates & (1<< _fktStatus.sigBild) );
                DBSG_PRINT( "darkStates=0x%02x, Signalbild=%d" , darkStates, _fktStatus.sigBild );
            }
        }
        break;
        default:
        ;
    }

}

//--------------private methods -----------------------------------------------
//Determine the hard/soft mask for all pins
uint8_t Fsignal::_getHsMask(){
//the bits are in groups of three for the parameters per address
//A maximum of 8 outputs can be managed
    uint8_t hsMask = getParam( LSMODE ) & 0x7;
    if ( _pinAnz > PPWA ) hsMask |= (getParam( SOFTMASK2 ) & 0x7) << 3; 
    if ( _pinAnz > 2*PPWA ) hsMask |= (getParam( SOFTMASK3 ) & 0x3) << 6;     
    return hsMask;
}

//..............
//Determine output masks for the sigState signal image
byte  Fsignal::_getSigMask( uint8_t sigState ) {
//The static mask is returned (for testing on 0xFF to ignore commands)
//sState: signal state
//static int parOffs[] = { 1,2,6,7,11,12,16,17 } ; //max 4 addresses provided
    byte parOffs = ((sigState>>1) * CV_BLKLEN ) + (sigState&1)  +1;
//the offset for the flashing parameters is larger by BLINK1-BILD1
    byte staticMask = getParam( parOffs );
    byte blinkMask = getParam( parOffs+(BLINK1-BILD1) );
    DBSG_PRINT("stMsk=%02X, blMsk=%02X ( Ofs=%d, Ofb=%d )", staticMask, blinkMask, parOffs, parOffs+(BLINK1-BILD1) );
//DBSG_PRINT("Fsignal-Freemem %d", freeMemory() );
    _sigMask.staticLed =  staticMask & ~blinkMask;//only bit set in staticMask
    _sigMask.blnkStd =  blinkMask & ~staticMask;//only bit set in blnkMask
    _sigMask.blnkInv =  staticMask & blinkMask;//Bit set in both masks
    DBSG_PRINT("  ..static=%02X, blk=%02X, inv=%02X", _sigMask.staticLed, _sigMask.blnkStd, _sigMask.blnkInv );
   return staticMask;
}

//..............
//initiate new signal image
void    Fsignal::_setSignal ( ) {
            _fktStatus.blinkTakt = true;
            _setSignalStatic();//switch on current signal image /static LEDs
            _setSignalBlink();//switch on current signal image /flashing LEDs
}

//Set all static signal outputs according to the current signal state
void Fsignal::_setSignalStatic ( ) {
//byte sigState; //current signal state, derived from the switch states
    byte sigOutMsk;//Bit mask of the output states (bit=1: set output, bit=0 reset output
    byte sigStaMsk;//Bit mask of the static LEDs (only these are switched)
    if ( !_fktStatus.dark ) {
//Signal is not switched off, show signal image
        DBSG_PRINT( "Sig %d EIN (%d)", _cvAdr, _fktStatus.sigBild );
//the current signal image is in _fktStatus.sigImage
//Determine output states according to signal state (CV value)
        _getSigMask( _fktStatus.sigBild ) ;
        sigOutMsk = _sigMask.staticLed ;
        sigStaMsk = ~(_sigMask.blnkInv | _sigMask.blnkStd) ;
//Set the outputs according to the current signal image
        for ( byte i=0; i< _pinAnz ; i++ ) {
            if ( sigStaMsk&1 ) {
//It is a statically switched LED
                if ( _sigLed[i] == NULL ) {
//Standard output
                    _digitalWrite( _outP[i], sigOutMsk&1 );
                } else {
//Softled output
                    DBSG_PRINT(" Static-LED%d=%d", i,sigOutMsk&1 );
                    _sigLed[i]->write( sigOutMsk&1, LINEAR );
                }
            }
            sigOutMsk = sigOutMsk >> 1;
            sigStaMsk = sigStaMsk >> 1;
        }
//DBSG_PRINT( " Signal %d, Status=0x%02x, Outputs: 0x%02x ", wIx, sigStatus, Dcc.getCV( CVBaseAdr[sigStatus] + CVoffs) );
    }
}
 
//Set all flashing signal outputs according to the current signal state and clock phase
void Fsignal::_setSignalBlink ( ) {
//byte sigState; //current signal state, derived from the switch states
    byte sigOutMask;//Bitmask of all flashing LEDs
    byte sigOutMskInv;//Bit mask of the output ports (bit=1: output flashes)
    if ( !_fktStatus.dark ) {
//Signal is not switched off, show signal image
//the current signal image is in _fktStatus.sigImage
//Determine output states according to signal state (CV value)
        _getSigMask( _fktStatus.sigBild ) ;
        sigOutMask   = _sigMask.blnkInv | _sigMask.blnkStd ;
        sigOutMskInv = _sigMask.blnkInv ;
        DBSG_PRINT( "Blinktakt MSK=%02X, INV=%02X", sigOutMask, sigOutMskInv );
        if ( sigOutMask ) {
//there are flashing LEDs
//Set the outputs according to the current signal image
            for ( byte i=0; i< _pinAnz ; i++ ) {
                if ( sigOutMask&1 ) {
//it's a blinking LED
                    if ( _sigLed[i] == NULL ) {
//Standard output
                        _digitalWrite( _outP[i], _fktStatus.blinkTakt ^ (sigOutMskInv&1) );
                    } else {
//Softled output
                        DBSG_PRINT(" Blink-LED%d=%d", i,_fktStatus.blinkTakt ^ (sigOutMskInv&1) );
                        _sigLed[i]->write( _fktStatus.blinkTakt ^ (sigOutMskInv&1), LINEAR );
                    }
                }
                sigOutMskInv = sigOutMskInv >> 1;
                sigOutMask = sigOutMask >> 1;
            }
            int taktZeit = getParam( _fktStatus.blinkTakt?BLINKTAKT1:BLINKTAKT2  ) *10;
            if ( taktZeit == 0 ) taktZeit = SIG_RISETIME + SIG_RISETIME/2 ;//If there is no cycle time, the time is slightly longer than the flashing time of the LED
            DBSG_PRINT( "Sig %d Takt= %d, Zeit=%d BT1=%d, BT2=%d", _cvAdr, _fktStatus.blinkTakt?BLINKTAKT1:BLINKTAKT2, taktZeit, getParam(BLINKTAKT1), getParam(BLINKTAKT2) );
            _blinkT.setTime(  taktZeit );
        }
//DBSG_PRINT( " Signal %d, Status=0x%02x, Outputs: 0x%02x ", wIx, sigStatus, Dcc.getCV( CVBaseAdr[sigStatus] + CVoffs) );
    }
}
 
//..............
//Switch off all signal lamps (when fading between signal images)
void Fsignal::_clrSignal ( byte onMsk ) {
//Switch off 'Soft'Leds of the signal corresponding to Msk
//Bits set in onMsk are not turned off
    if (!(getParam( LSMODE ) & NODARKLED) ) onMsk = 0;//Always switch everything dark
    DBSG_PRINT( "Sig %d AUS", _cvAdr);
//only delete 'soft' initial states
    for ( byte pIx=0; pIx< _pinAnz ; pIx++ ) {
        if ( _sigLed[pIx] != NULL && (onMsk&1) == 0) _sigLed[pIx]->write( OFF, BULB ); 
        onMsk = onMsk >> 1;
    }
//If necessary, also switch off the distant signal at the main signal
    if ( _vorSig != NULL && *_vorSig != NULL ) {
        DBSG_PRINT("Vorsig dunkelschalten");
        (*_vorSig)->setDark( true );
    } 
}
