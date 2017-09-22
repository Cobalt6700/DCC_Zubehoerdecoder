#ifndef GLOBAL_H
#define GLOBAL_H

#include <NmraDcc.h>
#include <MobaTools.h>

#define NC 0xff    // nicht verwendeten Funktionsausg�ngen kann der Port NC zugeweisen werden.
// Da die Pr�fung auf ung�ltige Pin-Nummern in den Arduino-internen Implementierungen je nach Prozessor
// unterschiedlich ist, wird im Sketch selbst auf NC gepr�ft, und gegebenenfalls die Arduino Funktion nicht aufgerufen.

#ifdef DEBUG
#define DB_PRINT( x, ... ) { sprintf_P( dbgbuf, (const char*) F( x ), __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
//#define DB_PRINT( x, ... ) { sprintf( dbgbuf,   x , __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
extern char dbgbuf[60];
#else
#define DB_PRINT( x, ... ) ;
#endif

#endif