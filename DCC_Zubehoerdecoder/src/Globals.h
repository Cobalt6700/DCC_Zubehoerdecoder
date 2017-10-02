#ifndef GLOBAL_H
#define GLOBAL_H

#include <NmraDcc.h>
#include <MobaTools.h>

#define NC 0xff    // nicht verwendeten Funktionsausg�ngen kann der Port NC zugeweisen werden.
// Da die Pr�fung auf ung�ltige Pin-Nummern in den Arduino-internen Implementierungen je nach Prozessor
// unterschiedlich ist, wird im Sketch selbst auf NC gepr�ft, und gegebenenfalls die Arduino Funktion nicht aufgerufen.

#define PPWA 3            // Zahl der Pins je Weichenadresse

#define SOLL_INVALID    0xff    // dcc-Sollwert ung�ltig

#define DEBUG
#ifdef DEBUG
#define DB_PRINT( x, ... ) { sprintf_P( dbgbuf, (const char*) F( x ), __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
//#define DB_PRINT( x, ... ) { sprintf( dbgbuf,   x , __VA_ARGS__ ) ; Serial.println( dbgbuf ); }
extern char dbgbuf[60];
extern unsigned int __heap_start;
extern char *__malloc_heap_start;
extern char *__brkval;
int freeMemory();
#else
#define DB_PRINT( x, ... ) ;
#endif

//-------------------------------Definition der CV-Adressen ---------------------------------------
#define CV_MODEVAL    47  // Initiierungs-CV
                          // =0x5? wenn die CV-Werte initiiert sind. Bits 0..3 f�r Betriebsarten
#define CV_POMLOW     48  // Adresse f�r die POM-Programmierung
#define CV_POMHIGH    49
#define CV_FUNCTION   50  // Start der Bl�cke f�r die Funktionskonfiguration
#define CV_BLKLEN      5  // L�nge eines CV-Blocks ( pro Adresse ein Block )
                          // Die Bedeutung ist weitgehend funktionsspezifisch

#define cvAdr(wIx,par)	CV_FUNCTION+CV_BLKLEN*wIx+par
#define getCvPar(wIx,par) Dcc.getCV( cvAdr(wIx,par) )


#endif