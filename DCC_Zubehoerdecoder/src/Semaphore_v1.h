#include <Arduino.h>
//#include <avr/power.h>

#include <NmraDcc.h>
#include <TimerOne.h>
#include <MobaTools.h>


// #include <Wire.h>
// #include <Adafruit_MCP23017.h>
// #include <Adafruit_PWMServoDriver.h>
// #include "Adafruit_WS2801.h"
// #include "SPI.h" 


#include <Math.h>

// Pin definitions, change to your needs
constexpr byte servoPin[] = { 2, 3 };
constexpr byte nbrTurnouts = sizeof(servoPin); // Number of turnouts
constexpr uint16_t servoSpeed = 8000;          // Full speed
MoToServo   SemaServo[nbrTurnouts];            // create servo objects


// #define SERVO 0
// #define RELAY 1

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//#define measureMode

#ifdef measureMode
  int loopCounter = 0;
  uint32_t loopTimer = millis();
#endif

#define THROWN 0
#define CLOSED 1

// #define SERVOMIN 210 // this is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX 850 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN 700 // this is the 'minimum' pulse length - 550 µs for ESP8266/ESP32/RP2040 700µs otherwise (must not be set shorter).
#define SERVOMAX 2300 // this is the 'maximum' pulse length - 2600 µs for ESP8266/ESP32/RP2040 2300µs otherwise (must not be set longer)

#define SERVOACCEL 1500 //increments / s2 used for soft start
#define SERVODECEL 1500 //increments / s2 used for soft stop

const uint16_t refreshInterval = 5000; //microseconds. Might be too low for Nano if 2 servos are processed at the same time

//#define SERVOMIN 50 // this is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX 4000 // this is the 'maximum' pulse length count (out of 4096)

//dimension considerations
// servo speed in incr/s, ranging from -1500 to 1500 (max speed of typical servo about +/- 1200 incr/s
// modeled as 16 bit integer times 20 to make room for fractions,creating a range from -30000 .. 30000
//fractions are important to keep track of actual speed in case of small accel/decel over several steps, not all resulting in a full increment

// position in incr ranging from SERVOMIN to SERVOMAX, typical range 200= 0.5ms) to 850 (= 2.07ms in 100Hz PWM)
// modeled as 16 bit with 6bit left shift and offset 150, allowing positions from 0 (150) (0.36ms) to 1023 (1173) (2.86ms)

// accel/decel in incr/s2, typical range from 1000 - 2500 (?)
// constant defined for sketch, not adjustable per channel

// #define colorDark 0x00000000    //all LED's dark
// #define relayON  0x0000007F     //50% blue to indicate active coil on relay
// #define relayOFF 0x00000000     //same as dark for when relay is off
// #define relayThrown 0x001F0000  //relay is off and on thrown position
// #define relayClosed 0x00001F00  //relay is off in closed position

#define servoMinPos 0x00001F00  //servo is in minimum position
#define servoMaxPos 0x001F0000  //servo is in maximum position
#define servoMove 0x00050500    //servo is currently moving

// #define aspectHalt 0x007F0000   //signal aspect color for halt (red)
// #define aspectSlow 0x005F0F00   //signal aspect color yellow (may also be used for blinking)
// #define aspectClear 0x00007F00  //signal aspect color green
// #define level1Col 0x000F0000    //red used for level crossing blink light

// #define blinkLEDInterval 800    //duration of blink cycle (half of it, it is symmetrical)

// called this way, it uses the default address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Adafruit_MCP23017 mcp;

NmraDcc Dcc;

const int DccIntrpt = 0;
const int DccDataPin = 2;
const int DccAckPin = 3; //not connected
//const int LEDDataPin = 6; //comment out if using SPI
//const int LEDClockPin = 7; //comment out if using SPI

#define numChannels 2
#define numPixels 0

// Adafruit_WS2801 thisStrip = Adafruit_WS2801(numPixels);
// uint32_t pixCopy[numPixels];
// bool ledChg = false;

// uint32_t ledBlinkTimer = millis();
// uint32_t timeElapsed = 0;
// bool blinkFlag = false;
// float faderCtr = 0;

// typedef void (*ledProcess) (void *);



typedef struct
{
  uint16_t dccAddr;
  uint16_t portA; //can be substituted by the position in the array for a SERVO only decoder
  uint16_t minPos; //lower boundary in increments from SERVOMINPOS to SERVOMAXPOS
  uint16_t maxPos; //upper boundary
  uint8_t  moveConfig; //determins acceleration and deceleration of servo movements in both directions. LS nibble when increasing/CL, MS nibble when decreasing/TH
                     //nibble content: bits 0,1 define stop mode 0: hard stop 1: soft stop; 2: overshoot; 3: bounceback
                     // bit 2 defines start mode: 0: immediate start; 1: soft start
                     // bit 3 defines hesitation: 0: no hesitation; 1: hesitation active
  uint8_t  oscLambda; //4 bit value for oscillation damper, 0 - 15 interpreted as 0.5 .. 8.0  LS nibble when increasing/CL, MS nibble when decreasing/TH
  uint8_t  oscFrequency; //4 bit value for oscillation frequency, 0 - 15 interpreted as 0.5 .. 8.0 Hz  LS nibble when increasing/CL, MS nibble when decreasing/TH
  uint16_t hesitatePosition; //increments per second
  uint8_t  hesitateSpeed; //increments per second
  uint16_t targetPos; //runtime data
  uint16_t currPos; //runtime data, increments shifted left by 6, with six bits for fractions
//   uint16_t moveDelayTH; //milliseconds pulse time for relays
  uint16_t moveDelayCL; //servo movement increments per second for servos. Real speed depends onf PWM frequency; CL is up, TH is down the range

  uint32_t nextMove; //runtime data milliseconds when next calculation is triggered
  float    currSpeed; //runtime data, increments per millisecond. Pos val is up, neg val is down
  uint32_t timeNull; //runtime data
  uint8_t  currMoveMode; //runtime data 0: at target; 1: accelerating; 2: linear movememnt; 3: hesitating; 4: stopping
} myServo;


myServo servoArray[numChannels] = {
    {50, 0, SERVOMIN, SERVOMAX-150, 0x00, 0x22, 0x33, 450, 0, SERVOMIN, SERVOMIN + 1, 0, 0,0,0,0},
    {51, 1, SERVOMIN, SERVOMAX-150, 0x00, 0x11, 0x55, 450, 0, SERVOMIN, SERVOMIN + 1, 0, 0,0,0,0},
};

uint16_t getServoPos(myServo * thisServo)
{
  return (thisServo->currPos >> 6) + 150;
}

void setServoPos(myServo * thisServo, uint16_t value) //150 <= value <= 1173
{
  value = min(1173, max(value, 150));
  thisServo->currPos = (value-150)<<6;
}

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
  Serial.print("notifyDccAccTurnoutOutput: ");
  Serial.print(Addr, DEC);
  Serial.print(',');
  Serial.print(Direction, DEC);
  Serial.print(',');
  Serial.println(OutputPower, HEX);

  for (int i = 0; i < numChannels; i++)
  {
    if (Addr == servoArray[i].dccAddr)
    {
      if (Direction == 0)
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].minPos : THROWN;
      else
        servoArray[i].targetPos = (servoArray[i].driveType == SERVO) ? servoArray[i].maxPos : CLOSED;
    //   if (servoArray[i].driveType == RELAY)
    //     setServoPos(&servoArray[i], (servoArray[i].targetPos == CLOSED) ? THROWN : CLOSED);
      servoArray[i].currMoveMode = 0; //new position command, stop any current movement
      servoArray[i].nextMove = micros(); //initialize timer
    }
  }
}

void processServo(myServo *thisServo)
{
//  uint32_t thisTime = micros();
  if (thisServo->currMoveMode == 0) //linear movement mode
  {
    uint16_t currPos = getServoPos(thisServo); //incr
    if (thisServo->targetPos != currPos)
    {
      if ((thisServo->nextMove < micros()))
      {
        //analyze the current status for further decision making
        bool moveUp = thisServo->targetPos > getServoPos(thisServo);
        uint8_t adjMode = moveUp ? (thisServo->moveConfig & 0x0F) : ((thisServo->moveConfig & 0xF0) >> 4);
        float linSpeed = moveUp ? (double) thisServo->moveDelayCL : (double)(-1) * thisServo->moveDelayTH; //set linSpeed to incr/s
        bool correctDir = (sgn(thisServo->currSpeed) == sgn(linSpeed)) || (thisServo->currSpeed == 0);
        bool softStop = ((adjMode & 0x03) == 1); //soft stop
        bool softStart = (adjMode & 04); //overshoot or soft stop
        bool beforeHesitate = adjMode & 0x08 ? (moveUp ? currPos < thisServo->hesitatePosition : currPos > thisServo->hesitatePosition) : false; 
        uint16_t moveEndPoint = beforeHesitate ? thisServo->hesitatePosition : thisServo->targetPos; //end point or hesitate
        float moveEndSpeed = (beforeHesitate ? (sgn(linSpeed) * thisServo->hesitateSpeed) : (adjMode & 0x02 ? (linSpeed) : (adjMode == 0 ? (linSpeed) : (0)))); //speed when arriving at end point, incr/s
        int32_t breakDistance = round(sq(linSpeed - moveEndSpeed) / (2 * (int32_t)SERVODECEL)); //s = v2 /2a
        uint16_t breakPoint = moveEndPoint - (sgn(linSpeed) * breakDistance);
        bool beforeBreakPoint = ((int)(breakPoint - currPos) * (int)sgn(linSpeed)) > 0;

        //calculate dynamic data for step duration and width 
        uint32_t stepDelay = thisServo->currSpeed == 0 ? refreshInterval : round(1000000 / abs(thisServo->currSpeed)); //calculating the duration of 1 step in micros/incr
        float stepFactor = thisServo->currSpeed == 0 ? 1 : 1 + (refreshInterval / stepDelay);  //calculate how many steps to take assuming 5ms cycle time

        if (correctDir)
        {
          if (linSpeed != 0)
          {
            if (beforeBreakPoint)
            {
              if (abs(thisServo->currSpeed) < abs(linSpeed))
              {
                if (softStart)
                //accelerate
                  thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVOACCEL) / 1000000)); //v = v0 + at
                else
                  thisServo->currSpeed = linSpeed;
                if (abs(thisServo->currSpeed) > abs(linSpeed))
                  thisServo->currSpeed = linSpeed;
              }
              //keep moving
              thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
              thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
            }
            else
            {
              if (getServoPos(thisServo) == thisServo->targetPos) //final position
              {
                setServoPos(thisServo, thisServo->targetPos);
                thisServo->nextMove += refreshInterval; //1ms wait                
              }
              else
              {
              //accel-/decel to moveEndSpeed
                bool endAccel = false;
                if (abs(moveEndSpeed) > abs(thisServo->currSpeed))
                //accelerate
                {
                  //add comparison to target speed at position, adjust acceleration if needed
                  thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVOACCEL) / 1000000)); //v = v0 + at
                  if (abs(moveEndSpeed) <= abs(thisServo->currSpeed))
                    thisServo->currSpeed = moveEndSpeed;
                  endAccel = thisServo->currSpeed == moveEndSpeed;
                }
                else
                //decelerate
                {
                  //add comparison to target speed at position, adjust deceleration if needed
                  int sgnSpeed = sgn(thisServo->currSpeed);
                  thisServo->currSpeed -= round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVODECEL) / 1000000)); //v = v0 - at
                  if (sgn(thisServo->currSpeed) != sgnSpeed)
                    thisServo->currSpeed = 0;
                  endAccel = thisServo->currSpeed == 0;
                }
                //advance to moveEndPoint
                if (endAccel)
                {
                  setServoPos(thisServo, moveEndPoint);
                  thisServo->nextMove += refreshInterval; //1ms wait                
                }
                else
                {
                  thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
                  thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
                }
              }
            }
            //when there, execute move end
            bool currMoveUp = thisServo->targetPos > getServoPos(thisServo);
            if ((getServoPos(thisServo) == thisServo->targetPos) || (moveUp != currMoveUp)) //overshooting when speed > 1 incr per cycle
              if ((adjMode & 0x03) > 1) //bounce back or overshooting
              {
                setServoPos(thisServo, thisServo->targetPos);
                thisServo->currMoveMode = 1; //enter oscillation phase
                thisServo->timeNull = micros();
              }
              else
                thisServo->currSpeed = 0;
          }
          else
          {
            setServoPos(thisServo, thisServo->targetPos);
            thisServo->nextMove = micros();
          }
        }
        else
        {
          if (softStop)
          {
            //decelerate and change direction. Speed is incr/s, accel/decel is incr/s2, time intervl is 1ms
            int sgnSpeed = sgn(thisServo->currSpeed);
            thisServo->currSpeed += round((sgn(linSpeed) * stepFactor * (stepDelay * (int32_t)SERVODECEL) / 1000000)); //v = v0 - at
            if (sgn(thisServo->currSpeed) != sgnSpeed)
              thisServo->currSpeed = 0;
          }
          else
            thisServo->currSpeed = 0;
          if (thisServo->currSpeed != 0)
          {
            thisServo->currPos += round(stepFactor * 64 * sgn(linSpeed)); //set the position
            thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
          }
          else
            thisServo->nextMove += refreshInterval; //standard 1ms wait
        }
    //    printPosition(thisServo->currSpeed, getServoPos(thisServo));
        // pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo));

      }
    }
  }
  else //oscillator mode
  {
    bool moveUp = getServoPos(thisServo) == thisServo->maxPos;
    uint8_t adjMode = moveUp ? (thisServo->moveConfig & 0x0F) : ((thisServo->moveConfig & 0xF0) >> 4);
    float thisLambda = (float)((moveUp ? (thisServo->oscLambda & 0x0F) : ((thisServo->oscLambda & 0xF0) >> 4)) + 1) / 2;
    float thisFreq = (float)((moveUp ? (thisServo->oscFrequency & 0x0F) : ((thisServo->oscFrequency & 0xF0) >> 4)) + 1) / 2;
    float timePassed2 = (float)(micros() - thisServo->timeNull) / 1000;
    float timePassed = timePassed2 / 1000;
    //calculate y(t)
    float origAmpl = thisServo->currSpeed / (TWO_PI * thisFreq);
    float currAmpl = origAmpl  * exp(thisLambda * timePassed * -1);
    float currVal = round(currAmpl * sin(TWO_PI * thisFreq * timePassed));
    uint16_t pwmVal = getServoPos(thisServo); //moveUp ? (uint16_t)SERVOMAX : (uint16_t)SERVOMIN;

    if ((currAmpl / origAmpl) > 0.1) //stop oscillator if amplitude < 10% of original value
    {
      //PWM to targetPos
      if ((adjMode & 0x03) == 2) //bounce back
        //PWM to targetPos + newAmpl
        pwmVal = pwmVal + currVal;
      else //3, overshoot
        //PWM to targetPos - abs(newAmpl)
        pwmVal = moveUp ? pwmVal - abs(currVal) : pwmVal + abs(currVal);
//      printPosition(thisServo->currSpeed, pwmVal);
    //   pwm.setPWM(thisServo->portA, 0, pwmVal);
    } 
    else //done, back to linear mode
    {
    //   pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo));

      thisServo->currMoveMode = 0; 
      thisServo->currSpeed = 0; 
    }
    thisServo->nextMove += refreshInterval; //standard 1ms wait
  }
}

void printPosition(float speedVal, uint16_t posVal)
{
  Serial.print(micros());
  Serial.print(", ");
  Serial.print(speedVal);
  Serial.print(", ");
  Serial.println(posVal);
}

void processServoSimple(myServo *thisServo)
{
  if (thisServo->targetPos != getServoPos(thisServo))
  {
    if ((thisServo->nextMove < micros()))
    {
      bool moveUp = thisServo->targetPos > getServoPos(thisServo);
      uint16_t stepSpeed = moveUp ? thisServo->moveDelayCL : thisServo->moveDelayTH; //incr/s 
      if (stepSpeed > 0) //valid speed settings
      {
        uint32_t stepDelay = stepSpeed > 0 ? round(1000000 / stepSpeed) : 0; //calculating the duration of 1 step
        float stepFactor = 1 + (refreshInterval / stepDelay);  //calculate how many steps to take assuming 5ms cycle time
        thisServo->currPos += round(stepFactor * 64 * (moveUp ? 1 : (-1))); //set the position
        thisServo->nextMove += round(stepFactor * stepDelay); //set the delay time
        bool nextDir = thisServo->targetPos > getServoPos(thisServo);
        if (moveUp != nextDir) //reached targetPos, so break the movement
        {
          setServoPos(thisServo, thisServo->targetPos);
          thisServo->nextMove = micros()+ refreshInterval;
        }
      }
      else //no settings, go with maximum speed to the target
      {
        setServoPos(thisServo, thisServo->targetPos);
        thisServo->nextMove = micros()+ refreshInterval;
      }
//      printPosition(thisServo->currSpeed, getServoPos(thisServo));
    //   pwm.setPWM(thisServo->portA, 0, getServoPos(thisServo)); //setting the PWM output
    }
  }
}

void processLocations()
{
  for (int i = 0; i < numChannels; i++)
  {
    if ((servoArray[i].targetPos != getServoPos(&servoArray[i])) || (servoArray[i].currMoveMode != 0))
    {      
        if (servoArray[i].moveConfig > 0){
            processServo(&servoArray[i]);
        }
        else { 
            processServoSimple(&servoArray[i]);
        }    
    }
  }
}

void setup()
{
//  clock_prescale_set(clock_div_1); //make this a 32MHz machine????
  
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DccAckPin, OUTPUT);
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(DccIntrpt, DccDataPin, 1);

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);

  Serial.println("Init Done");

  for (int i = 0; i < numChannels; i++)
  {    
      servoArray[i].currPos = (servoArray[i].currPos - 150)<<6;
  }  

}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  processLocations();
  
#ifdef measureMode
  if (loopTimer < millis())
  {
    Serial.println(loopCounter);
    loopCounter = 0;
    loopTimer += 1000;
  }
  loopCounter++;
#endif  
}
