/*Universal DCC accessory decoder
*ATTENTION: The CV addresses have generally changed compared to V6.x!
*3 outputs /accessory address
*Adjustable functionality:
*-Servo with switching relay for turnout polarization
*-Dual coil drives
*-static/flashing outputs
*-Light signals
*-Setting the servo end positions using a rotary encoder.
*The rotary encoder always refers to the last address and position set.
*-The operating modes and the starting behavior are controlled via the analog inputs A4/A5 (betrModeP and
*resModeP, parameterizable). To do this, pullups must be installed there.
*Depending on how far the voltage is pulled down, the modes are set:
*regardingModeP:
*5V (open) normal operating mode, no PoM
*3.3V (voltage divider 1:2) PoM always active, address always off defaults
*1.6V (voltage divider 2:1) IniMode: CV's are always set to init values ​​from the .h file
*0V programming mode /PoM (1st received telegram determines address)
*resModeP:
*If resModeP is pulled to 0, the servo currently influenced by the rotary encoder is transferred to the
*Middle position brought. As soon as the encoder is moved again, the servo moves again
*to the previous position.
*If resModeP is set to 0 when the program starts, all CV's will be reset to the defaults
*
*Characteristics:
*Multiple (consecutive) accessory addresses can be controlled. The possible number of addresses depends
*depends on the available digital outputs.
*A maximum of 16 servos can be controlled
*
*1. Address adjustable via programming
*
*The behavior of the configured functions is determined via CV programming:
*For servo outputs, the end positions and speed
*for double coil drives, the switch-on time of the coils.
*the flashing behavior for flashing outputs (not yet implemented in V3.0)
*
*Division of CVs:
*CV meaning
*45 identifier for initialization (also distinguishing LocoNet /DCC when starting for the first time)
*47 Initial initiation identifier, general options that apply to the entire decoder
*48/49 Pom address
*50 Number of configured addresses
*51-5x functions of the addresses (function codes)
*120-129 Parameters for 1st switch address
*130-139 Parameters for 2nd switch address
*...
*Meaning of the CV's for the various functions (CV numbers for 1st switch address)
*FSERVO servo:
*CV120 Bit0 = 1: (SAUTOOFF) AutoOff of the servo pulses when the servo is at a standstill
*Bit1 = 1: (SDIRECT) 'Direct mode' is set again even during the servo movement
*Actuating command reacts and, if necessary, the direction of rotation is changed immediately
*Bit2 = 1: (SAUTOBACK) Automatic return to the initial position after time in CV124
*Bit3 = 1: (NOPOSCHK) no check for servo position when receiving a DCC command
*With AUTOOFF and the same position, pulses are output again
*CV121 Position of the servo for switch position '0' (in degrees, 0...180)
*CV122 Position of the servo for switch position '1' (in degrees, 0...180)
*CV123 Servo speed
*CV124 If CV120 Bit4= 1: Time in 0.1s. Move units back automatically until.
*In this case, the servo always starts in the basic position when switched on
*
*FSERVO0 connected servos, must be immediately after FSERVO (following address)
*If the pin number is identical to the entry under FSERVO, only 1 servo will be set up,
*that can be placed in 4 positions. The entries of CV120 and CV123/124 are
*then irrelevant in the FSERVO0 entry.
*If FSERVO0 contains a different pin number for the servo, two servos become
*        furnished. The mode byte of FSERVO then applies to both. The location of the servos can
*can be freely assigned to the DCC commands. This allows, for example, 3-term form signals
*be controlled.
*CV120 bit pattern that indicates the position of the servos for the 4 possible commands
*Bit=0 rest position (position CV51), bit=1 working position (position CV52)
*The least significant bit controls the servo FSERVO,
*the higher order bit controls the servo FSERVO0
*Bit76543210
*++--OFF 1st address
*++----ON 1st address
*++-------OFF 2.Address
*++--------ON 2nd address
*CV121 .. 124 as with FSERVO
*
*F2SERVO Servo: (2 servos on one address)
*CV120..CV124 like the first servo (FSERVO)
*CV125..CV129 Parameter set of the 2nd servo (division like the 1st servo)
*!!!If the SAUTOBACK flag is only set for one servo, this must be the 1st servo.
*
*FCOIL double coil drive: (currently only with automatic switch-off)
*CV120 Bit0 = 1: (CAUTOOFF) Switch off coil output automatically only
*= 0: Switch off coil output also via DCC command
*Bit2 = 1: Control coil outputs only via DCC/LocoNet. Bit0/3 have no meaning
*= 0: Normal operation
*Bit3 = 1: (NOPOSCHK) no check for switch position. If necessary, the same will also be done
*A pulse is repeatedly emitted from the connection
*CV121 Coil duty cycle (in 10ms units)
*0= no automatic shutdown, Bit0 in the mode byte must be 0
*CV122 minimum switch-off time of the coil (in 10ms units)
*CV123 -
*CV124 current switch position (do not change manually!)
*
*FSTATIC static/flashing output
*CV120 Bit0 = 1: (BLKMODE) flashing, 0: static
*Bit1 = 1: (BLKSTRT) When flashing, both LEDs start flashing alternately
*Bit2 = 1: (BLKSOFT) with soft fade in/out
*Bit4..7: Risetime (in 50ms units, 0=default of 500)
*CV121 flashing on time (10ms units)
*CV122 flashing off time (10ms units)
*CV123 1. Switch-on time when flashing starts
*CV129 current status (do not change manually!)
*
*FSTATIC3 static/flashing output -extended mode
*CV120..122 values ​​for out1Pins
*CV123..125 values ​​for out2Pins
*CV126..128 values ​​for out3Pins
*CV120 Bit0 = 1: (BLKMODE) flashing, 0: static
*Bit1 = 1: (FSTAINV) Output is controlled inverted
*Bit2 = 1: (BLKSOFT) with soft fade in/out
*Bit4..7: Risetime (in 50ms units, 0=default of 500)
*CV121 Flashing on time (10ms units) if 0 there is no flashing
*CV122 Flashing off time (10ms units) if 0 applies off time = on time
*CV123..125 like 120..122 for out2Pins
*CV126..128 like 120..122 for out3Pins
*CV129 current status (do not change manually!)
*
*FSIGNAL2 light signal function with 1..3 switch addresses
*The type FSIGNAL0 must be entered for the subsequent addresses
*Light signals always start in state 0 when switched on (bit pattern CV51)
*CV120 signal mode:
*Bit7=1 : inverts the Softled outputs (HIGH=OFF) (MobaTools from V0.9)
*Bit6=1: When the signal image changes, LEDs that are active in both images remain
*constantly switched on and are not switched off for a short time
*Bit 2..0: Bit pattern hard/soft indicates which outputs switch 'hard' (Bit=1)
*and which outputs fade smoothly (Bit=0)
*CV121 Bit pattern of the outputs for command 1. Address 0 (red)
*CV122 Bit pattern of the outputs for command 1. Address 1 (green)
*CV123 index of the distant signal on the same mast ( 1 .... )
*CV124 Bit pattern of the states in which the distant signal is dark:
*Bit 0: Command 1.Address 0 (red)
*Bit 1: Command 1.Address 1 (green)
*Bit 2: Command 2.Address 0 (red)
*etc.
*CV125 bit pattern of the flashing LEDs for command 1. Address 0 (red)
*CV126 Bit pattern of the flashing LEDs for command 1. Address 1 (green)
*CV127 Flashing cycle -phase ON (10ms units)
*CV128 Flashing cycle -phase OFF (10ms units)

*FSIGNAL0 1st subsequent address (optional)
*CV130 Bit 2.. 0 Bit pattern hard/soft indicates which outputs switch 'hard' (Bit=1)
*and Which outputs fade smoothly (Bit=0)
*CV131 bit pattern of the outputs for command 2. Address 0 (red) (255: ignore command)
*CV132 bit pattern of the outputs for command 2. Address 1 (green) (255: ignore command)
*CV133 reserved
*CV134 reserved
*CV135 Bit pattern of the flashing LEDs for command 2. Address 0 (red)
*CV136 Bit pattern of the flashing LEDs for command 2. Address 1 (green)
*CV137-139 reserved

*FSIGNAL0 2nd subsequent address (optional)
*CV140 Bit 2.. 0 Bit pattern hard/soft indicates which outputs switch 'hard' (Bit=1)
*and Which outputs fade smoothly (Bit=0)
*CV141 Bit pattern of the outputs for command 3. Address 0 (red)
*CV143 Bit pattern of the outputs for command 3. Address 1 (green)
*CV143 reserved
*CV144 reserved
*CV145 Bit pattern of the flashing LEDs for command 3. Address 0 (red)
*CV146 Bit pattern of the flashing LEDs for command 3. Address 1 (green)
*CV147-149 reserved
*
*FVORSIG distant signal function
*largely like FSIGNAL2 except:
*CV123 low byte of the address of the announced main signal
*CV124 high byte of the address of the announced main signal
*/
