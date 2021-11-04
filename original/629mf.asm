; Ver 2.0 alpha modified to be function only decoder GP3/comparator input
; Ver 2.01 beta 1 with further development (see below for enhancement/fixes)
; Ver 2.02 addressing inverted functioning of F0 forwards and changed CV8 to 13
; Ver 2.03 Beta 2 F0 reverse invertion now fixed as well.
; Ver 2.04 F5-F8 and F9-F12 now not transposed.
; Ver 2.05 Added output inversion in CV99.
; Ver 2.06 Added decoder lock.
; Ver 2.07 and 2.08 only apply to 12F683 and 16F684
; Ver 2.09 DC Analogue mode improvements using CV14
; Ver 2.10 Improved DC brake
; Ver 2.11 Implemented CV21 and CV22 for F0-F12 operated by consist address
; Ver 2.12 Added support for LokMaus2 programming
; Ver 2.13 Changed CV29 default to 6 as per NMRA
; Ver 2.14 Fixed ignoring short address 112-127
; Ver 2.15 Tidied up redundant code and registers.
; Ver 2.16 Added flicker for PORTC outputs C,E,F,G and H
; Ver 2.17 Protected unsupported bits in CV29 so they cannot be set
; Ver 2.18 Corrected ACK pulse to 6mS
; Ver 2.19 Corrected error in CV Verify routine.
;

; This is a basic DCC mobile function decoder using the 12F629/675 (3 or 5 function)
; or 16F630/676 (8 function)
; It uses the internal 4MHz osc. Execution cycle is 1us.

; It incorporates ideas from D.Probst DCC decoder project
; and Heiko Schroeter's 12F629 mobile 'Z' gauge decoders
; Parts copyright (C) Dean Probst and (C) Heiko Schroeter but released
; under the GNU General Public License

; This version is by Paul Harman mailto:diydecoder@hotmail.co.uk and is not fully functional.
; It will be superceded by a working version but is sufficiently functional to test the 
; hardware platform.

;    This program is free software; you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation; either version 2 of the License, or
;    (at your option) any later version.

;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.

;    You should have received a copy of the GNU General Public License
;    along with this program; if not, write to the Free Software
;    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.


; BUGFIXES and ENHANCEMENTS
;
; 1. Programming on the programming track (service mode) now fully implemented.
;
; 2. Function 0 is now direction dependent for 14,27,28 and 126 speed steps
;
; 3. Functions 5-12 now implemented.
;
; 3A. Functions 13-28 now implemented
;
; 6. Changed manufacturer ID temporarily to 13 (DIY and development)
;
; B. Brake on DC should work as well as analogue DC.
;
; C. Support now present for a CV (CV13) to hold function definitions for use in DC mode.
;
; F. CV47 moved to CV13.
;
; i. Resolved inverse functioning of F0
;
; 3B. Resolved transposition of function group two blocks F5-F8 and F9-F12.
;
; A1. Added function output inversion.
;
; G. Decoder lock as per TCS and Digitrax (CV15 and CV16) RP
;
; 4. DC Analogue now works and uses CV14 as well as CV13
;
; 4A. Brake on DC should now return to DCC without requiring power off
;
; 7. Implemented CV21 and CV22 for proper functioning in consist
;
; 5. Fixed state machine timing variability by removing redundant code
;
; J. Added LokMaus2 programming support using CV7
;
; H. Added flicker for oil lamp or firebox on port C outputs using CV100.
;
; 11A. Fixed ACK pulse to 6mS
;
; 11B. Corrected CV verify routine.
;
;
; KNOWN BUGS:-
;
; 4. Brake on DC not tested (may actually work!).
;
; 6. Incorrect manufacturer ID (need an allocation of ID and ver No for production)
;
; 8. CV17 write does not wait for CV18 write before committing.
;
; 9. Function status not retained during power outage.
;
; 10. Flicker not very good on Lenz.
;
; PLANNED CHANGES
;
; A. Implementation of programmable special effects such as dim on outputs similar to Lenz Gold.
;
; D. Proper CV definitions for functions 13-28.
;
; E. Possible support for asymetric braking (in 12F675 or 16F676) if anyone can think of an application!
;
; H. Implementation of 'exotic' UK lighting effects such as oil lamp, flashing tail lamp.
;
; I. Perhaps a servo drive option for fitting to live steam/internal combustion locos.
;
;==========================================================================

 #define ManId .13	; This should be the NMRA assigned ID in CV8, but anything other than 8 or 255
 #define VerNo .2	; The version release number to identify the decoder in CV7

	LIST	p=12F629	; target processor, also should work in 12F675, 16F630 and 16F676
	__CONFIG  _BODEN_ON & _CP_OFF & _WDT_OFF & _MCLRE_OFF & _INTRC_OSC_NOCLKOUT & _PWRTE_ON
	__idlocs  0xB219

;==========================================================================
;
;       Register Definitions
;
;==========================================================================

W	EQU	H'0000'
F	EQU	H'0001'

;----- Register Files------------------------------------------------------

INDF	EQU	H'0000'
TMR0	EQU	H'0001'
PCL	EQU	H'0002'
STATUS	EQU	H'0003'
FSR	EQU	H'0004'
GPIO	EQU	H'0005'
PORTC	EQU H'0007'	; For 14 pin device compatibility (16F630/676)

PCLATH	EQU	H'000A'
INTCON	EQU	H'000B'
PIR1	EQU	H'000C'

TMR1L	EQU	H'000E'		
TMR1H	EQU	H'000F'		
T1CON	EQU	H'0010'		
CMCON	EQU	H'0019'		
OPTION_REG	EQU	H'0081'
TRISIO	EQU	H'0085'
TRISC	EQU	H'0087'	; For 14 pin device compatibility (16F630/676)
PIE1	EQU	H'008C'
PCON	EQU	H'008E'
OSCCAL	EQU	H'0090'
WPU	EQU	H'0095'
IOCB	EQU	H'0096'
IOC	EQU	H'0096'

VRCON	EQU	H'0099'
EEDATA	EQU	H'009A'	
EEADR	EQU	H'009B'	
EECON1	EQU	H'009C'
EECON2	EQU	H'009D'

ADCON0	EQU H'001F' ; For analogue device compatibility (12F675/16F676)
ADCON1	EQU H'009F' ; For analogue device compatibility (12F675/16F676)
ANSEL	EQU H'0091' ; For analogue device compatibility (12F675/16F676)

;----- STATUS Bits --------------------------------------------------------

IRP	EQU	H'0007'
RP1	EQU	H'0006'
RP0	EQU	H'0005'
NOT_TO	EQU	H'0004'
NOT_PD	EQU	H'0003'
Z	EQU	H'0002'
DC	EQU	H'0001'
C	EQU	H'0000'

;----- GPIO Bits --------------------------------------------------------

GP5	EQU	H'0005'
GPIO5	EQU	H'0005'
GP4	EQU	H'0004'
GPIO4	EQU	H'0004'
GP3	EQU	H'0003'
GPIO3	EQU	H'0003'
GP2	EQU	H'0002'
GPIO2	EQU	H'0002'
GP1	EQU	H'0001'
GPIO1	EQU	H'0001'
GP0	EQU	H'0000'
GPIO0	EQU	H'0000'

;----- INTCON Bits --------------------------------------------------------

GIE	EQU	H'0007'
PEIE	EQU	H'0006'
T0IE	EQU	H'0005'
INTE	EQU	H'0004'
GPIE	EQU	H'0003'
T0IF	EQU	H'0002'
INTF	EQU	H'0001'
GPIF	EQU	H'0000'

;----- PIR1 Bits ----------------------------------------------------------

EEIF	EQU	H'0007'
ADIF	EQU	H'0006'
CMIF	EQU	H'0003'
T1IF	EQU	H'0000'
TMR1IF	EQU	H'0000'

;----- T1CON Bits ---------------------------------------------------------

TMR1GE	EQU	H'0006'
T1CKPS1	EQU	H'0005'
T1CKPS0	EQU	H'0004'
T1OSCEN	EQU	H'0003'
NOT_T1SYNC	EQU	H'0002'
TMR1CS	EQU	H'0001'
TMR1ON	EQU	H'0000'

;----- COMCON Bits --------------------------------------------------------

COUT	EQU	H'0006'
CINV	EQU	H'0004'
CIS	EQU	H'0003'
CM2	EQU	H'0002'
CM1	EQU	H'0001'
CM0	EQU	H'0000'

;----- OPTION Bits --------------------------------------------------------

NOT_GPPU	EQU	H'0007'
INTEDG	EQU	H'0006'
T0CS	EQU	H'0005'
T0SE	EQU	H'0004'
PSA	EQU	H'0003'
PS2	EQU	H'0002'
PS1	EQU	H'0001'
PS0	EQU	H'0000'

;----- PIE1 Bits ----------------------------------------------------------

EEIE	EQU	H'0007'
ADIE	EQU	H'0006'
CMIE	EQU	H'0003'
T1IE	EQU	H'0000'
TMR1IE	EQU	H'0000'

;----- PCON Bits ----------------------------------------------------------

NOT_POR	EQU	H'0001'
NOT_BOD	EQU	H'0000'

;----- OSCCAL Bits --------------------------------------------------------

CAL5	EQU	H'0007'
CAL4	EQU	H'0006'
CAL3	EQU	H'0005'
CAL2	EQU	H'0004'
CAL1	EQU	H'0003'
CAL0	EQU	H'0002'

;----- IOCB Bits --------------------------------------------------------

IOCB5	EQU	H'0005'
IOCB4	EQU	H'0004'
IOCB3	EQU	H'0003'
IOCB2	EQU	H'0002'
IOCB1	EQU	H'0001'
IOCB0	EQU	H'0000'

;----- IOC Bits --------------------------------------------------------

IOC5	EQU	H'0005'
IOC4	EQU	H'0004'
IOC3	EQU	H'0003'
IOC2	EQU	H'0002'
IOC1	EQU	H'0001'
IOC0	EQU	H'0000'

;----- VRCON Bits ---------------------------------------------------------

VREN	EQU	H'0007'
VRR	EQU	H'0005'
VR3	EQU	H'0003'
VR2	EQU	H'0002'
VR1	EQU	H'0001'
VR0	EQU	H'0000'

;----- EECON1 -------------------------------------------------------------

WRERR	EQU	H'0003'
WREN	EQU	H'0002'
WR	EQU	H'0001'
RD	EQU	H'0000'

;==========================================================================
;
;       RAM Definition
;
;==========================================================================

        __MAXRAM H'FF'
        __BADRAM H'06'-H'09', H'0D', H'11'-H'18', H'1A'-H'1F', H'60'-H'7F'
        __BADRAM H'86'-H'89', H'8D', H'8F', H'91'-H'94', H'97'-H'98', H'9E'-H'9F', H'E0'-H'FF'

;==========================================================================
;
;       Configuration Bits
;
;==========================================================================

_CPD_ON		EQU	H'3EFF'
_CPD_OFF	EQU	H'3FFF'
_CP_ON		EQU	H'3F7F'
_CP_OFF		EQU	H'3FFF'
_BODEN_ON	EQU	H'3FFF'
_BODEN_OFF	EQU	H'3FBF'
_MCLRE_ON	EQU	H'3FFF'
_MCLRE_OFF	EQU	H'3FDF'
_PWRTE_OFF	EQU	H'3FFF'
_PWRTE_ON	EQU	H'3FEF'
_WDT_ON		EQU	H'3FFF'
_WDT_OFF	EQU	H'3FF7'
_LP_OSC		EQU	H'3FF8'
_XT_OSC		EQU	H'3FF9'
_HS_OSC		EQU	H'3FFA'
_EC_OSC		EQU	H'3FFB'
_INTRC_OSC_NOCLKOUT	EQU	H'3FFC'
_INTRC_OSC_CLKOUT  	EQU	H'3FFD'
_EXTRC_OSC_NOCLKOUT	EQU	H'3FFE'
_EXTRC_OSC_CLKOUT	EQU	H'3FFF'

;***************** User definable variables in this block ***********************************

#define cv19default 0			; no consist as default
#define cv29default B'00000110'	; CV29 bit 0 =0 normal dir
								; CV29 bit 1 =1 28/126 speed steps rather than 14/27
								; CV29 bit 2 =1 DC analogue rather than DC brake
								
#define Acksecs  .6		; Acknowledge time in mS
AckTime set (Acksecs * D'14') / D'10'	; 1.4 times round the loop approx 1mS @4MHz

;--------------- Do not change anything below this line ----------------------------------------
;-----------------------------------------------------------------------------------------------

; CONSTANTS

#define	F0_base		.33	; Function 0 uses CV 33 and 34
#define	F0_revoffset .1	; CV34 is next to CV33
#define	F1_base		.35	; Function 1-4 uses CV 35-38
#define	F5_base		.39	; Function 5-8 uses CV 39-42
#define	F9_base		.43	; Function 9-12 uses CV 43-46
#define	F13_base	.83	; Function 13-16 uses CV 83-86 *TemP*
#define	F17_base	.87	; Function 17-20 uses CV 87-90 *TemP*
#define	F21_base	.91	; Function 21-24 uses CV 91-94 *TemP*
#define	F25_base	.95	; Function 25-28 uses CV 95-98 *TemP*
#define Invert		.99	; Output invertion CV99
#define LokMaus2	.71 ; EEPROM location for LokMaus hundreds
#define Flicker		.100; CV100 used for flicker map, 0=flicker


; VARIABLES


#define nopreamb	D'19'	; No. of preamble half bits = 20
#define	Seed		.29		; Seed for random number generator
;Bit positions
#define norm_rev	.0		; CV29 Bit0 normal or reverse operation
#define fl_control	.1		; CV29 Bit1 FL control = 0 FL in speed14,=1 speed28 FL in function one
#define DCmode		.2		; CV29 bit2 Analogue DC mode enable (disable brake on DC support)
#define long_adr	.5
#define cv21_bit	.6		; CV21 if 1 than consist adr controls Functions


; Pin connection of 12F629/630 Function decoder
; gp0 = Function output F (or track input comparator mode)
; gp1 = Function output E (or track input comparator mode)
; gp2 = Function output D
; 
; gp3 = Track input (non comparator mode)
; gp4 = Function output B
; gp5 = Function output A

; Extra definitions for 14 pin chips
; PC0 = Function output H
; PC1 = Function output G
; PC2 = Function output F
; PC3 = Function output E
; PC4 = Function output D (again)
; PC5 = Function output C

#define gp0	0		; Function F (or Track with comparator)
#define gp1	1		; Function E (or Track with comparator)
#define gp2	2		; Function D
#define gp3	3		; Track (non comparator)
#define gp4	4		; Function B
#define gp5	5		; Function A


;******************************************************
; Bit Positions in CONFIG0, status flags
#define bitrec		0	; What value has just rec. bit
#define CVaccessBit	1	; Two consecutive packets to be send
#define rev_bit		2	; watch CV3/4 when changing direction set by new speed packet
#define cst_rev		3	; Consist Reverse Mode
#define cst_mode	4	; consist mode
#define cst_adr		5	; valid consist adr
#define resetflag	6	; Signal reset for service mode
#define smflag		7	; Service Mode Flag
; Bit positions in FLAG
#define FourByte	0	; 1 indicates Direct CV mode in service mode
#define	rev_mode	1	; 1 indicates that controller is set to reverse
#define	brake		2	; Stores current polarity of track
;******************************************************
; RAM locations. EEPROM routine uses 0x1A  to 0x1F ! <- is this 12C509?

		cblock	0x20
CONFIG0				; Contains various bits to test
ENDVAL				; Offset to byte check routines
PRECOUNT			; Preamble counter
STATE				; Where are we in the DCC package
DATA1				; First transm. byte
DATA2				; Second trans. byte
DATA3				; Third transm. byte
DATA4				; Fourth transm. byte
DATA5				; Fifth transm. byte
DATA6				; Sixth transm. byte
SAMPLES				; How many samples taken for one or zero
TEMP				; Temporary scratch reg
TEMP1				; ditto
TEMP2				; ditto
TEMP3				; ditto
COUNTER    			; 

CVACCESSRATE
TIMEOUTCOUNT
CV14
CV19
CV21				; F1-F8 consist active status
CV22				; F9-F12 and F0, "  but shifted up 2-bits
CV29
MYEEDATA
FnSTATE
FnMASK
FLAG				; bit 0=1 four or more bytes to decode, bit 1=1 reverse direction called
TestByte
INVERTMAP			; Holds contents of invertmap CV to save on EEPROM reads
OUTPUT				; Values to be applied to output function pins
FlickMASK			; Copy of flicker CV
RANDOM				; Random number updated by flicker routine
		endc
;************************************************************************************************
;************* Here we start ! ******************************************************************************

	org	0
START:
	bsf	STATUS,RP0
	call	0x3FF
	movwf	OSCCAL		;put calibration value into OSCCAL
	bcf	STATUS,RP0

;************* setup sets all what is ness. *****************************************************************
dccsetup:
; Temporarily removed RAM clearance to save space
	movlw	0x20		; bottom of RAM
	movwf	FSR
Next1:
	clrf	INDF
	incf	FSR,F
	movlw	0x60		; end of ram +1
	subwf	FSR,W
	btfss	STATUS,C
	goto	Next1

	movlw	Seed		; Get seed for random number generator
	movwf	RANDOM
	movlw	nopreamb	; Preamble = 20 half bits
	movwf	PRECOUNT
; call master init and read contents of EEPROM into RAM mirrors
	call	ChkPowerUp	; Check EEPROM for consistancy and refresh if required
	call	ReadNMRA	; Load CVs into RAM image buffer for quick access
; Switch off all outputs by loading invert mask into output buffer
	movf	INVERTMAP,W		; Get invertmap	
	movwf	FnSTATE			; Save mask in output buffer
	call	Function_activate	; Activate the outputs
	btfsc	CV29,DCmode		; Check for DC mode
;	movlw	0x7			; set GP2:0 to digital I/O and disable comparator
;	movwf	CMCON
;	clrf	PORTC
	bsf	STATUS,RP0
;	movlw	B'00001000'	; gp0,1,2,4,5 are outputs ...
;	movwf	TRISIO		; ... gp3 dcc data in if comparator not used
	movlw	B'11000000'	; 
	clrf	TRISC		; Set PC5:0 output on 14 pin devices (16F630/676)
;	bcf	STATUS,RP0

;*** Removed support for GP3 input to save code space ***

; Need to test here for GP2 connected to GP3 and set up input for comparator
; GP2 is pulsed with a very short pulse which is not part of the DCC signal
;	clrf	GPIO		; Set GP2 low
;	btfsc	GPIO,GP3	; Test that GP3 is also low
;	goto	starthi		; not low so pins not linked, GP3 is the input
;	bsf		GPIO,GP2	; Set GP2 high
;	btfss	GPIO,GP3	; Test that GP3 is also high
;	goto	starthi		; not high so pins not linked, GP3 is the input
;	clrf	GPIO		; Set GP2 low again
;	btfsc	GPIO,GP3	; Test that GP3 has returned low
;	goto	starthi		; not low so pins not linked, GP3 is the input
; Set up for comparator as input rather than GP3
;	bsf		STATUS,RP0
	movlw	B'00001011'	; gp2:5 are outputs, GP3 and GP0:1 inputs
	movwf	TRISIO		; 
	bcf		STATUS,RP0
	movlw	0x2			; Leave GP2 as IO and set GP1:0 to comparator in
	movwf	CMCON
; Set bit somewhere to flag to use CMCON,COUT instead of GP3 possibly

; Drop through to starthiC

;************************************************************************************************************
; High Bit Level Section with comparator
starthiC:
	call	Value		;4,5
conthiC:
	btfss	CMCON,COUT	;1	HighLevel ?
	goto	startloC		;2,3	No
	call	Speed_Sub	;3,4
	goto	conthiC		;21,22
;----------------------------------------------------------------------------------------------------
; Low Level Bit Section with comparator
startloC:
	call	Value		;4,5
contloC:
	btfsc	CMCON,COUT	;1
	goto	starthiC		;2,3
	call	Speed_Sub	;3,4
	goto	contloC		;21,22
;----------------------------------------------------------------------------------------------------
; High Bit Level Section with GP3
;starthi:
;	call	Value		;4,5
;conthi:
;	btfss	GPIO,gp3	;1	HighLevel ?
;	goto	startlo		;2,3	No
;	call	Speed_Sub	;3,4
;	goto	conthi		;22,23
;----------------------------------------------------------------------------------------------------
; Low Level Bit Section with GP3
;startlo:
;	call	Value		;4,5
;contlo:
;	btfsc;	GPIO,gp3	;1
;	goto	starthi		;2,3
;	call	Speed_Sub	;3,4
;	goto	contlo		;22,23
;----------------------------------------------------------------------------------------------------
				;Value checks for correct bit and jumps to where we are in DCC package
				;Computed goto has to reside wholy within page 0, so make sure not too
				;much code is added in front.
Value:
	clrf	SAMPLES		; 6
	movf	STATE,W		; 7
	addwf	PCL,F		; 8,9
				;Jump table to routines from where we do a RETURN !
				;cycl. to here ->  cycl for rout.
	goto	waitn		; 10,11
	goto	waitlo
	goto	testlo
; First byte
	goto	bitset		; Half bit
	goto	lastv		; Bit 0
	goto	bitset		; Half bit
	goto	lastv		; Bit 1
	goto	bitset		; Half bit
	goto	lastv		; Bit 2
	goto	bitset		; Half bit
	goto	lastv		; Bit 3
	goto	bitset		; Half bit
	goto	lastv		; Bit 4
	goto	bitset		; Half bit
	goto	lastv		; Bit 5
	goto	bitset		; Half bit
	goto	lastv		; Bit 6
	goto	bitset		; Half bit
	goto	lastx		; Bit 7 so Byte received
; Seperator bits
	goto	end11		; Check first half bit of byte 1-2 seperator
	goto	end12		; Check second half bit of byte 1-2 seperator
	goto	end11		; Check first half bit of byte 2-3 seperator
	goto	end22		; Check second half bit of byte 2-3 seperator
	goto	end31		; Check first half bit of byte 3-4 seperator
	goto	end32		; Check second half bit of byte 3-4 seperator
	goto	end31		; Check first half bit of byte 4-5 seperator
	goto	end42		; Check second half bit of byte 4-5 seperator
	goto	end31		; Check first half bit of byte 5-6 seperator
	goto	end52		; Check second half bit of byte 5-6 seperator
	goto	end61		; Check first half bit of byte 6 terminator
	goto	end62		; Check second half bit of byte 6 terminator
; Decoding will occur after six bytes. If another byte is signalled there will be a frame error

; I guess that nothing should ever get here, but if it does it will check CV8 and return

;******************************************************************************************

ChkPowerUp:
; New, more compact set to factory defaults procedure

; Set the page register to 1 at power on. Page mode will not power cycle between
; setting the page register and programming the CV. A power cycle reset will ensure
; that register and address only modes will always program CV1-4.

; Computed goto so must remain wholy within page 0

	movlw	.1			; Page register default is 1
	movwf	MYEEDATA
	movlw	.0			; Page register uses EEPROM location 0
	call	EEPROM_WRITE

; Check that CV8 has not been changed to force a factory reset
	movlw	0x8		 ; CV8 will contain the manufacturer ID if all is sound
	call	EEPROM_READ
	xorlw	ManId
	btfsc	STATUS,Z
	return			 ; CV 8 is set correctly so return without setting factory defaults

; Set CVs to default values
	movlw	.255		; CV8=255 while resetting to factory default
	movwf	MYEEDATA
	movlw	.8
	call	EEPROM_WRITE

	clrf	TEMP			; TEMP is used as a counter for the table
Next_default:
	Call	GetCVDefault	; Get CV number
	movwF	TEMP1
	incf	TEMP,F
	Call	GetCVDefault	; Get CV contents
	movwf	MYEEDATA
	movf	TEMP1,W
	call	EEPROM_WRITE	; Write CV
	incf	TEMP,F
	movlw	.8
	xorwf	TEMP1,W
	btfsc	STATUS,Z		; Check for CV8, always last
	return					; Done if CV8 done
	goto	Next_default	; Do next CV

GetCVDefault:
	movf	TEMP,W	;
	addwf	PCL,F	;

; Table format is:- 	retlw CV_Number
;						retlw CV_value

	retlw	.1
	retlw	.3		; CV1=3

	retlw	.7
	retlw	VerNo	; CV7 = version number

	retlw	.13
	retlw	b'11111111'	; CV13 = F1-8 active in DC analogue mode

	retlw	.14
	retlw	b'00111111'	; CV14 = F0,9-12 active in DC analogue mode

	retlw	.15
	retlw	.0		; CV15=0 decoder lock default

	retlw	.16
	retlw	.0		; CV16=0 decoder lock default

	retlw	.17
	retlw	.0		; CV17=0 and CV18=0 long address 0000

	retlw	.18
	retlw	.0

	retlw	.19
	retlw	.0		; CV19=0

	retlw	.21
	retlw	b'11111111'	; CV21 = F1-8 active in consist mode

	retlw	.22
	retlw	b'00111111'	; CV22 = F0 and F9-12 active in consist mode

	retlw	.29
	retlw	.6		; CV29=6

; CV 33-46 are function to output mappings similar to the NMRA method. Bit positions are
; the same as in port C register where possible to keep life simple rather than NMRA method
; Output A = GP5/PA5 = value of 128
; Output B = GP4/PA4 = value of 64
; Output C = PC5 = value of 32
; Output D = GP2/(PC4) = value of 16
; Output E = (GP1)/PC3 = value of 8
; Output F = (GP0)/PC2 = value of 4
; Output G = PC1 = value of 2
; Output H = PC0 = value of 1

	retlw	F0_base
	retlw	b'10000000'	; CV33 = F0 forward -> Output A

	retlw	F0_base + 1
	retlw	b'01000000'	; CV34 = F0 reverse -> Output B

	retlw	F1_base
	retlw	b'00100000'	; CV35 = F1 -> Output C

	retlw	F1_base + 1
	retlw	b'00010000'	; CV36 = F2 -> Output D

	retlw	F1_base + 2
	retlw	b'00001000'	; CV37 = F3 -> Output E

	retlw	F1_base + 3
	retlw	b'00000100'	; CV38 = F4 -> Output F

	retlw	F5_base
	retlw	b'00000010'	; CV39 = F5 -> Output G

	retlw	F5_base + 1
	retlw	b'00000001'	; CV40 = F6 -> Output H

	retlw	F5_base + 2
	retlw	b'00000000'	; CV41 = F7 -> No Output

	retlw	F5_base + 3
	retlw	b'00000000'	; CV42 = F8 -> No Output

	retlw	F9_base
	retlw	b'00000000'	; CV43 = F9 -> No Output

	retlw	F9_base + 1
	retlw	b'00000000'	; CV44 = F10 -> No Output

	retlw	F9_base + 2
	retlw	b'00000000'	; CV45 = F11 -> No Output

	retlw	F9_base + 3
	retlw	b'00000000'	; CV46 = F12 -> No Output

	retlw	F13_base
	retlw	b'00000000'	; CV83 = F13 -> No Output

	retlw	F13_base + 1
	retlw	b'00000000'	; CV84 = F14 -> No Output

	retlw	F13_base + 2
	retlw	b'00000000'	; CV85 = F15 -> No Output

	retlw	F13_base + 3
	retlw	b'00000000'	; CV86 = F16 -> No Output

	retlw	F17_base
	retlw	b'00000000'	; CV87 = F17 -> No Output

	retlw	F17_base + 1
	retlw	b'00000000'	; CV88 = F18 -> No Output

	retlw	F17_base + 2
	retlw	b'00000000'	; CV89 = F19 -> No Output

	retlw	F17_base + 3
	retlw	b'00000000'	; CV90 = F20 -> No Output

	retlw	F21_base
	retlw	b'00000000'	; CV91 = F21 -> No Output

	retlw	F21_base + 1
	retlw	b'00000000'	; CV92 = F22 -> No Output

	retlw	F21_base + 2
	retlw	b'00000000'	; CV93 = F23 -> No Output

	retlw	F21_base + 3
	retlw	b'00000000'	; CV94 = F24 -> No Output

	retlw	F25_base
	retlw	b'00000000'	; CV95 = F25 -> No Output

	retlw	F25_base + 1
	retlw	b'00000000'	; CV96 = F26 -> No Output

	retlw	F25_base + 2
	retlw	b'00000000'	; CV97 = F27 -> No Output

	retlw	F25_base + 3
	retlw	b'00000000'	; CV98 = F28 -> No Output

	retlw	Invert
	retlw	b'00000000'	; CV99  No inversions

	retlw	Flicker
	retlw	b'11111111'	; CV99  No flickering

; CV8 must always be last. CV8=255 while setting to defaults is in progress
	retlw	.8		 ; stamp power up by rewriting manufacturer ID to CV8
	retlw	ManId


;************************************************************************************************************
; Each of the following state routines enters on step 12 and exits with step 22

				; waitn waits for 20 half bits of the preamble
waitn:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when preamble is received
	nop						;13
	btfss	CONFIG0,bitrec	;14   there are only ones in preamble
	goto	waitn1			;15,16
	decfsz	PRECOUNT,F		;16
	goto	ret4			;17,18
	movlw	nopreamb		;18
	movwf	PRECOUNT		;19
	incf	STATE,F			;20
	return					;21,22

waitn1:
	movlw	nopreamb		;17
	movwf	PRECOUNT		;18

ret4:
	nop						;19
	nop						;20
	return					;21,22

;************************************************************************************************************
				; waitlo waits for low half bit after preamble

waitlo:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	nop						;13
	nop						;14
	nop						;15
	nop						;16
	nop						;17
	btfsc	CONFIG0,bitrec	;18	must be a zero
	goto	ret5			;19,20	might be long preamble so go round again
	incf	STATE,F			;20
ret5:
	return					;21,22

;************************************************************************************************************
				; testlo test second half bit of start bit

testlo:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	nop						;13
	nop						;14
	nop						;15
	nop						;16
	nop						;17
	btfsc	CONFIG0,bitrec	;18	must be a zero
	goto	frameerr		;19 incomplete zero bit so no good
	incf	STATE,F			;20
	return					;21,22

;************************************************************************************************************
				; bitset takes first half bit of a bit in the byte

bitset:	
	incf	STATE,F			;12
	bcf	STATUS,C			;13
	btfsc	CONFIG0,bitrec	;14
	bsf	STATUS,C			;15
	rlf	DATA6,F				;16
	nop						;17
	nop						;18
	nop						;19
	nop						;20
	return					;21,22

;************************************************************************************************************
				; lastv checks that first half bit = second half bit
lastv:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	nop						;13
	nop						;14
	incf	STATE,F			;15
	btfss	CONFIG0,bitrec	;16
	goto	lastv1			;17,18
	btfss	DATA6,0			;18
	goto	frameerr		;19,20 error if half bits not equal
	nop						;20
	return					;21,22

lastv1:
	btfsc	DATA6,0			;19
	goto	frameerr		;20,21	error if half bits not equal
	return					;21,22
;************************************************************************************************************
				; lastx checks that first half bit = second half bit
				; and sets offset to byte check routine
lastx:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	incf	ENDVAL,F		;13
	movf	ENDVAL,W		;14
	addwf	STATE,F			;15
	btfss	CONFIG0,bitrec	;16
	goto	lastx1			;17,18
	btfss	DATA6,0			;18
	goto	frameerr		;19	error if half bits not equal
	nop						;20
	return					;21,22

lastx1:
	btfsc	DATA6,0			;19
	goto	frameerr		;20	error if half bits not equal
	return					;21,22

;************************************************************************************************************
				; end11 end of first byte there must be zero
				; end11 first half bit. end12 second half bit
end11:
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	btfsc	CONFIG0,bitrec	;13
	goto	frameerr		;14,15 first half bit is not zero
	incf	STATE,F			;15
	incf	ENDVAL,F		;16
	nop						;17
	nop						;18
	nop						;19
	nop						;20
	return					;21,22

end12:	
	btfsc	CONFIG0,bitrec	;12
	goto	frameerr		;13,14 second half bit is not zero
	movf	DATA6,W			;14
	movwf	DATA1			;15
	movlw	0x03			;16	point STATE to beginning of jump table
	movwf	STATE			;17
	bcf	FLAG,FourByte		;18
	nop						;19
	nop						;20
	return					;21,22

;************************************************************************************************************
				; end21 end of second byte there must be zero
				; end21 first half bit. end22 second half bit

; end21: is the same as end11:, so use end11:

end22:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	btfsc	CONFIG0,bitrec	;13
	goto	frameerr		;14,15 second half bit is not zero
	movf	DATA6,W			;15
	movwf	DATA2			;16
	movlw	0x03			;17	point STATE to beginning of jump table
	movwf	STATE			;18
	nop						;19
	nop						;20
	return					;21,22

;************************************************************************************************************
				; end31 end of third byte there must be a one if last byte, or 0 if more
				; end31 first half bit. end32 second half bit
				; there does not appear to be any testing of the first half bit.
				; Use spare cycles to flag only three bytes.
end31:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	incf	STATE,F			;13
	incf	ENDVAL,F		;14
	nop						;15
	nop						;16
	nop						;17
	nop						;18
	nop						;19
	nop						;20
	return					;21,22

end32:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	movf	DATA6,W			;13
	movwf	DATA3			;14
	btfsc	CONFIG0,bitrec	;15	if 0 other bytes will follow
	goto	end32x			;16,17 not zero so prepare to decode
	movlw	0x03			;17	point STATE to beginning of jump table
	movwf	STATE			;18
	nop						;19
	nop						;20
	return					;21,22

end32x: 
	clrf	STATE			;18	reset STATE for next preamble
	clrf	ENDVAL			;19
	clrf	DATA4			;20
	clrf	DATA5			;21
	clrf	DATA6			;22
	goto	decode			;23,24	no need here for precise cycle counting, because we decode now
;************************************************************************************************************
				; end41 end of forth byte there must be a one if last byte, or 0 if more
				; end41 first half bit. end42 second half bit
				
				; there does not appear to be any testing of the first half bit	

; end41: is the same as end31:, so use end31:

end42:	
	movf	DATA6,W			;12
	movwf	DATA4			;13
	btfsc	CONFIG0,bitrec	;14	if 0 other bytes will follow
	goto	end42x			;15,16 not zero so prepare to decode
	movlw	0x03			;16	point STATE to beginning of jump table
	movwf	STATE			;17
	bsf	FLAG,FourByte		;18	Indicate that four or more bytes have been decoded
	nop						;19
	nop						;20
	return					;21,22

end42x: 
	clrf	STATE			;17	reset STATE for next preamble
	clrf	ENDVAL			;18
	clrf	DATA5			;19
	clrf	DATA6			;20
	bsf	FLAG,FourByte		;21	Indicate that four bytes have been decoded
	goto	decode			;22,23	no need to count, because we decode now
;************************************************************************************************************
				; end51 end of fifth byte there must be a one if last byte, or 0 if more
				; end51 first half bit. end52 second half bit
				
				; there does not appear to be any testing of the first half bit	

; end51: is the same as end31:, so use end31:

end52:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	movf	DATA6,W			;13
	movwf	DATA5			;14
	btfsc	CONFIG0,bitrec	;15	if 0 other bytes will follow
	goto	end52x			;16,17 not zero so prepare to decode
	movlw	0x03			;17	point STATE to beginning of jump table
	movwf	STATE			;18
	nop						;19
	nop						;20
	return					;21,22

end52x: 
	clrf	STATE			;18	reset STATE for next preamble
	clrf	ENDVAL			;19
	clrf	DATA6			;20
	bsf	FLAG,FourByte		;21	Indicate that four bytes have been decoded
	goto	decode			;22,23	no need to count, because we decode now
;************************************************************************************************************
				; end61 end of sixth byte there must be a one since last byte
				; end61 first half bit. end62 second half bit

end61:	
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	btfss	CONFIG0,bitrec	;13
	goto	frameerr		;14,15 first half bit is not one
	incf	STATE,F			;15
	incf	ENDVAL,F		;16
	nop						;17
	nop						;18
	nop						;19
	nop						;20
	return					;21,22

end62:
	clrf	TIMEOUTCOUNT	;12 clear DC timeout when half bit received
	btfss	CONFIG0,bitrec	;13	test correct ending
	goto	frameerr		;14,15 second half bit is not one
	clrf	STATE			;15	reset STATE for next preamble
	clrf	ENDVAL			;16
	goto	decode			;17,18	no need to count, because we decode now
;************************************************************************************************************
				; Frame error in packet
frameerr:
	movlw	nopreamb
	movwf	PRECOUNT
	clrf	STATE
	clrf	ENDVAL
	clrf	DATA1
	clrf	DATA2
	clrf	DATA3
	clrf	DATA4
	clrf	DATA5
	clrf	DATA6
	clrf	SAMPLES
; clear packet status bits in CONFIG0 Byte
	bcf	CONFIG0,bitrec
	bcf	CONFIG0,CVaccessBit
	bcf	CONFIG0,rev_bit
;	movlw	b'11111000'		; Old code here
;	andwf	CONFIG0,F
	return

;******************************************************
;************* DECODING *******************************
;******************************************************

decode:
	movf	DATA1,W		; Exclusive or check
	xorwf	DATA2,W
	xorwf	DATA3,W
	xorwf	DATA4,W
	xorwf	DATA5,W
	xorwf	DATA6,W
	btfss	STATUS,Z
	goto	exit_bank1	; error in packet

chkconsist:
	bcf	CONFIG0,cst_mode
	movlw	.19			; Read CV19
	call	EEPROM_READ
	movwf	TEMP
	andlw	B'01111111'
;	iorlw	0			<- implied!
	btfsc	STATUS,Z
	goto	chkadr		; no consist address set in CV19
	bsf	CONFIG0,cst_mode; Consist address is active in CV19
	bsf	CONFIG0,cst_rev
	btfss	TEMP,7		; reverse mode?
	bcf	CONFIG0,cst_rev

	xorwf	DATA1,W
	btfss	STATUS,Z
	goto	chkadr		; Not consist, so check as baseline
	bsf	CONFIG0,cst_adr	; valid consist address
	goto	decode_1	; Decode packet as consist

chkadr:
	movlw	.112
	subwf	DATA1,W
	btfss	STATUS,C	; Less than 112 is a normal short address
	goto	chknormal
	movlw	.128
	subwf	DATA1,W
	btfss	STATUS,C	; Greater than 127 is a long address or accessory
	goto	check_sm	; 112-127 is short address but could be service mode
	movlw	B'11000000'
	andwf	DATA1,W
	xorlw	B'11000000'
	btfsc	STATUS,Z
	goto	chk_longadr	; 192 or Greater is long address
	goto	exit_bank1	; 128-191 is accessory decoder

chknormal:
	movf	DATA1,W		; Set ZERO Flag if a broadcast address
	btfsc	STATUS,Z
	goto	decode_1	; zero is valid Broadcast address
;................................
baseline:
	movlw	.1
	call	EEPROM_READ
	xorwf	DATA1,W
	btfss	STATUS,Z
	goto	exit_bank1
	bcf		CONFIG0,cst_adr	; baseline address
	btfsc	CV29,long_adr	; Is decoder using a long address?
	goto	exit_bank1		; Yes - ignore short address.
	goto	decode_1		; No - decode packet.
;................................

chk_longadr:
	btfss	CV29,long_adr	; Is decoder using a long address?
	goto	exit_bank1		; No
	btfsc	DATA1,3		; 128-191
	goto	exit_bank1
	movlw	.17			; CV17
	call	EEPROM_READ	; Get long address to compare
	xorwf	DATA1,W
	btfss	STATUS,Z
	goto	exit_bank1
	movlw	.18			; CV18
	call	EEPROM_READ
	xorwf	DATA2,W
	btfss	STATUS,Z
	goto	exit_bank1
	bcf		CONFIG0,cst_adr	; baseline address
; Shift data bytes down to take account of longer address. Decoding assumes short address
	movf	DATA3,W
	movwf	DATA2
	movf	DATA4,W
	movwf	DATA3
	movf	DATA5,W
	movwf	DATA4
; Next two probably not required because DATA6 will only contain the checksum
;	movf	DATA6,W
;	movwf	DATA5

; Decoder information originaly in DATA1, DATA2 and DATA5 no longer valid
; Command is now in DATA2, Primary data in DATA3 and extended data in DATA4 

;----- Command Decode -------------

decode_1:
	movf	DATA2,W		; Command decoding jump table
	andlw	0xE0		; Isolate command
decode1:
	xorlw	.0			; Consist (000.....)
	btfsc	STATUS,Z
	goto	ConsistGroup

	bcf	CONFIG0,resetflag
	bcf	CONFIG0,smflag

	xorlw	.64			; Speed reverse (010.....) 14/28 speedsteps
	btfsc	STATUS,Z
	goto	Speed_rev

	xorlw	.32			; Speed forward (011.....) 14/28 speedsteps
	btfsc	STATUS,Z
	goto	Speed_for

	xorlw	.224		; Function group one (100.....)
	btfsc	STATUS,Z
	goto	Function_one

	xorlw	.96			; CV access (111.....)
	btfsc	STATUS,Z
	goto	CVaccess

	xorlw	.192		; Advanced (001.....) 126 speedsteps
	btfsc	STATUS,Z
	goto	Speed_126	; Convert to 28 step command

	xorlw	.128		; Function group two (101.....)
	btfsc	STATUS,Z
	goto	Function_two

	; here by default
	goto	Function_three	; Future expansion command (110.....), F13-F28 only

;*****************************************************************************************************************


;*****************************************************************************************************************

EEPROM_WRITE:
; Address in W, DATA in MYEEDATA
	bsf	STATUS,RP0
;	movwf	EEADR		<- superfluous if reading first
	call	EEPROM_READ	; get previously stored value
	bcf	STATUS,RP0
	xorwf	MYEEDATA,W	; Compare with new value
	btfsc	STATUS,Z	; If zero no need to write
	return				; No need to write so return
	movf	MYEEDATA,W	; Load value to be written
	bsf	STATUS,RP0
	movwf	EEDATA
	bsf	EECON1,WREN
	movlw	0x55
	movwf	EECON2
	movlw	0xAA
	movwf	EECON2
	bsf	EECON1,WR		; Instigate write sequence
wait_ee_write:
	btfsc	EECON1,WR	; Wait for write to complete
	goto	wait_ee_write
	bcf	STATUS,RP0
	return

EEPROM_READ:
; Address in W
; return data in W
	bsf	STATUS,RP0
	movwf	EEADR
	bsf	EECON1,RD
	movf	EEDATA,W
	bcf	STATUS,RP0
	return

;************************************************************************************************************
ReadNMRA:
	movlw	.19			; CV19 = Consist address
	call	EEPROM_READ
	movwf	CV19

	movlw	.21			; CV21 = Consist F1-F8 mapping
	call	EEPROM_READ
	movwf	CV21

	movlw	.22			; CV22 = Consist F0 and F9-F12 mapping
	call	EEPROM_READ
	movwf	CV22
	rlf		CV22,F		; Shift up a couple of bits to make it easy later
	rlf		CV22,F		; 

	movlw	.29			; CV29 = Configuration bits
	call	EEPROM_READ
	movwf	CV29

	movlw	Invert		; CV99 = output invert map
	call	EEPROM_READ
	movwf	INVERTMAP

	movlw	Flicker		; CV100 = flicker map
	call	EEPROM_READ
	movwf	FlickMASK

	goto	exit_bank1	; start sampling

;************* SERVICE MODE *******************************************

; Direct CV (4 byte):-		DATA1      DATA2      DATA3      DATA4
;				1110CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1

; Legacy modes (3 byte):-	DATA1      DATA2      DATA3     
;				1110CAAA 0 DDDDDDDD 0 EEEEEEEE 1



check_sm:

; Might need to check here for short address 112-127 and go to chk_normal

	btfss	CONFIG0,resetflag; check for SM, reset packet has to come first
	goto	chknormal

	btfss	CONFIG0,smflag	; Second SM packet
	goto	set_sm_flag

; Direct mode is four bytes, but other modes are only three (inc check byte)
; Use FLAG,FourByte=1 to indicate Direct CV mode

	btfss	FLAG,FourByte	; Direct CV Mode if set, else not enough bytes so legacy
	goto	ad_mode_write	; Go to legacy address only/register/paged mode

; Direct CV mode. This bit manipulates the data to be the same as POM (short address).
dm_mode_write:			; Drop through to Direct CV mode
	movf	DATA3,W		; Data in Byte 3
	movwf	DATA4		; now CV contents in DATA 4
	movf	DATA2,W		; CV number in Byte 2, 0=CV1
	movwf	DATA3		; Now most of CV number (all the useful bit) in DATA3
	movf	DATA1,W		; Command in Byte 1
	movwf	DATA2		; Command now in DATA2
	goto	DoCV		; Carry on as if POM

; Address only/register/paged mode. This part manipulates the data to be the same as POM.
; Losing support for modes other than Direct CV will save about 50 program words! 

ad_mode_write:
	movf	DATA2,W		; Data in byte 2
	movwf	DATA4		; Now CV contents in DATA4
	movlw	B'00001100'	; Write command for POM/Direct CV ready 
	btfss	DATA1,3		; Test that write mode is what is required
	movlw	B'00000100'	; Verify command for POM/Direct CV if not write mode
	movwf	DATA2		; Command Verify or Write now in DATA2
	movf	DATA1,W		; Get register address from command byte 
	andlw	b'00000111'	; Mask off CV address (lower 3 bits)
	movwf	DATA3		; Put CV address in DATA3 (0=register 1)
	btfsc	DATA3,2		; Test for CV1-4 (paged bank) or CV5-8 (control bank)
	goto	Not_page	; Not in the paged bank, so sort out CV541 and page register
; This bit adds on the page offset
	movlw	.0			; Lets see what is in the page register, EEPROM location 0
	call	EEPROM_READ	; Read the page register
	movwf	TEMP
	decf	TEMP,F		; Take off one from page register to get multiplier
	movlw	b'11100000'	; Check for out of range page register
	andwf	TEMP,W		; Zero result for valid page value
	btfss	STATUS,Z	; Carry on if page in range
	goto	sm_error	; Out of range so reset and don't program
	bcf		STATUS,C	; Clear carry ready for multiply
	rlf		TEMP,F		; times 2
	bcf		STATUS,C	; Clear carry ready for multiply
	rlf		TEMP,W		; Times 2 again (2x2=4). Page multiplier now in top 6 bits of W
	iorwf	DATA3,F		; Data 3 should now be 0PPPPPAA, or CV address, 0=CV1
	goto	DoCV		; Carry on as if POM

Not_page:	; W contains CV address 100-111, CV541,page register,CV7 and CV8
	btfsc	DATA3,1		; Test for CV541/page register or CV7-8
	goto	DoCV		; 11X, CV7 or 8, Carry on as if POM
	btfsc	DATA3,0		; Test for CV541 or page register
	goto	Set_page	; 101, set page register
	movlw	.28			; CV541 POM address = 28
	movwf	DATA3		
	goto	DoCV		; Carry on as if POM

Set_page
; New method, set CV to CV0 by using value -1 in DATA3
	movlw	B'11111111'	; -1 in binary, will increment to 0
	movwf	DATA3		; Change from 101 (register 6) to page register
	goto	DoCV		; carry on as normal

;******************CV Access group*******************************

; Routine for 'Operations mode' or 'Programming on the main' and 'Direct CV' modes
; all modes converted to be compatible with short address POM for DoCV routine

;POM (short address) :-	DATA1      DATA2      DATA3      DATA4      DATA5
;			0aaaaaaa 0 1110CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
;
;POM (long address) :-	DATA1      DATA2      DATA3      DATA4      DATA5      DATA6
;			11aaaaaa 0 aaaaaaaa 0 1110CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1

; The CV address is 1 less than the CV number so address 00 00000000 = CV1
; CC Field, 11=Write, 01=Verify, 10=Bit manipulation (not yet tested)
; a=decoder address, A=CV address, D=CV contents, E=checksum

; For bit manipulation the DDDDDDDD field is broken up into 111KDBBB where D is the
; bit value, BBB is the bit position within the CV, K=1 means write, K=0 means verify


CVaccess:
	btfss	CONFIG0,cst_mode	; chk if baseline address ok
	goto	CVaccess1
	btfsc	CONFIG0,cst_adr		; if consist adr then no POM CVaccess
	goto	exit_bank1
CVaccess1:
	btfsc	CONFIG0,CVaccessBit	; second packet received ?
	goto	DoCV
	bsf	CONFIG0,CVaccessBit
	goto	exit_bank1	
DoCV:
	bcf	CONFIG0,CVaccessBit
Test_CV:
; LokMaus2 support mode here getting value from 71 that was written to CV7
;
; Process is to write hundreds columns to CV7 then use tens and units to write CV
;
;            CV7 = Hh                   <- First write
;                 /  \
;                /    \
;       CV No = HTU    htu = CV value
;                ||     ||
;        CV No = TU     tu = Value      <- Second write
;
; This will result in value htu being put in CV HTU
;
; Bit values in EEPROM 71 (hex 47) indicate the following:-
;	0 (1) = 1,  Add 100 to CV contents
;	1 (2) = 1,  Add 200 to CV contents if bit 3 = 0 (02 or 22 programmed to CV7)
;	2 (4) = 1,  Add 200 to CV contents if bit 3 = 1 (12 programmed to CV7)
;	3 (8) = 1,  Add 100 to CV number
;	4 (16) = 1, Add 200 to CV number
;
; Subtracting 2 from EEPROM 71 if bit 3 = 1 simplifies things as follows:-
;	0 (1) = 1,  Add 100 to CV contents
;	1 (2) = 1,  Add 200 to CV contents
;	2 (4) = 1,  ignore
;	3 (8) = 1,  Add 100 to CV number
;	4 (16) = 1, Add 200 to CV number
;
; Get CV7 value from EEPROM 71, and subtract 2 if required
	movlw	LokMaus2		; Read CV7 saved value
	call	EEPROM_READ		; Get current contents
	movwf	TEMP			; Save CV7 contents into TEMP
	movlw	.2				; Get 2 ready in case it neads to be subtracted
	btfsc	TEMP,3			; Test for 1x
	subwf	TEMP,F			; Take away 2 to leave bit 1 free of tens column
; Add digits from CV 7 to CV number and value
	movf	DATA4,W			; Get CV value
	btfsc	TEMP,0			; Test for x1
	addlw	.100			; Add 100 to CV value
	btfsc	TEMP,1			; Test for x2
	addlw	.200			; Add 200 to CV value
	movwf	DATA4			; Save modified value
	movf	DATA3,W			; Get CV number
	btfsc	TEMP,3			; Test for 1x
	addlw	.100			; Add 100 to CV number
; Only CV1-127 currently supported, so 2xx will be out of range.
;	btfsc	TEMP,4			; Test for 2x
;	addlw	.200			; Add 200 to CV number
	movwf	DATA3			; Save modified CV number
	clrf	MYEEDATA		; Clear location 'CV7'
	movlw	LokMaus2		; EEPROM CV7 saved location
	call	EEPROM_WRITE	; Write location 'CV7'

	movf	DATA2,W			; Load top two bits
	andlw	B'00000011'		; Mask off from instruction
	btfss	STATUS,Z		; check that they are both zero
	goto	exit_bank1		; CV is above CV256 if non zero
	movf	DATA3,W			; load lowest 8 bits of CV address
	movwf	TEMP
	incf	TEMP,F			; Address 0 is CV1 so increase to get CV and EEPROM number
	btfsc	TEMP,7			; Valid CV for this decoder is CV1-127
	goto	exit_bank1		; CV is above 127
	movf	DATA2,W
	andlw	B'00001100'
	xorlw	B'00001100'		; CC=11=Write 
	btfsc	STATUS,Z
	goto	Legacy_chk
	xorlw	B'00001000'		; CC=01=Verify 
	btfsc	STATUS,Z
	goto	cv_verify
	xorlw	B'00001100'		; CC=10=Bit manipulation 
	btfss	STATUS,Z
	goto	exit_bank1		; CC=00=no valid CV command

; Convert bit manipulation into standard write/verify commands
	movlw	B'00001100'	; Write command 
	btfss	DATA4,4		; Test that write mode is what is required
	movlw	B'00000100'	; Verify command if not
	movwf	DATA2		; Command Verify or Write now in DATA2

; Convert bit manipulation data field into standard CV write/verify format
	movf	DATA4,W		; Get the bit position number from the data
	andlw	b'00000111'	; Mask off all but the bit position
	movwf	TEMP1		; Store in counter
	movf	TEMP,W		; Get CV number
	call	EEPROM_READ	; Get current contents of CV
	movwf	TEMP3		; Put the current contents into TEMP3
	clrf	TEMP2
	bsf		STATUS,C	; Set a bit in the carry ready to rotate
	incf	TEMP1,F		; Zero is one bit, not zero bits!!!
Multiply:
	rlf		TEMP2,F		; Shift a bit up
	decfsz	TEMP1,F		; Is that enough shifting?
	goto	Multiply	; No
	movf	TEMP2,W		; TEMP2 now contains one bit in the correct position
	iorwf	TEMP3,F		; Set the bit in TEMP3
	xorlw	b'11111111'	; Complement the bit position (a zero now in the bit position)
	btfss	DATA4,3		; Test the bit
	andwf	TEMP3,F		; Clear the bit if it should be zero
	movf	TEMP3,W		; Get the manipulated data field
	movwf	DATA4		; And put it in DATA4 to be processed as a normal 8 bit CV value

Legacy_chk:
; Check for write to CV 1 and reset consist details in CV19 and long address in CV29 if so
	decf	TEMP,W			; Temp contains CV number. CV1-1=0
	btfss	STATUS,Z		; If zero skip to check for four bytes
	goto 	cv_prog			; Not CV1 so program
; Not sure if we should be checking for POM/Direct mode, and just resetting consist anyway
;	btfsc	FLAG,FourByte	; Check for four bytes (CV direct or POM)
;	goto 	cv_prog			; Four bytes so just program

; Need to put this in a subroutine as part of Consist group

	clrf	MYEEDATA		; Put zero in consist address
	movlw	.19				; CV19 = consist address
	call	EEPROM_WRITE	; write CV19
	movlw	.29				; Need to reset consist active bit in CV29
	call	EEPROM_READ		; Get current contents of CV29
	movwf	MYEEDATA		;
	bcf		MYEEDATA,5		; Clear long address enable bit
	movlw	.29				; CV29 = configuration CV
	call	EEPROM_WRITE	; write CV29

cv_prog:
; Check for just verify rather than programming
	btfss	DATA2,3		; 1=write, 0=verify
	goto	cv_verify	; 0, so verify
						; 1 so carry on and write
; Check decoder lock here before writing
; CV number in TEMP, CV value in DATA4
	movlw	.1				; Check for CV1
	xorwf	TEMP,W			; Compare CV number with 1
	btfsc	STATUS,Z		; Not CV1 so check next
	goto	Write_OK		; CV1 so write even if locked
	movlw	.15				; Check for CV15
	xorwf	TEMP,W			; Compare CV number with 15
	btfsc	STATUS,Z		; Not CV15 so check next
	goto	Write_OK		; CV15 so write even if locked
	movlw	.15				; Get CV15 for comparison
	call	EEPROM_READ		; CV15 now in W
	movwf	TEMP2			; Store contents of CV15
	xorlw	.255			; Check for 'broadcast' value
	btfsc	STATUS,Z		; CV15 not 255 so check next
	goto	Write_OK		; CV15 255 so write even if locked
	movlw	.16				; Get CV16 to compare
	call	EEPROM_READ		; CV16 now in W
	xorwf	TEMP2, W		; Compare CV15 and CV16 contents
	btfss	STATUS,Z		; CV15 and CV16 are equal, so write
	goto	exit_bank1		; Not equal so exit without writing
Write_OK:
	movf	DATA4,W			; value in DATA4
	movwf	MYEEDATA
	movf	TEMP,W			; CV number held in TEMP
; Check for CV7 write for LokMaus2 mode
	xorlw	.7
	movlw	LokMaus2		; Get CV7 save location ready
	btfsc	STATUS,Z		; If CV7, use CV7 save location
	movwf	TEMP			; =7 so store 'CV7' value instead
; Check for CV29 write, must not allow illegal bits to be set
;	movf	TEMP,W			; CV number held in TEMP
	xorlw	.29
	movlw	b'00000111'		; Get mask ready 
	btfsc	STATUS,Z		; Is it CV 29?
	andwf	MYEEDATA,F		; Yes, so clear unsuppored bits.

	movf	TEMP,W			; CV number held in TEMP
	call	EEPROM_WRITE	; write CV
;	goto	acknowledge		; Why not load CV number, drop through and auto verify?
;	movf	TEMP,W			; CV number held in TEMP

cv_verify:

	movf	TEMP,W			; Get address of CV to verify
	call	EEPROM_READ		; Read CV
	xorwf	DATA4,W			; Compare CV contents with value
	btfss	STATUS,Z		; equal then ACK
	goto	exit_bank1		; not equal exit
;---------------------------------------------------------------

; Need to add pin 6/11(/5 on 8-pin) acknowledgement only pin here.

acknowledge:
	movlw	0xFF
	movwf	GPIO	; Turn outputs A,B and D on at least
	movwf	PORTC	; Turn on the rest as well
	clrf	TEMP1
	movlw	AckTime		; acktime in ms can be set in *.h file
	movwf	TEMP
loop_ack1:
	decfsz	TEMP1,F
	goto	loop_ack1
	decfsz	TEMP,F
	goto	loop_ack1

	clrf	GPIO	; turn off
	clrf	PORTC
exit_cv:
	bcf	CONFIG0,smflag	; clear SM mode in case
	bcf	CONFIG0,resetflag
	goto	ReadNMRA

set_sm_flag:
	bsf	CONFIG0,smflag
	goto	exit_bank1

sm_error:
	bcf	CONFIG0,resetflag
	bcf	CONFIG0,smflag
	goto	exit_bank1
;*********************** SPEED COMMAND GROUP *********************************************

; 126 step :-		DATA1      DATA2      DATA3      DATA4
; (short address)	0aaaaaaa 0 00111111 0 DSSSSSSS 0 EEEEEEEE 1
;
; 126 step :-		DATA1      DATA2      DATA3      DATA4      DATA5
; (long address) 	11aaaaaa 0 aaaaaaaa 0 00111111 0 DSSSSSSS 0 EEEEEEEE 1

; 28 step :-		DATA1      DATA2      DATA3
; (short address)	0aaaaaaa 0 01DSSSSS 0 EEEEEEEE 1
;
; 28 step :-		DATA1      DATA2      DATA3      DATA4
; (long address) 	11aaaaaa 0 aaaaaaaa 0 01DSSSSS 0 EEEEEEEE 1

; 14 step :-		DATA1      DATA2      DATA3
; (short address)	0aaaaaaa 0 01DLSSSS 0 EEEEEEEE 1
;
; 14 step :-		DATA1      DATA2      DATA3      DATA4 
; (long address) 	11aaaaaa 0 aaaaaaaa 0 01DLSSSS 0 EEEEEEEE 1

; Where a=loco address, D=direction, S=speed information, L=Function 0

; Bit order for 28 step mode SSSSS = 04321, for 14 step SSSS = 3210

Speed_126:
; This bit converts a 126 step speed command to be the same as a 28 step command
	movf	DATA2,W		; Get command
	xorlw	b'00111111'	; Check for 128 speedstep command
	btfss	STATUS,Z	; Is it a 128 speedstep command?
	goto	exit_bank1	; No, it is not. No other commands in this group supported.
	rlf		DATA3,W		; Shift direction bit into C, without shifting DATA3
	rrf		DATA3,F		; Shift speed bits down and shift direction into MSb
	bsf		STATUS,C	; Set middle instruction bit
	rrf		DATA3,F		; Shift it in
	bcf		STATUS,C	; Clear left instruction bit
	rrf		DATA3,F		; Shift it in, and shift LSb into C
	bsf		DATA3,4		; Set LSb in 28 speedstep instruction
	btfss	STATUS,C	; Check LSb in C, is it set?
	bcf		DATA3,4		; No, clear LSb in 28 speedstep instruction
	movf	DATA3,W		; Move newly constructed instruction from DATA3	
	movwf	DATA2		; To DATA2.
	btfsc	DATA2,5		; Check direction, is it forward?
	goto	Speed_for	; Yes, do 28 step forward	
;	goto	Speed_rev	; No, do 28 step reverse.  <- implied goto


; This bit works out from the supplied direction command and configuration bits
; which direction the loco should currently be going physicaly. Result is stored
; in FLAG:1, 1=reverse, 0=forwards

; Important to ignore commands to baseline address when consist is active for
; this group. Test is CONFIG0,cst_adr=0 and CONFIG0,cst_mode=1

Speed_rev:			; Controller says go in reverse

; Only need to accept commands from baseline address if consist mode not set
	btfss	CONFIG0,cst_mode	; Is a consist address set?
	goto	Speed_rev_chk		; No, address must be baseline OK
								; Yes so test for consist address
	btfss	CONFIG0,cst_adr		; Is the address baseline?
	goto	exit_bank1			; Yes, address was baseline so ignore
								; No, address was consist OK
Speed_rev_chk:
	btfsc	CONFIG0,cst_rev
	goto	Speed_for_cst
Speed_rev_cst:
	bcf	CONFIG0,rev_bit
	btfsc	CV29,norm_rev		; Normal or reverse operation ?
	goto	Sp_for1
Sp_rev1:
	bsf		FLAG,rev_mode		; Set flag for reverse mode
	goto	Check14

Speed_for:			; Controller says go forwards
; Only need to accept commands from baseline address if consist mode not set
	btfss	CONFIG0,cst_mode	; Is a consist address set?
	goto	Speed_for_chk		; No, address must be baseline OK
								; Yes so test for consist address
	btfss	CONFIG0,cst_adr		; Is the address baseline?
	goto	exit_bank1			; Yes, address was baseline so ignore
								; No, address was consist OK
Speed_for_chk:
	btfsc	CONFIG0,cst_rev
	goto	Speed_rev_cst
Speed_for_cst:
	bcf	CONFIG0,rev_bit
	btfsc	CV29,norm_rev		; Normal or reverse operation ?
	goto	Sp_rev1
Sp_for1:
	bcf		FLAG,rev_mode		; Clear flag, forward mode
;	goto	Check14				; <- implied

Check14:
; routine if 14 step mode active.
	btfss	CV29,fl_control	; Check CV29 bit 1. 28/128 ?
	call	Function_zero	; No, Function 0 controlled by speed group

	goto	Function_activate

;***********************************************************************************************************************
Function_one:			; Function group one (F0-F4)
	movfw	CV21			; Get function consist status
	andlw	b'00001111'		; Mask off leaving consist mask for just F1-F4
	btfss	CONFIG0,cst_adr	; Check for using consist address, skip if consist
	xorlw	b'00001111'		; Invert mask to get non consist mask
	movwf	FnMASK			; Save mask for F1 to F4
	btfss	CONFIG0,cst_mode	; check if consist is currently active
	clrf	FnMASK			; consist not active so do all functions anyway
	movlw	F1_base				; Set CV base, F1=CV35,F2=CV36,F3=CV37,F4=CV38
	call	Function_block

; Function 0 is a special case that cannot be handled by the subroutine
	btfsc	CV29,fl_control	; Check CV29 bit 1. 28/128 ?
	call	Function_zero	; Yes, Function 0 controlled by function group 1
	call	Function_activate
	goto	exit_bank1

Function_two:		; Function group two (F5-F12)
	btfss	DATA2,4			; Test for 5-8 or 9-12
	goto	Function_nine	; Bit is not set so do 9-12
							; Bit is set so drop through to 5-8
; Function 5-8
	swapf	CV21,W			; Get F5-F8 consist status into lower 4 bits
	andlw	b'00001111'		; Mask off leaving consist mask for just F5-F8
	btfss	CONFIG0,cst_adr	; Check for using consist address, skip if consist
	xorlw	b'00001111'		; Invert mask to get non consist mask
	movwf	FnMASK			; Save mask for F5-F8
	btfss	CONFIG0,cst_mode	; check if consist is currently active
	clrf	FnMASK			; consist not active so do all functions anyway
	movlw	F5_base				; Set CV base, F5=CV39,F6=CV40,F7=CV41,F8=CV42
	call	Function_block
	call	Function_activate
	goto	exit_bank1

; Function 9-12
Function_nine:
	swapf	CV22,W			; Get F9-F12 consist status in lower 4 bits
	andlw	b'00001111'		; Mask off leaving consist mask for just F9-F12
	btfss	CONFIG0,cst_adr	; Check for using consist address, skip if consist
	xorlw	b'00001111'		; Invert mask to get non consist mask
	movwf	FnMASK			; Save mask for F9 to F12
	btfss	CONFIG0,cst_mode	; check if consist is currently active
	clrf	FnMASK			; consist not active so do all functions anyway
	movlw	F9_base				; Set CV base, F9=CV43,F10=CV44,F11=CV45,F12=CV46
	call	Function_block
	call	Function_activate
	goto	exit_bank1

; Function group three (F13-F28)
; Command is in the form of:-
;			DATA2      DATA3
;			1101111A 0 FFFFFFFF
; Where A is 0 for F13-20 and 1 for F21-28, F is a function state bit
Function_three:
	clrf	FnMASK			; assume not consist address so do all functions
	btfsc	CONFIG0,cst_adr	; check if consist address
	decf	FnMASK,F		; consist so set to all 1 to disable all functions 

	movf	DATA2,W		; Get instruction
	andlw	b'11111110'	; mask off valid bits
	xorlw	b'11011110'	; Zero if a valid F13-F28 instruction
	btfss	STATUS,Z	; Is it F13-F28?
	goto	exit_bank1	; No, it is a currently unsupported bit command then
	btfsc	DATA2,0		; Is it F21-F28?
	goto	Function_21	; Yes, do F21-F28
						; No, do F13-F20
	movf	DATA3,W		; Need to move function bits into DATA2
	movwf	DATA2
	movlw	F13_base	; Get base CV for F13-F16
	call	Function_block
	swapf	DATA2,F		; Move the other four functions into the lower four bits
	movlw	F17_base	; Get base CV for F17-F20
	call	Function_block
	call	Function_activate
	goto	exit_bank1

Function_21:
	movf	DATA3,W		; Need to move function bits into DATA2
	movwf	DATA2
	movlw	F21_base	; Get base CV for F21-F24
	call	Function_block
	swapf	DATA2,F		; Move the other four functions into the lower four bits
	movlw	F25_base	; Get base CV for F25-F28
	call	Function_block	; 
	call	Function_activate
	goto	exit_bank1

; Function block subroutine. This is used for each block of four functions 1-4, 5-8 and 9-12.
; Input is just the CV base in W. The function control, is in the lower four bits of DATA2
;
;		DATA2
;		xxxxFFFF
;
; Where F is a function control bit. The lowest numbered function has the lowest bit value
;
; For functions 13-28 the data will have to be put into the same format as above in blocks
; of four functions from the format below and processed four bits at a time by swapping nibbles.
;
;		DATA3
;		FFFFFFFF


Function_block:
	movwf	TEMP		; Store CV base in TEMP

Do_Function:
	bcf	CONFIG0,CVaccessBit	; clear CVaccess flag

; Function 1,5,9,13,17,21,25
	btfsc	FnMASK,0	; Check that we should be doing this function
	goto	End_Fn1		; Set so skip
	movf	TEMP,W		; Get CV number ready to read
	call	EEPROM_READ	; Read map for Function 1
	iorwf	FnSTATE, F	; Set mapped output on
	btfsc	DATA2,0		; Jump to end and leave F1 set if function active
	goto	End_Fn1
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
End_Fn1
; Function 2,6,10,14,18,22,26
	incf	TEMP,F		; Move on to next CV
	btfsc	FnMASK,1	; Check that we should be doing this function
	goto	End_Fn2
	movf	TEMP,W		; Get CV number ready to read
	call	EEPROM_READ	; Read map for Function 2
	iorwf	FnSTATE, F	; Set mapped output on
	btfsc	DATA2,1		; Jump to end and leave F2 set if function active
	goto	End_Fn2
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
End_Fn2
; Function 3,7,11,15,19,23,27
	incf	TEMP,F		; Move on to next CV
	btfsc	FnMASK,2	; Check that we should be doing this function
	goto	End_Fn3
	movf	TEMP,W		; Get CV number ready to read
	call	EEPROM_READ	; Read map for Function 3
	iorwf	FnSTATE, F	; Set mapped output on
	btfsc	DATA2,2		; Jump to end and leave F3 set if function active
	goto	End_Fn3
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
End_Fn3
; Function 4,8,12,16,20,24,28
	incf	TEMP,W		; Move on to next CV and get CV number ready to read
	btfsc	FnMASK,3	; Check that we should be doing this function
	goto	End_Fn4
	call	EEPROM_READ	; Read map for Function 3
	iorwf	FnSTATE, F	; Set mapped output on
	btfsc	DATA2,3		; Jump to end and leave F4 set if function active
	goto	End_Fn4
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
End_Fn4
	return				; Job done

; DATA2 bit 4 contains the state of function 0, either from speed command (14 bit)
; or function command (28/128 bit)
; FLAG,rev_mode contains the direction data, 1=reverse

; In practice Function 0 is two seperate functions, one forward and one reverse.

Function_zero:
;Set up activation mask
	movf	CV22,W			; Get F0 consist status in bits 2 and 3
	andlw	b'00001100'		; Mask off leaving consist mask for just F0
	btfss	CONFIG0,cst_adr	; Check for using consist address, skip if consist
	xorlw	b'00001100'		; Invert mask to get non consist mask
	movwf	FnMASK			; Save mask for F0
	btfss	CONFIG0,cst_mode	; check if consist is currently active
	clrf	FnMASK			; consist not active so do all functions anyway

; Function 0 forwards
	btfsc	FnMASK,3	; Check that we should be doing this function
	goto	End_fwd		; Set so don't do
	movlw	F0_base		; Function 0 Forwards mapping CV
	call	EEPROM_READ	; Read map for Function 0 forwards
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
	btfsc	DATA2,4		; Jump to end if F0 inactive, leave function off
	btfsc	FLAG,rev_mode	; Jump to end if reverse selected , leave function off
	goto	End_fwd
	xorlw	0xFF		; Invert map again
	iorwf	FnSTATE, F	; Set mapped output on
End_fwd:
; Function 0 reverse
	btfsc	FnMASK,2	; Check that we should be doing this function
	goto	End_rev
	movlw	(F0_base + F0_revoffset)	; Function 0 Backwards mapping
	call	EEPROM_READ	; Read map for Function 0 backwards
	xorlw	0xFF		; Invert map
	andwf	FnSTATE, F	; Leave non mapped outputs and clear mapped
	btfsc	DATA2,4		; Jump to end if F0 inactive, leave function off
	btfss	FLAG,rev_mode	; Jump to end if forward selected , leave function off
	goto	End_rev
	xorlw	0xFF		; Invert map again
	iorwf	FnSTATE, F	; Set mapped output on
End_rev:
	return				; Done

Function_activate:
; FnSTATE now contains the the active state of outputs A to H.
; Invert according to CV
	movf	INVERTMAP,W	; Get invert map
	xorwf	FnSTATE, W	; Invert required bits of function state and leave in W
	movwf	PORTC		; Put outputs C-H on Port C, C=PC5,D=PC4,E=PC3,F=PC2,G=PC1,H=PC0
	movwf	TEMP		; Store state ready for shifting
	rrf		TEMP, F		; Shift outputs A-H down one (H into carry)
	rrf		TEMP, W		; Shift outputs A-H down another one (G into carry)
	movwf	GPIO		; stick outputs A-F on GPIO, A=GP5,B=GP4,C=GP2,E=GP1,F=GP0
	return

;***********************************************************************************************************************
				; Consist Group
; Decoder reset apears as a consist group instruction to the broadcast address
; I think that this bit may contain errors!

ConsistGroup:
	bcf	CONFIG0,CVaccessBit	; clear CVaccess flag
	movf	DATA2,W
	xorlw	.0			; Service mode decoder reset instruction 00000000 
	btfsc	STATUS,Z
	goto	soft_reset	;
	xorlw	.1			; service mode hard reset instruction 00000001
	btfsc	STATUS,Z
	goto	hard_reset	; Set CV29 to default and clear CV19.
; Check here for setting/reseting of CV29 bit 5 using SM inst. 0000101x
	andlw	B'11111110'
	xorlw	B'00001010'	; Service mode set advanced addressing instruction 0000101x
	btfsc	STATUS,Z
	goto	long_adr_set

; Start checking for consist instructions.
; 0001001x where x is the direction of travel relative to the consist (CV19 bit 7)
	movf	DATA2,W
	andlw	B'11111110'
	xorlw	B'00010010'
	btfsc	STATUS,Z
	goto	cst_ctrl_set
	goto	exit_bank1

hard_reset:
; Perform NMRA service mode Hard Reset
	clrf	MYEEDATA	; no consist
	movlw	.19			; get EEPROM location of CV19
	call	EEPROM_WRITE

	movlw	cv29default	; CV29	
	movwf	MYEEDATA
	movlw	.29		; get EEPROM location of CV29
	call	EEPROM_WRITE

; reset CV31 and CV32 here as well if implemented.

soft_reset:
; Service mode reset packet received, so prepare for service mode
	bsf	CONFIG0,resetflag	; set resetflag for SM mode detection
; Decoder will now respond to address 112-127 as service mode packets
; Make sure that all function outputs are inactive
	clrf	FnSTATE			; Turn all functions off
	call	Function_activate
	goto	ReadNMRA

; Set consist address and direction in CV19
cst_ctrl_set:
	bsf		DATA3,7
	btfss	DATA2,0
	bcf		DATA3,7
	movf	DATA3,W
	movwf	MYEEDATA
	movlw	.19
	call	EEPROM_WRITE
	goto	exit_bank1

; Set/reset bit in CV29 for long addressing 
long_adr_set:
	movlw	.29
	call	EEPROM_READ	; Get CV29 contents
	movwf	MYEEDATA	; Put in buffer
	bsf		MYEEDATA,5	; Set bit
	btfss	DATA2,0		; Check bit
	bcf		MYEEDATA,5	; Clear bit if it should not be set
	movlw	.29
	call	EEPROM_WRITE; Rewrite CV29
;	goto	exit_bank1	<- implied goto

;***********************************************************************************************************************
exit_bank1:
	clrf	TIMEOUTCOUNT	; ANALOG watchdog
	clrf	TEMP
	clrf	DATA1
	clrf	DATA2
	clrf	DATA3
	clrf	DATA4
	clrf	DATA5
	clrf	DATA6
	clrf	ENDVAL
	return
;***********************************************************************************************************************
; Speed subroutine replaces speed macros to save space

; Need to replace this with routine to manage exotic function output.
; Timing is critical, enters on step 5 and should exit with last step of return step 20
; which makes only 7 instructions per pass for correct timing. Stretching to 9 will match
; current routine which appears to work.
Speed_Sub
	incf	SAMPLES,F	;5
	bcf	CONFIG0,bitrec	;6
	movlw	0xFD		;7
	addwf	SAMPLES,W	;8
	btfss	STATUS,C	;9
	bsf	CONFIG0,bitrec	;10
	decfsz	COUNTER,F	;11
	Goto	Do_Flicker	;12,13 Update random number generator
	movf	FnSTATE,W	;13	Get function state
	btfss	RANDOM,7	;14	Test random bit
	andwf	FlickMASK,W	;15	Bit clear, so switch off flicker outputs
	xorwf	INVERTMAP,W	;16 Invert required bits of function state and leave in W
	movwf	PORTC		;17 Put outputs C-H on Port C, C=PC5,(D=PC4),E=PC3,F=PC2,G=PC1,H=PC0
	decfsz	TIMEOUTCOUNT,F	;18	For ANALOG mode. Timeoutcount is cleared in state machine.
	return				;19,20 timeout not expired, so carry on receiving bits
	goto	ANALOG		;20,21 Timeout expired so do analogue timing not critical <= implied goto

Do_Flicker:
; Randomisation routine
	movlw	01DH		;14 Load prime number
	clrc				;15 Clear carry ready for shift
	rlf		RANDOM,F	;16
	btfsc	STATUS,C	;17
	xorwf	RANDOM,F	;18
; Random bit supplied in RANDOM,7 
	return				;19,20


;************************************************************************************************************
; Analogue routine here. Came here because no valid DCC signal detected

; Need to look at DC behaviour from CV. Normal to support Brake On DC
; or analogue operation but not both

ANALOG:
	btfss	CV29,DCmode	; DC analogue or brake on DC supported?
	return				; Clear, so carry on as normal and wait for a valid packet
						; Set, analogue enabled so must be running on a DC layout.

; Need to turn Functions 1-12 on according to CV13/CV14 values
; and operate F0 according to DC polarity if true analogue mode.

	movlw	.13		; CV13 will contain analogue group 1 active functions 1-8
	call	EEPROM_READ
	movwf	DATA2		; Hold analogue function status in DATA2
	movlw	F1_base		; Start with CV35 for F1 mapping
	call	Function_block
	swapf	DATA2,F		; Move the other four functions into the lower four bits
	movlw	F5_base 	; Get base CV for F5-8
	call	Function_block
	movlw	.14	; CV14 will contain analogue group 2 active functions 9-12 and F0
	call	EEPROM_READ
	movwf	CV14		; Store CV14 for later
	movwf	DATA2		; Put analogue function status in DATA2
	rrf		DATA2,W		; Shift F9-F12 into lower four bits
	rrf		DATA2,W		; 
	movlw	F9_base		; Get base CV for F9-12
	call	Function_block
	call	Function_activate
; Do F0 here according to track polarity. Track polarity can only be accuratly
; measured in comparator mode so no Function 0 support for PA3 mode.

Alog_Chg:
	bsf		FLAG,rev_mode	; Set forwards
	btfss	CMCON,COUT	; Are we going forwards on DC?
	bcf		FLAG,rev_mode	; No, set reverse
	bcf		DATA2,4			; Set function 0 off
	btfsc	FLAG,rev_mode	; Going forwards?
	goto 	Alog_Rev		; No, so do reverse
	btfsc	CV14,0			; F0 forwards enabled on DC?
	bsf		DATA2,4			; Yes, set function 0 on
	goto	Alog_Act
Alog_Rev:
	btfsc	CV14,1			; F0 reverse enabled on DC?
	bsf		DATA2,4			; Yes, set function 0 on
Alog_Act:
	call	Function_zero	; Do F0 forwards and F0 backwards
	call	Function_activate


DC_Analogue:
; Put support for exotic functions on analogue in here
	clrw					; Set zero flag
	btfsc	CMCON,COUT	; Test the comparator output
	xorlw	0xff			; Clear zero flag
	btfsc	FLAG,rev_mode	; Are we going in reverse?
	xorlw	0xff			; Toggle zero flag
	btfsc	STATUS,Z		; Test for 0 (0=same)
	goto 	DC_Analogue		; Same so test again
	goto 	Alog_Chg		; Different so change direction





	org	0x2108	; Make sure that CV8=255 to force load of EEPROM to defaults on first boot
	DW	.255	; CV8=255

	END
