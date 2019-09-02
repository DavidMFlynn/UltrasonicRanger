;====================================================================================================
;
;    Filename:      UltrasonicRanger.asm
;    Created:       8/4/2019
;    File Version:  1.1d2   8/30/2019
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    UltrasonicRanger is sample code.
;    Features and configurations will be added as needed.
;
;
;    Features: 	TTL Packet Serial
;	Interfaces for:
;	  4 Ultrasonic Distance Sensors
;	  System LED
;
;
;    History:
; 1.1d2   8/30/2019	Added Wait State between sensor triggers
; 1.1d1   8/4/2019	Copied from TetheredJoy.
; 1.1a1   6/10/2019 	Working. Time to do some tests.
;
;====================================================================================================
; ToDo:
;
;
;====================================================================================================
;====================================================================================================
; What happens next:
;   At power up the system LED will blink.
;====================================================================================================
;
;   Pin 1 (RA2/AN2) Sensor 3 Trigger Pulse Output
;   Pin 2 (RA3/AN3) CCP3 Sensor 3 Echo Pulse Input
;   Pin 3 (RA4/AN4) CCP4 Sensor 4 Echo Pulse Input
;   Pin 4 (RA5/MCLR*) VPP/MCLR*
;   Pin 5 (GND) Ground
;   Pin 6 (RB0) LED1/SW1 (Active Low Input/Output)(System LED)
;   Pin 7 (RB1/AN11/SDA1) not used
;   Pin 8 (RB2/AN10/TX) TTL Serial RX
;   Pin 9 (RB3/CCP1) CCP1 Sensor 1 Echo Pulse Input
;
;   Pin 10 (RB4/AN8/SLC1) LED2/SW2 (Active Low Input/Output)
;   Pin 11 (RB5/AN7) TTL Serial TX
;   Pin 12 (RB6/AN5/CCP2) ICSPCLK
;   Pin 13 (RB7/AN6) ICSPDAT
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) Sensor 4 Trigger Pulse Output
;   Pin 16 (RA7/CCP2) CCP2 Sensor 2 Echo Pulse Input
;   Pin 17 (RA0/AN0) Sensor 1 Trigger Pulse Output
;   Pin 18 (RA1/AN1) Sensor 2 Trigger Pulse Output
;
;====================================================================================================
;
;
	list	p=16f1847,r=hex,W=1	; list directive to define processor
	nolist
	include	p16f1847.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_ON & _LVP_OFF
;
; Write protection off
; 4x PLL Enabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming enabled
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
	constant	useRS232=1
	constant	UseEEParams=1
;
	constant	RP_LongAddr=0
	constant	RP_AddressBytes=1
	constant	RP_DataBytes=4
	constant	UseRS232SyncBytes=1
kRS232SyncByteValue	EQU	0xDD
	constant	UseRS232Chksum=1
;
kRS232_MasterAddr	EQU	0x01	;Master's Address
kRS232_SlaveAddr	EQU	0x22	;This Slave's Address
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
;====================================================================================================
	nolist
	include	F1847_Macros.inc
	list
;
;
;    Port A bits
PortADDRBits	EQU	b'10111000'
PortAValue	EQU	b'00000000'
ANSELA_Val	EQU	b'00000000'	;RA0/AN0, RA1/AN1
;
#Define	Sensor1Trig	LATA,0	;Sensor 1 Trigger Pulse Output
#Define	Sensor2Trig	LATA,1	;Sensor 2 Trigger Pulse Output
#Define	Sensor3Trig	LATA,2	;Sensor 3 Trigger Pulse Output
#Define	Sensor3Echo	PORTA,3	;CCP3 Sensor 3 Echo Pulse Input
#Define	Sensor4Echo	PORTA,4	;CCP4 Sensor 4 Echo Pulse Input
#Define	RA5_In	PORTA,5	;VPP/MCLR*
#Define	Sensor4Trig	LATA,6	;Sensor 4 Trigger Pulse Output
#Define	Sensor2Echo	PORTA,7	;CCP2 Sensor 2 Echo Pulse Input
;
;
;    Port B bits
PortBDDRBits	EQU	b'11011111'	;MagEnc_CSBit, CCP1, MagEnc_CLKBit
PortBValue	EQU	b'00000000'
ANSELB_Val	EQU	b'00000000'	;RB5/AN7
;
#Define	RB0_Out	LATB,0	;LED1/SW1 (Active Low Input/Output)(System LED)
#Define	SW1_In	PORTB,0
#Define	RB1_In	PORTB,1	; not used
#Define	RB2_In	PORTB,2	;RX Serial Data
#Define	Sensor1Echo	PORTB,3	;CCP1 Sensor 1 Echo Pulse Input
#Define	SW2_In	PORTB,4	;LED2/SW2 (Active Low Input/Output)
#Define	RB5_In	PORTB,5	;TX Serial Data
#Define	RB6_In	PORTB,6	;ICSPCLK
#Define	RB7_In	PORTB,7	;ICSPDAT
;
SysLED_Bit	EQU	0	;LED1/SW1 (Active Low Input/Output)
LED2_Bit	EQU	4	;LED2/SW2 (Active Low Input/Output)
;
#Define	SysLED_Tris	TRISB,SysLED_Bit	;LED1 (Active Low Output)
#Define	LED1_Tris	TRISB,LED1_Bit	;LED1 (Active Low Output)
#Define	LED1_Lat	LATB,LED1_Bit	;LED1 (Active Low Output)
;
#Define	LED2_Tris	TRISB,LED2_Bit	;LED4 (Active Low Output)
#Define	LED2_Lat	LATB,LED2_Bit	;LED4 (Active Low Output)
;
;========================================================================================
;========================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
;OSCCON_Value	EQU	b'01110010'	; 8 MHz
OSCCON_Value	EQU	b'11110000'	;32MHz
;
;T2CON_Value	EQU	b'01001110'	;T2 On, /16 pre, /10 post
T2CON_Value	EQU	b'01001111'	;T2 On, /64 pre, /10 post
PR2_Value	EQU	.125
;
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
LEDFastTime	EQU	d'20'
;
;T1CON_Val	EQU	b'00000001'	;Fosc=8MHz, PreScale=1,Fosc/4,Timer ON
;T1CON_Val	EQU	b'00100001'	;Fosc=32MHz, PreScale=4,Fosc/4,Timer ON
T1CON_Val	EQU	b'00110001'	;32MHz, FOSC/4 1:8 Prescale, Enabled = 1 MHz
CCPxCON_Rising_Val	EQU	b'00000101'
CCPxCON_Falling_Val	EQU	b'00000100'
;
;TXSTA_Value	EQU	b'00100000'	;8 bit, TX enabled, Async, low speed
TXSTA_Value	EQU	b'00100100'	;8 bit, TX enabled, Async, high speed
RCSTA_Value	EQU	b'10010000'	;RX enabled, 8 bit, Continious receive
BAUDCON_Value	EQU	b'00001000'	;BRG16=1
; 8MHz clock low speed (BRGH=0,BRG16=1)
;Baud_300	EQU	d'1666'	;0.299, -0.02%
;Baud_1200	EQU	d'416'	;1.199, -0.08%
;Baud_2400	EQU	d'207'	;2.404, +0.16%
;Baud_9600	EQU	d'51'	;9.615, +0.16%
; 32MHz clock low speed (BRGH=1,BRG16=1)
Baud_300	EQU	.26666	;300, 0.00%
Baud_1200	EQU	.6666	;1200, 0.00%
Baud_2400	EQU	.3332	;2400, +0.01%
Baud_9600	EQU	.832	;9604, +0.04%
Baud_19200	EQU	.416	;19.18k, -0.08%
Baud_38400	EQU	.207	;38.46k, +0.16%
Baud_57600	EQU	.138	;57.55k, -0.08%
BaudRate	EQU	Baud_38400
;
kSysMode	EQU	.0	;Default Mode
kSysFlags	EQU	.0
;
DebounceTime	EQU	.10
kMaxMode	EQU	.0
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 256 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xEF, Bank2 0x120..0x16F
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	SysLED_Time		;sys LED time
	SysLED_Blinks		;0=1 flash,1,2,3
	SysLED_BlinkCount
	SysLEDCount		;sys LED Timer tick count
;
	LED2_Blinks		;0=off,1,2,3
	LED2_BlinkCount		;LED1_Blinks..0
	LED2_Count		;tick count
;
	SWFlags
;
	RangerFlags
	RangerFlags2
	Range1
	Range2
	Range3
	Range4
	RangeScratch:2		;Used by ISR
;
	Capture1StartTime:2
	Capture2StartTime:2
	Capture3StartTime:2
	Capture4StartTime:2
	Capture_A:2
	CaptureState
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; 50 mS RX timeiout
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
	TXByte		;Next byte to send
	RXByte		;Last byte received
	SerFlags
;
;-----------------------
;Below here are saved in eprom
	SysMode
	RS232_MasterAddr
	RS232_SlaveAddr
	SysFlags
;
	endc
;
;--------------------------------------------------------------
;---RangerFlags---
#Define	TriggerFlag	RangerFlags,0
#Define	Sensor1Active	RangerFlags,1
#Define	Sensor2Active	RangerFlags,2
#Define	Sensor3Active	RangerFlags,3
#Define	Sensor4Active	RangerFlags,4
#Define	AutoTrigger	RangerFlags,5
;
#Define	Timing1Flag	RangerFlags2,0
#Define	Timing2Flag	RangerFlags2,1
#Define	Timing3Flag	RangerFlags2,2
#Define	Timing4Flag	RangerFlags2,3
#Define	Timing1Go	RangerFlags2,4
#Define	Timing2Go	RangerFlags2,5
#Define	Timing3Go	RangerFlags2,6
#Define	Timing4Go	RangerFlags2,7
;
;---SWFlags bits---
#Define	SW1_Flag	SWFlags,0
#Define	SW2_Flag	SWFlags,1
;
;---SerFlags bits---
#Define	DataReceivedFlag	SerFlags,1
#Define	DataSentFlag	SerFlags,2
;
;---------------
#Define	FirstRAMParam	SysMode
#Define	LastRAMParam	SysFlags
;
#Define	ssRX_Timeout	SysFlags,0
;
;
;
;================================================================================================
;  Bank1 Ram 0A0h-0EFh 80 Bytes
	cblock	0x0A0
	RX_ParseFlags
	RX_Flags
	RX_DataCount
	RX_CSUM
	RX_SrcAdd:RP_AddressBytes
	RX_DstAdd:RP_AddressBytes
	RX_TempData:RP_DataBytes
	RX_Data:RP_DataBytes
	TX_Data:RP_DataBytes
;
	endc
;
;
;================================================================================================
;  Bank2 Ram 120h-16Fh 80 Bytes
;
#Define	Ser_Buff_Bank	2
;
	cblock	0x120
	Ser_In_Bytes		;Bytes in Ser_In_Buff
	Ser_Out_Bytes		;Bytes in Ser_Out_Buff
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Buff:20
	Ser_Out_Buff:20
	endc
;
;================================================================================================
;  Bank3 Ram 1A0h-1EFh 80 Bytes
;=========================================================================================
;  Bank5 Ram 2A0h-2EFh 80 Bytes
;
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	Param70
	Param71
	Param72
	Param73
	Param74
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
;=========================================================================================
;Conditions
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10d1
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;
; default values
	ORG	0xF000
	de	kSysMode	;nvSysMode
	de	kRS232_MasterAddr	;nvRS232_MasterAddr, 0x0F
	de	kRS232_SlaveAddr	;nvRS232_SlaveAddr, 0x10
	de	kSysFlags	;nvSysFlags
;
	ORG	0xF0FF
	de	0x00	;Skip BootLoader
;
	cblock	0x0000
;
	nvSysMode
	nvRS232_MasterAddr
	nvRS232_SlaveAddr
	nvSysFlags
;
	endc
;
#Define	nvFirstParamByte	nvSysMode
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
BootLoaderStart	EQU	0x1E00
;
	ORG	0x000	; processor reset vector
	movlp	high BootLoaderStart
	goto	BootLoaderStart
;
ProgStartVector	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.01 seconds
;
;
	ORG	0x004	; interrupt vector location
	CLRF	PCLATH
	CLRF	BSR	; bank0
;
;
	BTFSS	PIR1,TMR2IF
	goto	SystemTick_end
;
	BCF	PIR1,TMR2IF	; reset interupt flag bit
;------------------
; These routines run 100 times per second
;
;------------------
;Decrement timers until they are zero
;
	call	DecTimer1	;if timer 1 is not zero decrement
	call	DecTimer2
	call	DecTimer3
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
;
; All LEDs off
	movlb	0x01	;bank 1
	bsf	SysLED_Tris
;
	BSF	LED2_Tris
;
; Read Switches
	movlb	0x00	;bank 0
	BCF	SW1_Flag
	BTFSS	SW1_In
	BSF	SW1_Flag
	BTFSS	SW1_In
;
	BCF	SW2_Flag
	BTFSS	SW2_In
	BSF	SW2_Flag
	BTFSS	SW2_In
;
;
	movlb	0x00	;bank 0	
;--------------------
; Sys LED time
	DECFSZ	SysLEDCount,F	;Is it time?
	bra	SystemBlink_end	; No, not yet
;
	movf	SysLED_Blinks,F
	SKPNZ		;Standard Blinking?
	bra	SystemBlink_Std	; Yes
;
; custom blinking
;
SystemBlink_Std	CLRF	SysLED_BlinkCount
	MOVF	SysLED_Time,W
SystemBlink_DoIt	MOVWF	SysLEDCount
	movlb	0x01	;bank 1
	bcf	SysLED_Tris	;LED ON
SystemBlink_end:
;--------------------
	movlb	0x00	;bank 0
;-------------
;
SystemTick_end:
;--------------------
; Handle Captures
	movlb	0	;bank 0
	btfss	PIR1,CCP1IF
	bra	Capture1_end
;
	movlw	low CCPR1L
	movwf	FSR0L
	movlw	high CCPR1L
	movwf	FSR0H
	movlw	low Capture1StartTime
	movwf	FSR1L
	movlw	high Capture1StartTime
	movwf	FSR1H
;	
	btfsc	Timing1Flag	;Timing?
	bra	Capture1_Calc	; Yes
;
	bsf	Timing1Flag	;Capture start time
	moviw	FSR0++
	movwi	FSR1++
	moviw	FSR0++
	movwi	FSR1++
	movlb	5	;bank 5
	movlw	CCPxCON_Falling_Val
	movwf	CCP1CON
	bra	Capture1_Done
;
; Capture_A = CCPRn - Capture1StartTime
; Range1 = Capture_A / 32
Capture1_Calc	bcf	Timing1Flag
	movf	Range1,W
	movwf	RangeScratch
	call	Capture_A_Calc
	movwf	Range1
	bcf	Timing1Go
;
	movlb	5	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP1CON	
;	
Capture1_Done	movlb	0	;bank 0
	bcf	PIR1,CCP1IF
Capture1_end:
;--------------
	btfss	PIR2,CCP2IF
	bra	Capture2_end
;
	movlw	low CCPR2L
	movwf	FSR0L
	movlw	high CCPR2L
	movwf	FSR0H
	movlw	low Capture2StartTime
	movwf	FSR1L
	movlw	high Capture2StartTime
	movwf	FSR1H
;
	btfsc	Timing2Flag
	bra	Capture2_Calc
;
	bsf	Timing2Flag
	moviw	FSR0++
	movwi	FSR1++
	moviw	FSR0++
	movwi	FSR1++
	movlb	5	;bank 5
	movlw	CCPxCON_Falling_Val
	movwf	CCP2CON
	bra	Capture2_Done
;
; Capture_A = CCPRn - Capture2StartTime
; Range2 = Capture_A / 32
Capture2_Calc	bcf	Timing2Flag
	movf	Range2,W
	movwf	RangeScratch
	call	Capture_A_Calc
	movwf	Range2
	bcf	Timing2Go
;
	movlb	5	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP2CON	
;	
Capture2_Done	movlb	0	;bank 0
	bcf	PIR2,CCP2IF
Capture2_end:
;--------------
	btfss	PIR3,CCP3IF
	bra	Capture3_end
;
	movlw	low CCPR3L
	movwf	FSR0L
	movlw	high CCPR3L
	movwf	FSR0H
	movlw	low Capture3StartTime
	movwf	FSR1L
	movlw	high Capture3StartTime
	movwf	FSR1H
;	
	btfsc	Timing3Flag
	bra	Capture3_Calc
;
	bsf	Timing3Flag
	moviw	FSR0++
	movwi	FSR1++
	moviw	FSR0++
	movwi	FSR1++
	movlb	6	;bank 6
	movlw	CCPxCON_Falling_Val
	movwf	CCP3CON
	bra	Capture3_Done
;
; Capture_A = CCPRn - Capture3StartTime
; Range3 = Capture_A / 32
Capture3_Calc	bcf	Timing3Flag
	movf	Range3,W
	movwf	RangeScratch
	call	Capture_A_Calc
	movwf	Range3
	bcf	Timing3Go
;
	movlb	6	;bank 6
	movlw	CCPxCON_Rising_Val
	movwf	CCP3CON	
;	
Capture3_Done	movlb	0	;bank 0
	bcf	PIR3,CCP3IF
Capture3_end:
;---------------
	btfss	PIR3,CCP4IF
	bra	Capture4_end
;
	movlw	low CCPR4L
	movwf	FSR0L
	movlw	high CCPR4L
	movwf	FSR0H
	movlw	low Capture4StartTime
	movwf	FSR1L
	movlw	high Capture4StartTime
	movwf	FSR1H
;	
	btfsc	Timing4Flag
	bra	Capture4_Calc
;
	bsf	Timing4Flag
	moviw	FSR0++
	movwi	FSR1++
	moviw	FSR0++
	movwi	FSR1++
	movlb	6	;bank 6
	movlw	CCPxCON_Falling_Val
	movwf	CCP4CON
	bra	Capture4_Done
;
; Capture_A = CCPRn - Capture4StartTime
; Range4 = Capture_A / 32
Capture4_Calc	bcf	Timing4Flag
	movf	Range4,W
	movwf	RangeScratch
	call	Capture_A_Calc
	movwf	Range4
	bcf	Timing4Go
;
	movlb	6	;bank 6
	movlw	CCPxCON_Rising_Val
	movwf	CCP4CON	
;	
Capture4_Done	movlb	0	;bank 0
	bcf	PIR3,CCP4IF
Capture4_end:
;
;==================================================================================
;AUSART Serial ISR
;
IRQ_Ser	BTFSS	PIR1,RCIF	;RX has a byte?
	BRA	IRQ_Ser_End
	CALL	RX_TheByte
;
IRQ_Ser_End:
;-----------------------------------------------------------------------------------------
	retfie		; return from interrupt
;
Capture_A_Calc	moviw	FSR0++	;CCPRxL
	movwf	Capture_A
	moviw	FSR0++	;CCPRxH
	movwf	Capture_A+1
;
	moviw	FSR1++	;CapturexStartTime
	subwf	Capture_A,F
	moviw	FSR1++	;CapturexStartTime+1
	subwfb	Capture_A+1,F
;
	clrf	RangeScratch+1
;
	lsrf	Capture_A+1,F
	rrf	Capture_A,F
	lsrf	Capture_A+1,F
	rrf	Capture_A,F
	lsrf	Capture_A+1,F
	rrf	Capture_A,F
	lsrf	Capture_A+1,F
	rrf	Capture_A,F
	lsrf	Capture_A+1,F
	rrf	Capture_A,F
	movf	Capture_A+1,F
	SKPZ		;Is Capture_A/32 > 255?
	bra	Capture_A_Calc_Err	; Yes, error
;
	movf	Capture_A,W
	subwf	RangeScratch,W	;W=RangeScratch-Capture_A
	SKPNB		;RangeScratch>=Capture_A?
	bra	Capture_A_Calc_Av	; No
	movf	Capture_A,W
	return
;
Capture_A_Calc_Av	movf	Capture_A,W
	bra	Capture_A_Calc_Av2
;
Capture_A_Calc_Err	movlw	0xFF
Capture_A_Calc_Av2	addwf	RangeScratch,F
	movlw	0x00
	addwfc	RangeScratch+1,F
	lsrf	RangeScratch+1,F
	rrf	RangeScratch,W
	return
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	include <F1847_Common.inc>
	include <SerBuff1938.inc>
	include <RS232_Parse.inc>
;
;=========================================================================================
;
start	call	InitializeIO
;
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
MainLoop	CLRWDT
;
	call	GetSerInBytes
	SKPZ		;Any data?
	CALL	RS232_Parse	; yes
;
	movlb	1
	btfss	RXDataIsNew
	bra	ML_1
	mCall0To1	HandleRXData
ML_1:
;
	movlb	0	;Bank 0
;
;---------------------
; Handle Serial Communications
	BTFSC	PIR1,TXIF	;TX done?
	CALL	TX_TheByte	; Yes
;
; move any serial data received into the 32 byte input buffer
	BTFSS	DataReceivedFlag
	BRA	ML_Ser_Out
	MOVF	RXByte,W
	BCF	DataReceivedFlag
	CALL	StoreSerIn
;
; If the serial data has been sent and there are bytes in the buffer, send the next byte
;
ML_Ser_Out	BTFSS	DataSentFlag
	BRA	ML_Ser_End
	CALL	GetSerOut
	BTFSS	Param78,0
	BRA	ML_Ser_End
	MOVWF	TXByte
	BCF	DataSentFlag
ML_Ser_End:
;----------------------
; Do the captures
;
kScanStateIdle	EQU	0	; State 0, Idle
kScanStateTrigger1	EQU	1	; State 1, Trigger Sensor 1
kScanStateTiming1	EQU	2	; State 2, Timing Sensor 1
kScanStateWait1	EQU	3	; Wait for 1 to go quiet
kScanStateTrigger2	EQU	4	; State 3, Trigger Sensor 2
kScanStateTiming2	EQU	5	; State 4, Timing Sensor 2
kScanStateWait2	EQU	6	; Wait for 1 to go quiet
kScanStateTrigger3	EQU	7	; State 5, Trigger Sensor 3
kScanStateTiming3	EQU	8	; State 6, Timing Sensor 3
kScanStateWait3	EQU	9	; Wait for 1 to go quiet
kScanStateTrigger4	EQU	10	; State 7, Trigger Sensor 4
kScanStateTiming4	EQU	11	; State 9, Timing Sensor 4
kScanStateWait4	EQU	12	; Wait for 1 to go quiet
;
;
MaxAquisitionTime	EQU	.20	;0.2 Seconds
DwellStateTime	EQU	.10	;0.1 Seconds
;
C_State0	movlb	0	;Bank 0
	movf	CaptureState,F
	SKPZ
	bra	C_State1
;
	btfsc	TriggerFlag	;Triggered?
	bra	C_State0_Trig	; Yes
;
	btfss	AutoTrigger	;Auto-Trigger?
	bra	C_State_end	; No
	
C_State0_Trig	bcf	TriggerFlag
	bra	C_State_Next
	
	
;-------
C_State1	movlw	kScanStateTrigger1
	subwf	CaptureState,W
	SKPZ
	bra	C_State2
;
	btfss	Sensor1Active
	bra	C_State_Next
;
	bcf	Timing1Flag
	bsf	Timing1Go
; paranoid config of CCP
	movlb	1	;Bnak 1
	bcf	PIE1,CCP1IE	;Disable interrupt
	movlb	5	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP1CON
	movlb	0	;Bank 0
	bcf	PIR1,CCP1IF	;clear just in case
	movlb	1	;Bnak 1
	bsf	PIE1,CCP1IE	;Enable interrupt
;
	movlb	2	;Bank 2
	bsf	Sensor1Trig
	call	Delay10uS
	bcf	Sensor1Trig
	movlb	0	;Bank 0
	movlw	MaxAquisitionTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State2	movlw	kScanStateTiming1
	subwf	CaptureState,W
	SKPZ
	bra	C_State2A
;
	btfss	Sensor1Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State2_TO
;
	btfsc	Timing1Go
	bra	C_State_end
	movlw	DwellStateTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State2_TO	bcf	Timing1Flag	;cancel
	CLRF	Range1	; no echo
	bra	C_State_Next
;	
C_State2A	movlw	kScanStateWait1
	subwf	CaptureState,W
	SKPZ
	bra	C_State3
;
	btfss	Sensor1Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State_Next
	bra	C_State_end
;
;-------
C_State3	movlw	kScanStateTrigger2
	subwf	CaptureState,W
	SKPZ
	bra	C_State4
;
	btfss	Sensor2Active
	bra	C_State_Next
;
	bcf	Timing2Flag
	bsf	Timing2Go
;
; paranoid config of CCP
	movlb	1	;Bnak 1
	bcf	PIE2,CCP2IE	;Disable interrupt
	movlb	5	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP2CON
	movlb	0	;Bank 0
	bcf	PIR2,CCP2IF	;clear just in case
	movlb	1	;Bnak 1
	bsf	PIE2,CCP2IE	;Enable interrupt
;
	movlb	2	;Bank 2
	bsf	Sensor2Trig
	call	Delay10uS
	bcf	Sensor2Trig
	movlb	0	;Bank 0
	movlw	MaxAquisitionTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State4	movlw	kScanStateTiming2
	subwf	CaptureState,W
	SKPZ
	bra	C_State4A
;
	btfss	Sensor2Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State4_TO
;
	btfsc	Timing2Go
	bra	C_State_end
	movlw	DwellStateTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State4_TO	bcf	Timing2Go	;cancel
	clrf	Range2	; no echo
	bra	C_State_Next
;
;	
C_State4A	movlw	kScanStateWait2
	subwf	CaptureState,W
	SKPZ
	bra	C_State5
;
	btfss	Sensor2Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State_Next
	bra	C_State_end
;
;-------------
C_State5	movlw	kScanStateTrigger3
	subwf	CaptureState,W
	SKPZ
	bra	C_State6
;
	btfss	Sensor3Active
	bra	C_State_Next
;
	bcf	Timing3Flag
	bsf	Timing3Go
; paranoid config of CCP
	movlb	1	;Bnak 1
	bcf	PIE3,CCP3IE	;Disable interrupt
	movlb	6	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP3CON
	movlb	0	;Bank 0
	bcf	PIR3,CCP3IF	;clear just in case
	movlb	1	;Bnak 1
	bsf	PIE3,CCP3IE	;Enable interrupt
;
	movlb	2	;Bank 2
	bsf	Sensor3Trig
	call	Delay10uS
	bcf	Sensor3Trig
	movlb	0	;Bank 0
	movlw	MaxAquisitionTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State6	movlw	kScanStateTiming3
	subwf	CaptureState,W
	SKPZ
	bra	C_State6A
;
	btfss	Sensor3Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State6_TO
;
	btfsc	Timing3Go
	bra	C_State_end
	movlw	DwellStateTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State6_TO	bcf	Timing3Flag	;cancel
	clrf	Range3	; no echo
	bra	C_State_Next
;
;	
C_State6A	movlw	kScanStateWait3
	subwf	CaptureState,W
	SKPZ
	bra	C_State7
;
	btfss	Sensor3Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State_Next
	bra	C_State_end
;
;-----------
C_State7	movlw	kScanStateTrigger4
	subwf	CaptureState,W
	SKPZ
	bra	C_State8
;
	btfss	Sensor4Active
	bra	C_State_Next
;
	bcf	Timing4Flag
	bsf	Timing4Go
; paranoid config of CCP
	movlb	1	;Bnak 1
	bcf	PIE3,CCP4IE	;Disable interrupt
	movlb	6	;bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP4CON
	movlb	0	;Bank 0
	bcf	PIR3,CCP4IF	;clear just in case
	movlb	1	;Bnak 1
	bsf	PIE3,CCP4IE	;Enable interrupt
;
	movlb	2	;Bank 2
	bsf	Sensor4Trig
	call	Delay10uS
	bcf	Sensor4Trig
	movlb	0	;Bank 0
	movlw	MaxAquisitionTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State8	movlw	kScanStateTiming4
	subwf	CaptureState,W
	SKPZ
	bra	C_State8A
;
	btfss	Sensor4Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State8_TO
;
	btfsc	Timing4Go
	bra	C_State_end
	movlw	DwellStateTime
	movwf	Timer3Lo
	bra	C_State_Next
;
C_State8_TO	bcf	Timing4Flag	;cancel
	CLRF	Range4	; no echo
	bra	C_State_Next
;
;	
C_State8A	movlw	kScanStateWait4
	subwf	CaptureState,W
	SKPZ
	bra	C_State9
;
	btfss	Sensor4Active
	bra	C_State_Next
;
	movf	Timer3Lo,F
	SKPNZ
	bra	C_State_Next
	bra	C_State_end
;
;-------------
C_State9	clrf	CaptureState
	bra	C_State_end
;
C_State_Next:
	movlb	0	;Bank 0
	incf	CaptureState,F
C_State_end:
;
	goto	MainLoop
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;=========================================================================================
;=========================================================================================
; call once
;=========================================================================================
;
InitializeIO	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLW	OSCCON_Value
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON
;
	movlb	4	; bank 4
	bsf	WPUA,WPUA5	;Put a pull up on the MCLR unused pin.
;
	movlb	3	; bank 3
	movlw	ANSELA_Val
	movwf	ANSELA
	movlw	ANSELB_Val
	movwf	ANSELB
;
;SPI MISO >> SDI1 RB1, default
;SPI CLK >> RB4, default
	movlb	2	;bank 2
	bsf	APFCON0,RXDTSEL	;RX >> RB2
	bsf	APFCON0,SDO1SEL	;SPI MOSI >> SDO1 RA6
;
	bsf	APFCON0,CCP2SEL	;CCP2 >> RA7
;
	bsf	APFCON1,TXCKSEL	;TX >> RB5
;
;
;Setup T2 for 100/s
	movlb	0	; bank 0
	MOVLW	T2CON_Value
	MOVWF	T2CON
	MOVLW	PR2_Value
	MOVWF	PR2
	movlb	1	; bank 1
	bsf	PIE1,TMR2IE	; enable Timer 2 interupt
;
;	
; clear memory to zero
	CALL	ClearRam
	CLRWDT
	CALL	CopyToRam
;
;=======================
; Setup all 4 CCPs for capture
	movlb	0	;Bank 0
	movlw	T1CON_Val
	movwf	T1CON
;
	movlb	5	;Bank 5
	movlw	CCPxCON_Rising_Val
	movwf	CCP1CON
	movwf	CCP2CON
;
	movlb	6	;Bank 6
	movlw	CCPxCON_Rising_Val
	movwf	CCP3CON
	movwf	CCP4CON
;
	movlb	1	;Bank 1
	bsf	PIE1,CCP1IE
	bsf	PIE2,CCP2IE
	bsf	PIE3,CCP3IE
	bsf	PIE3,CCP4IE
;
	MOVLB	0x00	;Bank 0
; setup data ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA
	MOVLB	0x01	; bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
;
	if useRS232
; setup serial I/O
	BANKSEL	BAUDCON	; bank 3
	movlw	BAUDCON_Value
	movwf	BAUDCON
	MOVLW	low BaudRate
	MOVWF	SPBRGL
	MOVLW	high BaudRate
	MOVWF	SPBRGH
	MOVLW	TXSTA_Value
	MOVWF	TXSTA
	MOVLW	RCSTA_Value
	MOVWF	RCSTA
	movlb	0x01	; bank 1
	BSF	PIE1,RCIE	; Serial Receive interupt
	movlb	0x00	; bank 0
;
	endif
;
	CLRWDT
;-----------------------
;
	MOVLB	0x00
	MOVLW	LEDTIME
	MOVWF	SysLED_Time
	movlw	0x01
	movwf	SysLEDCount	;start blinking right away
;
;
	CLRWDT
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,GIE	; enable interupts
;
	return
;
;=========================================================================================
;=========================================================================================
;
;
	org 0x800
	include <SerialCmds.inc>
;
	org BootLoaderStart
	include <BootLoader.inc>
;
;
	END
;
