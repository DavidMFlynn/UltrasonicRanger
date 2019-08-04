;=========================================================================================
;
;    Filename:      SerialServoCmds.inc
;    Date:          6/1/2018
;    File Version:  1.1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
; Notes:
;  Command interpreter for SerialServo
;
; History
; 1.1    6/1/2018	All commands are here now.
; 1.0    5/25/2018	Moved out of SerialServo.asm
;=========================================================================================
;
kCmd_SetMode	EQU	0x81	;+1 data (SysMode), return ACK
kCmd_GetMode	EQU	0x01
kCmd_GetRange	EQU	0x82	;+1 data Return Sensor(n)
kCmd_Trigger	EQU	0x83	;return ACK
;
;
kCmd_SaveParams	EQU	0x94	;Save all eeprom params, return ACK
kCmd_RestoreParams	EQU	0x95	;Copy to ram, return ACK
kCmd_ReBoot	EQU	0x99	;ReBoot the controller
kCmd_RunBootloader	EQU	0x9A	;Set EEPROM (0xFF) = 0xFF and reboot
;
; these commands save params and return a 0,0,0,0 packet with the new address
kCmd_SetMasterAddr	EQU	0xA1	;+1 data, return a zero packet
kCmd_SetSlaveAddr	EQU	0xA2	;+1 data, return a zero packet
;
;Special packet (0xDD, 0xDD, 0x01, 0x02, 0xAA, 0x00, 0x00, 0x00, checksum)
kCmd_SendIDString	EQU	0xAA	; returns "DMFE Ranger\n"
;
;=========================================================================================
;
RS232_Send_S0	mCall1To0	RS232_Send
	return
;
;=========================================================================================
; Entry: RXDataIsNew=1, data in RX_Data
;
HandleRXData	movlb	1	;bank 1
	bcf	RXDataIsNew
	btfsc	RXDataValidFlag	;from master to me?
	bra	HandleRXData_1
HandleRXData_Rtn	movlb	0	;bank 0
	return		; no, ignore this packet
;---kCmd_SetMode-------------------
HandleRXData_1	movlb	0	;bank 0
	movlw	low RX_Data
	movwf	FSR1L
	movlw	high RX_Data
	movwf	FSR1H
;
	movlw	low TX_Data
	movwf	FSR0L
	movlw	high TX_Data
	movwf	FSR0H
;
	moviw	FSR1++
	movwf	Param78	;aka RX_Data
	movlw	kCmd_SetMode
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMode_end
; Set Mode
	movlw	kMaxMode+1
	subwf	INDF1,W
	SKPB		;kMaxMode+1>Data
	bra	HandleRXData_Rtn
;
	moviw	FSR1++
	movwf	SysMode
	goto	TX_ACK
;
Cmd_SetMode_end:
;---kCmd_GetMode------------------
	movlw	kCmd_GetMode
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_GetMode_end
; Get Mode
	movf	SysMode,W
	movwi	FSR0++
	goto	RS232_Send_S0
;
Cmd_GetMode_end:
;---kCmd_GetRange-------------------
	movlw	kCmd_GetRange
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	kCmd_GetRange_end
; Get sensor data for sensor n
	moviw	FSR1++
	andlw	0x03
	addlw	low Range1
	movwf	FSR1L
	movlw	high Range1
	movwf	FSR1H
	moviw	FSR1++
	movwi	FSR0++
	goto	RS232_Send_S0
;
kCmd_GetRange_end:
;---kCmd_Trigger--------------------
	movlw	kCmd_Trigger
	subwf	Param78,W
	SKPZ
	bra	kCmd_Trigger_end
; Set trigger flag
	bsf	TriggerFlag
	goto	TX_ACK
kCmd_Trigger_end:
;---kCmd_SaveParams-----------------
	movlw	kCmd_SaveParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SaveParams_end
	mCall1To0	SaveParams
	goto	TX_ACK
Cmd_SaveParams_end:
;---kCmd_RestoreParams--------------
	movlw	kCmd_RestoreParams
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_RestoreParams_end
	mCall1To0	CopyToRam
	goto	TX_ACK
Cmd_RestoreParams_end:
;---kCmd_ReBoot---------------------
	movlw	kCmd_ReBoot
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_ReBoot_end
	reset		;Does not return anything
Cmd_ReBoot_end:
;---kCmd_RunBootloader--------------
	movlw	kCmd_RunBootloader
	subwf	Param78,W
	SKPZ
	bra	Cmd_RunBootloader_end
	movlw	0xFF
	movwf	EEDataTemp
	mCall1To0	EEWriteW	;Write EEPROM address in W
	reset
Cmd_RunBootloader_end:
;---kCmd_SetMasterAddr------------
	movlw	kCmd_SetMasterAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetMasterAddr_end
	moviw	FSR1++
	movwf	RS232_MasterAddr
	mCall1To0	SaveParams
	goto	RS232_Send_S0
Cmd_SetMasterAddr_end:
;---kCmd_SetSlaveAddr-------------
	movlw	kCmd_SetSlaveAddr
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SetSlaveAddr_end
	moviw	FSR1++
	movwf	RS232_SlaveAddr
	mCall1To0	SaveParams
	goto	RS232_Send_S0
Cmd_SetSlaveAddr_end:
;---kCmd_SendIDString------------
	movlw	kCmd_SendIDString
	subwf	Param78,W	;aka RX_Data
	SKPZ
	bra	Cmd_SendIDString_end
	clrf	Param79
Cmd_SendIDString_L1	movf	Param79,W
	call	IDString
	iorlw	0x00
	SKPNZ
	return
	call	StoreSerOut_S0
	incf	Param79,F
	bra	Cmd_SendIDString_L1
;
IDString	brw
	retlw	'D'
	retlw	'M'
	retlw	'F'
	retlw	'E'
	retlw	' '
	retlw	'R'
	retlw	'a'
	retlw	'n'
	retlw	'g'
	retlw	'e'
	retlw	'r'
	retlw	'\n'
	retlw	0x00
;
Cmd_SendIDString_end:
;--------------------------------
	return
;
TX_ACK	movlw	0xFF
StoreSerOut_S0	mCall1To0	StoreSerOut
	return
;
;=========================================================================================
