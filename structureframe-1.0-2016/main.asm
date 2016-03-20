
.include "m64adef.inc"

.device atmega64a

//constants definition
.equ segind0 = 0xC0	//7-segment LED indicator's digits definitions
.equ segind1 = 0xF9	//		 0
.equ segind2 = 0xA4	//		 _
.equ segind3 = 0xB0	//	5	|_|	1  <--6
.equ segind4 = 0x99	//	4	|_|.7	2 
.equ segind5 = 0x92	//
.equ segind6 = 0x82	//		 3
.equ segind7 = 0xF8	//
.equ segind8 = 0x80	//
.equ segind9 = 0x90	//
.equ segindDash = 0b_1011_1111		//"-" sign
.equ segindA = 0b_1000_1000		//"A"
.equ segindB = 0b_1000_0011		//"b"
.equ segindC = 0b_1100_0110		//"C"
.equ segindD = 0b_1010_0001		//"d"
.equ segindE = 0b_1000_0110		//"E"
.equ segindF = 0b_1000_1110		//"F"

.equ segindR = 0b_1010_1111		//"r"

.equ presetLow = 0x39	//Here we can set a desired period of time for a precise delay (2 bytes long)
.equ presetHigh = 0x01	//on 16 MHz and with no prescaler, there is approx. 62500 (0xF424) overflows per second (for 8-bit timer)
//150 Hz (0x01A1) works well for the LED indicator
//200 Hz (0x0139) for the green 4-digit indicator
.equ owfPerSecond8BitLow = 0x24
.equ owfPerSecond8BitHigh = 0xF4 //supra

//symbolic custom registers names
.def digitToDisp1 = R21			//1st digit to be displayed on the LED. Only hexadecimal digits are defined!
.def digitToDisp2 = R22			//2nd digit to be displayed on the LED
.def digitToDisp3 = R23			//3rd digit to be displayed on the LED
.def digitToDisp4 = R20			//4th digit to be displayed on the LED
.def overflowsCounterLow = R24		//incrementing every time when overflow of the timer0 occurs
.def overflowsCounterHigh = R25		//my my, there's too many overflows for a humble 8-bit register

.def flagStorage = R19			//Custom flags storage
.equ achtung =						0		//And custom symbolic bit names, why not
.equ transmit =						1		//USART1 transmission should be started
.equ timeToRefresh =				2		//LED digits should be refreshed
.equ uartTXBufferOverflow =			3
.equ allDataSent =					4
.equ dummy3 =			5		
.equ dummy4 =			6
.equ dummy5 =			7

.equ control1 = 0x61			//"a"
.equ control2 = 0x77			//"w"
.equ control3 = 0x6B			//"k" (ASCII)

.equ uartRAMStorageRXLength = 8	//well, a length of storage in RAM, dedicated for saving UART's received bytes
.equ uartRAMStorageTXLength = 8	//same for bytes to transmit

;--------------------------------------------------------------------------------------------
;Macro definitions

.macro UOUT        ;Universal OUT command. Works with either memory mapped and usual I/O registers.
.if @0 < 0x40
	OUT @0,@1         
.else
	STS @0,@1
.endif
.endm

.macro UIN        ;Universal IN command. Works with either memory mapped and usual I/O registers.
.if @1 < 0x40
	IN @0,@1         
.else
	LDS @0,@1
.endif
.endm

.macro PUSHSREG
PUSH R16		//Stores the value of R16 in stack
IN R16, SREG	//Stores SREG in R16...
PUSH R16		//...and then stores the value of SREG in stack
.endm

.macro POPSREG
POP R16			//Extract SREG value from stack...
OUT SREG, R16	//...and apply it to SREG
POP R16			//Extract R16 value from stack
.endm

;--------------------------------------------------------------------------------------------
.DSEG			//SRAM memory segment
.ORG SRAM_START //start from the beginning

uartRX: .BYTE uartRAMStorageRXLength	//allocate space for read buffer...
uartErrorsCounter: .BYTE 1				//...and for errors counter
uartTX: .BYTE uartRAMStorageTXLength	//allocate space for write buffer
uartTXRead: .BYTE 1
uartTXWrite: .BYTE 1
;--------------------------------------------------------------------------------------------
.CSEG
//Reset and Interrupt Vectors table

	.ORG 0x0000	;(RESET) 
	RJMP Reset

	.ORG INT0addr	;(INT0) External Interrupt Request 0
	RETI
	.ORG INT1addr	;(INT1) External Interrupt Request 1
	RETI
	.ORG INT2addr	; External Interrupt Request 2
	RETI
	.ORG INT3addr	; External Interrupt Request 3
	RETI
	.ORG INT4addr	; External Interrupt Request 4
	RETI
	.ORG INT5addr	; External Interrupt Request 5
	RETI
	.ORG INT6addr	; External Interrupt Request 6
	RETI
	.ORG INT7addr	; External Interrupt Request 7
	RETI
	.ORG OC2addr	;(TIMER2 COMP) Timer/Counter2 Compare Match
	RETI
	.ORG OVF2addr	;(TIMER2 OVF) Timer/Counter2 Overflow
	RETI
	.ORG ICP1addr	;(TIMER1 CAPT) Timer/Counter1 Capture Event
	RETI
	.ORG OC1Aaddr	;(TIMER1 COMPA) Timer/Counter1 Compare Match A
	RETI
	.ORG OC1Baddr	;(TIMER1 COMPB) Timer/Counter1 Compare Match B
	RETI
	.ORG OVF1addr	;(TIMER1 OVF) Timer/Counter1 Overflow
	RETI
	.ORG OC0addr	;(TIMER0 COMP) Timer/Counter0 Compare Match
	RETI

	.ORG OVF0addr	;(TIMER0 OVF) Timer/Counter0 Overflow
	RJMP Timer0Over

	.ORG SPIaddr	;(SPI,STC) Serial Transfer Complete
	RETI
	.ORG URXC0addr	;(USART0,RXC) USART0, Rx Complete
	RETI
	.ORG UDRE0addr	;(USART0,UDRE) USART0 Data Register Empty
	RETI
	.ORG UTXC0addr	;(USART0,TXC) USART0, Tx Complete
	RETI
	.ORG ADCCaddr	;(ADC) ADC Conversion Complete
	RETI
	.ORG ERDYaddr	;(EE_RDY) EEPROM Ready
	RETI
	.ORG ACIaddr	;(ANA_COMP) Analog Comparator
	RETI
	.ORG OC1Caddr	; Timer/Counter1 Compare Match C
	RETI
	.ORG ICP3addr	; Timer/Counter3 Capture Event
	RETI
	.ORG OC3Aaddr	; Timer/Counter3 Compare Match A
	RETI
	.ORG OC3Baddr	; Timer/Counter3 Compare Match B
	RETI
	.ORG OC3Caddr	; Timer/Counter3 Compare Match C
	RETI
	.ORG OVF3addr	; Timer/Counter3 Overflow
	RETI

	.ORG URXC1addr	;(USART1,RXC) USART1, Rx Complete
	RJMP U1_RXcomplete

	.ORG UDRE1addr	;(USART1,UDRE) USART1 Data Register Empty
	RJMP U1_DREmpty

	.ORG UTXC1addr	;(USART1,TXC) USART1, Tx Complete
	RETI
	.ORG TWIaddr	;(TWI) 2-wire Serial Interface
	RETI
	.ORG SPMRaddr	;(SPM_RDY) Store Program Memory Ready
	RETI

.ORG INT_VECTORS_SIZE	;end of table

;--------------------------------------------------------------------------------------------
//Interrupts Handler//

Timer0Over:	//Timer0 overflow interrupt

PUSH YH
 PUSH YL
  PUSHSREG	//Store both R16 and SREG in stack

ADIW overflowsCounterHigh:overflowsCounterLow, 1	//incrementing the whole word

CPI overflowsCounterHigh, presetHigh	//if the higher register contains half of preset...
BRLO notAFastPreset							//...AND...

CPI overflowsCounterLow, presetLow	//...the lower register contains another half, then go to executing our payload
BRLO notAFastPreset						//else - exit

//this string executes once in a certain period
	ORI flagStorage, (1<<timeToRefresh)	//Set timeToRefresh flag
	LDI overflowsCounterLow, 0x00	//zeroing the overflows counter
	LDI overflowsCounterHigh, 0x00
	INC R17							//Circling from 0 to 3 periodically (fast), to determine which digit should be lit
	CPI R17, 4						//compare with 4
	BRNE notAFastPreset				//if not 4, then exit, else reset R17 to 0
	LDI R17, 0						//reset to 0

notAFastPreset:

MOV YH, R14
MOV YL, R15
MOV R16, R13

ADIW YH:YL, 1

CPI YH, owfPerSecond8BitHigh	//if the higher register contains half of preset...
BRLO notALongPreset				//...AND...

CPI YL, owfPerSecond8BitLow		//...the lower register contains another half, then go to executing our payload
BRLO notALongPreset				//else - exit

	LDI YL, 0x00	//zeroing the overflows counter
	LDI YH, 0x00
	INC R16
	CPI R16, uartRAMStorageRXLength		//compare with length of the RX storage
	BRNE notALongPreset				//if not equal, then exit, else reset R17 to 0
	LDI R16, 0						//reset to 0

notALongPreset:

MOV R14, YH
MOV R15, YL
MOV R13, R16

  POPSREG		//Extract SREG and R16 from stack
 POP YL
POP YH

RETI

;-------------------

U1_DREmpty:	//USART 1 Data Register Empty Interrupt

PUSH R18
 PUSH YH
  PUSH YL
   PUSHSREG	//Store both R16 and SREG in stack

LDS R16, uartTXRead
LDS R18, uartTXWrite

SBRC flagStorage, uartTXBufferOverflow
RJMP unreadData

CP R16, R18
BRNE unreadData

	//Buffer is empty
	UIN R18, UCSR1B
	ANDI R18, ~(1<<UDRIE1)	//Forbid this interrupt
	UOUT UCSR1B, R18
	RJMP exitTXIntTrue

unreadData:

LDI YL, low(uartTX)
LDI YH, high(uartTX)
ADD YL, R16
CLR R18
ADC YH, R18

LD R18, Y
UOUT UDR1, R18

ANDI flagStorage, ~(1<<uartTXBufferOverflow)	//Clear TX Overflow flag
INC R16
CPI R16, uartRAMStorageTXLength
BRLO exitTXInt
CLR R16

exitTXInt:

STS uartTXRead, R16

exitTXIntTrue:

   POPSREG		//Extract SREG and R16 from stack
  POP YL
 POP YH
POP R18

RETI

;-------------------

U1_RXcomplete:	//USART 1 Receive Complete interrupt

PUSH R18
 PUSH YH			//Saving in stack registers to be used
  PUSH YL
   PUSHSREG	//Store both R16 and SREG in stack

CLR R18
UIN R16, UCSR1A		//Read error flags
SBRC R16, 2			//Skip if no error
	SUBI R18, (-1)	//Else increment R18
SBRC R16, 3			//Skip...
	SUBI R18, (-1)	//...Increment
SBRC R16, 4			//Skip...
	SUBI R18, (-1)	//...Increment

LDI YL, low(uartErrorsCounter)	//Load Y pair with address of the Errors SRAM storage
LDI YH, high(uartErrorsCounter)
LD R16, Y						//Read error count from SRAM
ADD R16, R18					//Apply new errors to it...
ST Y, R16						//And store back

UIN R16, UDR1	//Read received data
CPI R18, 0x00	//If there are read errors...
BREQ errorFF
	LDI R16, 0xFF	//Then write 0xFF instead of data
errorFF:
ST X+, R16		//Store data in SRAM and post-increment

LDI YL, low(uartRX)	//Load Y pair with start address of the SRAM storage
LDI YH, high(uartRX)
ADIW YH:YL, (uartRAMStorageRXLength-1)	//add (storage_size - 1) to Y
CP YL, XL
CPC YH, XH			//If X pointer reached last allocated cell in the storage...
BRGE exitInt2		//...then reset it to the beginning
	LDI XL, low(uartRX)	//load X pointer with address of SRAM storage
	LDI XH, high(uartRX)

exitInt2:

ORI flagStorage, (1<<achtung)	//Achtung! Some new data received!

   POPSREG		//Extract SREG and R16 from stack
  POP YL		//Extracting saved values from stack
 POP YH
POP R18

RETI

;-------------------
//End of Interrupts Handler//

;--------------------------------------------------------------------------------------------
//Storage of static data in flash
decAddrTable: .dw disp0, disp1, disp2, disp3, disp4, \
		disp5, disp6, disp7, disp8, disp9, \
		dispA, dispB, dispC, dispD, dispE, dispF, \
		dispR, dispDash	//Adresses of the labels, stored in a certain place (decAddrTable) in program memory
uartDataSequence: .db 0x48, 0x65, 0x6C, 0x6C, 0x6F, 0x2C, \
					0x20, 0x77, 0x6F, 0x72, 0x6C, 0x64, 0x21, 0x0D, 0x0A, 0x00	//"Hello, world!" + CR + LF + 0x00(ASCII)
;--------------------------------------------------------------------------------------------
Reset:

//SRAM flush
			LDI	ZL, Low(SRAM_START)	; Load Z with SRAM start address
			LDI	ZH, High(SRAM_START)
			CLR	R16					; R16 <- 0x00
Flush:		ST 	Z+, R16				; Flush byte and increment
			CPI	ZH, High(RAMEND+1)	; Is ZH == high half of the RAMEND address?
			BRNE Flush				; Loop if no
 
			CPI	ZL,Low(RAMEND+1)	; Same for low half of the address
			BRNE Flush

		CLR	ZL
		CLR	ZH

//R0-R31 flush
	LDI	ZL, 0x1E	; Address of R30 (in SRAM address space)
	CLR	ZH
	DEC	ZL			; Decrement address (flushing begins from R29 since we use R30:R31 as an address pointer)
	ST Z, ZH		; Load register with zero
	BRNE PC-2		; If Zero flag is cleared step back 2 times

//Thanks for code to DI HALT, Testicq and all fellow comrades from easyelectronics.ru

//Stack initialization
LDI R16, Low(RAMEND)
OUT SPL, R16
LDI R16, High(RAMEND)
OUT SPH, R16

//USART1 Initialization
; Set baud rate
; 9600 baud -> 0x0067
; 1Mbaud -> 0x0000
LDI R16, 0x00
UOUT UBRR1H, R16
UOUT UBRR1L, R16

; 7 - (RXC1) USART Receive Complete					(r/o)
; 6 - (TXC1) USART Transmit Complete (clearing by writing 1)
; 5 - (UDRE1) USART Data Register Empty				(r/o)
; 4 - (FE1) Frame Error (must be set to 0)			(r/o)
; 3 - (DOR1) Data Overrun (must be set to 0)		(r/o)
; 2 - (UPE1) USART Parity Error (must be set to 0)	(r/o)
; 1 - (U2X1) Double the USART Transmission Speed
; 0 - (MPCM1) Multi-Processor Communication Mode
LDI R16, 0b_0100_0000
UOUT UCSR1A, R16

; 7 - (RXCIE1) RX Complete Interrupt Enable
; 6 - (TXCIE1) TX Complete Interrupt Enable
; 5 - (UDRIE1) USART Data Register Empty Interrupt Enable
; 4 - (RXEN1) Receiver Enable
; 3 - (TXEN1) Transmitter Enable
; 2 - (UCSZ12) Character Size (combined with the UCSZn1:0 bit in UCSRC)
; 1 - (RXB81) Receive Data Bit 8 (for nine data bits only)	(r/o)
; 0 - (TXB81) Transmit Data Bit 8 (for nine data bits only)
; TX Complete' and 'UDR Empty' interrupts enable, enable receiver and transmitter, no ninth bit:
LDI R16, 0b_1011_1000
UOUT UCSR1B, R16

; Set frame format: asynchronous operation, no parity, 8 data, 1 stop bit
LDI R16, 0b_0000_0110
UOUT UCSR1C, R16

//GPIO Initialization
LDI R16, 0xFF
OUT DDRC, R16			//write 1-s into each port C...
OUT DDRA, R16			//...and port A direction registers

//Timer0 Initialization
LDI R16, 0b_0000_0001	//set CS00 bit in TCCR0 register
OUT TCCR0, R16			//now using system clock for Timer0 without prescaler
OUT TIMSK, R16			//set TOIE0 in TIMSK register
//now we have the overflow interrupt enabled for timer0

//------------------------------------
LDI XL, low(uartRX)		//load X pointer with address of SRAM storage...
LDI XH, high(uartRX)	//...used for store last 8 bytes from USART1

CLT						//clear T flag which means "incorrect number to display"

LDI R16, 0x00			//clear R16 for the order's sake

SEI			//interrupts enabled globally

;--------------------------------------------------------------------------------------------
//Main Routine//
Start:

SBRS flagStorage, achtung	//Skip next instruction is there is ACHTUNG!
RJMP noNewData				//bypass

	ANDI flagStorage, ~(1<<achtung)		//clear ACHTUNG

	MOVW YH:YL, XH:XL				//Load X pointer

	RCALL compareYAndRXStorageStart

	LD R16, -Y						//look into SRAM, last 3 written bytes deep
	CPI R16, control3
	BRNE noNewData		//Branch if not equal to last, 3rd byte of the control sequence

		RCALL compareYAndRXStorageStart

		LD R16, -Y
		CPI R16, control2
		BRNE noNewData				//Branch if last two received bytes not equal to 3rd and 2nd control sequence bytes

			RCALL compareYAndRXStorageStart

			LD R16, -Y
			CPI R16, control1
			BRNE noNewData			//Branch if last 3 received bytes not equal to 3rd, 2nd and 1st control sequence bytes

	ORI flagStorage, (1<<transmit)		//Else set TRANSMIT status if proper control sequence is received

noNewData:


SBRS flagStorage, transmit
RJMP nothingToSend				//If the flag isn't set then skip

SBRC flagStorage, uartTXBufferOverflow
RJMP nothingToSend				//If the flag is set then skip

	MOV R16, R11
	LDI ZL, Low(uartDataSequence*2)
	LDI ZH, High(uartDataSequence*2)

	ADD ZL, R16
	CLR R16
	ADC ZH, R16

	LPM R16, Z	//Load (from Program Memory) a content of the cell the Z points to
	MOV R10, R16
	RCALL storeR10ToTXBuffer

	UIN R16, UCSR1B
	ORI R16, (1<<UDRIE1)	//Permit interrupt
	UOUT UCSR1B, R16

	MOV R16, R11
	INC R16
	LDI ZL, 14 //we have 15 bytes long sequency
	CP ZL, R16
	BRGE usartTXSkipLabel1
		CLR R16
		ANDI flagStorage, ~(1<<transmit)

	usartTXSkipLabel1:
	MOV R11, R16

nothingToSend:


SBRS flagStorage, timeToRefresh			//7-segment digits output routine
RJMP notATimeToRefresh					//If the flag isn't set then skip

	ANDI flagStorage, ~(1<<timeToRefresh)	//CBR wont work or I am stupid -_- clear the flag

	LDI YL, low(uartRX)
	MOV R16, R13
	ADD YL, R16
	LDI R16, 0x00
	LDI YH, high(uartRX)
	ADC YH, R16
	LD R16, Y
	MOV R28, R16
	MOV R29, R16
	ANDI R28, 0b_0000_1111		//Mask high and low digits
	ANDI R29, 0b_1111_0000
	LSR R29
	LSR R29
	LSR R29
	LSR R29						//Shift high masked digit right 4 times
	MOV digitToDisp1, R29
	MOV digitToDisp2, R28		//Display both high and low digit separately on the LED

	MOVW YH:YL, XH:XL

	RCALL compareYAndRXStorageStart

	LD R16, -Y					//Read last stored byte
	MOV R28, R16
	MOV R29, R16
	ANDI R28, 0b_0000_1111		//Mask high and low digits
	ANDI R29, 0b_1111_0000
	LSR R29
	LSR R29
	LSR R29
	LSR R29						//Shift high masked digit right 4 times
	MOV digitToDisp3, R29
	MOV digitToDisp4, R28		//Display both high and low digit separately on the LED

	CLT			//if everything will be correct (see below) then clear the T flag (which means "incorrect number")

	CPI digitToDisp1, 0x10		//if the 1st register...
	BRLO HH1			//...is more than F...
	SET				//...then set the T flag (which means "incorrect number")

	HH1:

		CPI digitToDisp2, 0x10		//if the 2nd one...
		BRLO HH2			//...is more than F...
		SET				//...then set the T flag (which means "incorrect number")

	HH2:

			CPI digitToDisp3, 0x10		//if the 3rd one...
			BRLO HH3			//...is more than F...
			SET				//...then set the T flag (which means "incorrect number")

	HH3:

				CPI digitToDisp4, 0x10		//if the 4th one...
				BRLO HH4			//...is more than F...
				SET				//...then set the T flag (which means "incorrect number")

	HH4:

	CPI digitToDisp1, 0x00		//same for less than 0
	BRPL LL1			//BRanch if PLus (if the N flag in SREG is cleared)
	SET

	LL1:

		CPI digitToDisp2, 0x00
		BRPL LL2			//BRanch if PLus (if the N flag in SREG is cleared)
		SET

	LL2:

			CPI digitToDisp3, 0x00
			BRPL LL3			//BRanch if PLus (if the N flag in SREG is cleared)
			SET

	LL3:

				CPI digitToDisp4, 0x00
				BRPL LL4			//BRanch if PLus (if the N flag in SREG is cleared)
				SET

	LL4:

	CPI R17, 0			//Is it time to display 1st digit of LED?
	BREQ firstDigTeleport
	
	CPI R17, 1			//Is it time to display 2nd digit of LED?
	BREQ secondDigTeleport

	CPI R17, 2			//Is it time to display 3rd digit of LED?
	BREQ thirdDigTeleport

	CPI R17, 3			//Is it time to display 3rd digit of LED?
	BREQ fourthDigTeleport

notATimeToRefresh:

RJMP Start		//Go to start
//End of Main Routine//
;--------------------------------------------------------------------------------------------

firstDigTeleport:
JMP firstDig

secondDigTeleport:
JMP secondDig

thirdDigTeleport:
JMP thirdDig

fourthDigTeleport:
JMP fourthDig		// ^__^'
;--------------------------------------------------------------------------------------------

//Subroutines

compareYAndRXStorageStart:
//Warning: affects YH:YL and R16
LDI R16, low(uartRX)		//Check if we at the start address
	CP YL, R16
LDI R16, high(uartRX)
	CPC YH, R16
BRNE jumpNotEnd
ADIW YH:YL, uartRAMStorageRXLength	//If so, go to the end of the storage and one byte further
jumpNotEnd:
RET
;----------------------------------

; Store data in USART TX Buffer
; R10 contains data to be stored in TX buffer

storeR10ToTXBuffer:
//Warning: affects YH:YL and R16
UIN R16, UCSR1B
ANDI R16, ~(1<<UDRIE1)	//Forbid ALL THE INTERRUPTS that may affect TX buffer and/or its pointers
UOUT UCSR1B, R16

LDI	YL, low(uartTX)		//Load Y with the start address of the buffer
LDI	YH, high(uartTX)
LDS R16, uartTXWrite	//Load R16 with write pointer

ADD	YL, R16		//Add write pointer to start address...
CLR	R16
ADC	YH, R16		//...with carry

MOV R16, R10	//Extract byte from R10...

ST Y, R16		//...and store it in buffer + write pointer

LDS YH, uartTXRead	//Get read pointer...
LDS YL, uartTXWrite	//...and write pointer again
INC YL				//Increment write pointer

CPI	YL, uartRAMStorageTXLength	//If write pointer reached the end of TX buffer...
BRLO TXWriterSkip
	CLR	YL						//...then reset it

TXWriterSkip:
CP YL, YH				//Compare the pointers
BRNE noTXOverflow		//If not equal, then there's no overflow

	//TX buffer overflow
	ORI flagStorage, (1<<uartTXBufferOverflow)	//Set TX Overflow flag

noTXOverflow:
STS	uartTXWrite, YL	//Save write pointer

UIN R16, UCSR1B
ORI R16, (1<<UDRIE1)	//Permit interrupt(s)
UOUT UCSR1B, R16
RET
;--------------------------------------------------------------------------------------------

//Decoding the value of R18//
Decode:		//if one of 4 digits is chosen, then select a sign to be displayed

LSL R18				//Logical Shift Left: a number gets multiplied by 2 (e.g. 0011<<1 == 0110, 3*2=6)
LDI ZL, Low(decAddrTable*2)	//Put the low part of the table of addresses' address into the Z
LDI ZH, High(decAddrTable*2)	//Same for the high one
//Note that the preprocessing of the assembler interpretes addresses as words (for using in program counter)
//And, in order to appeal to specific bytes (not the whole word), we should multiply an address by 2

CLR R16		//CLeaRing the R16
ADD ZL, R18	//Adding the "offset" to the address of the table of addresses
ADC ZH, R16	//If there was an overflow string upper ^, the "C" flag should appear
		//So we can handle this flag by ADding zero with Carry 
//Now the Z points to the beginning of the table PLUS number of cells defined by the R18
//After all, the Z points exactly to desired address in the table

LPM YL, Z+	//Load (from Program Memory) a content of the cell the Z points to. And increment the Z.
LPM YH, Z	//Next part of final destination address
//LPM command works with bytes, not with words, remember?

MOVW ZH:ZL, YH:YL	//now the desired address goes into the Z

IJMP	//go to the address of desired subsequence
//http://easyelectronics.ru/avr-uchebnyj-kurs-vetvleniya.html

RJMP Start	//Go to start of the Main Routine <--- probably, now with index jumping, this string is useless
;--------------------------------------------------------------------------------------------

firstDig:
LDI R16, 0b_0000_0001		//turn on PC0 (1st digit)
OUT PORTC, R16
BRTS dispE			//if the number is incorrect, display the "E" letter ("Err-")
	MOV R18, digitToDisp1	//Just put an appropriate number (that should be lit) in R18
RJMP decode			//go to specific digit displaying

secondDig:
LDI R16, 0b_0000_0010		//turn on PC1 (2nd digit)
OUT PORTC, R16
BRTS dispR			//if the number is incorrect, display the "r" letter ("Err-")
	MOV R18, digitToDisp2	//Just put an appropriate number (that should be lit) in R18
RJMP decode			//go to specific digit displaying

thirdDig:
LDI R16, 0b_0000_0100		//turn on PC2 (3rd digit)
OUT PORTC, R16
BRTS dispR			//if the number is incorrect, display the "r" letter ("Err-")
	MOV R18, digitToDisp3	//Just put an appropriate number (that should be lit) in R18
RJMP decode			//go to specific digit displaying

fourthDig:
LDI R16, 0b_0000_1000		//turn on PC3 (4th digit)
OUT PORTC, R16
BRTS dispDash			//if the number is incorrect, display dash ("Err-")
	MOV R18, digitToDisp4	//Just put an appropriate number (that should be lit) in R18
RJMP decode			//go to specific digit displaying
;--------------------------------------------------------------------------------------------

disp0:
LDI R16, segind0	//displays 0...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp1:
LDI R16, segind1	//displays 1...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp2:
LDI R16, segind2	//displays 2...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp3:
LDI R16, segind3	//displays 3...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp4:
LDI R16, segind4	//displays 4...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp5:
LDI R16, segind5	//displays 5...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp6:
LDI R16, segind6	//displays 6...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp7:
LDI R16, segind7	//displays 7...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp8:
LDI R16, segind8	//displays 8...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

disp9:
LDI R16, segind9	//displays 9...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispA:
LDI R16, segindA	//displays A...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispB:
LDI R16, segindB	//displays B...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispC:
LDI R16, segindC	//displays C...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispD:
LDI R16, segindD	//displays D...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispE:
LDI R16, segindE	//displays E...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

dispF:
LDI R16, segindF	//displays F...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

;---

dispR:
LDI R16, segindR	//displays R...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start

;---

dispDash:
LDI R16, segindDash	//displays "-"...
OUT PORTA, R16		//...on the LED indicator
RJMP Start		//Get back to the start
