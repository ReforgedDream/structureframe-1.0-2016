
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
.equ dummy1 =						4
.equ dummy2 =						5
.equ spiMOSIBufferOverflow =		6		
.equ spiMISOBufferOverflow =		7

.equ control1 = 0x61			//"a"
.equ control2 = 0x77			//"w"
.equ control3 = 0x6B			//"k" (ASCII)

.equ uartRAMStorageRXLength = 8	//well, a length of storage in RAM, dedicated for saving UART's received bytes
.equ uartRAMStorageTXLength = 8	//same for bytes to transmit

.equ spiRAMStorageMISOLength = 8	//a length of storage in RAM, dedicated for saving SPI's received bytes
.equ spiRAMStorageMOSILength = 8	//same for bytes to transmit through SPI

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

uartRX:				.BYTE uartRAMStorageRXLength	//allocate space for read buffer...
uartErrorsCounter:	.BYTE 1				//...and for errors counter

uartTX:				.BYTE uartRAMStorageTXLength	//allocate space for write buffer
uartTXRead:			.BYTE 1
uartTXWrite:		.BYTE 1

spiMISO:			.BYTE spiRAMStorageMISOLength	//allocate space for SPI MISO...
spiMISORead:		.BYTE 1
spiMISOWrite:		.BYTE 1		//read and write pointers

spiMOSI:			.BYTE spiRAMStorageMOSILength	//...and MOSI buffers
spiMOSIRead:		.BYTE 1
spiMOSIWrite:		.BYTE 1		//read and write pointers

subrStartLabel:		.BYTE 2
subrReadLabel:		.BYTE 2
subrWriteLabel:		.BYTE 2
subrSource:			.BYTE 1
subrLength:			.BYTE 1

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
	RJMP SPIcomplete

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
	CPI R16, spiRAMStorageMOSILength	//compare with length of the RX storage
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

SPIcomplete:	//SPI serial transfer complete interrupt

PUSH R18
 PUSH YL
  PUSH YH
   PUSHSREG

UIN R16, SPDR

LDI YL, low(spiMISO)
LDI YH, high(spiMISO)
LDS R18, spiMISOWrite
ADD YL, R18
CLR R18
ADC YH, R18

ST Y, R16

LDS R18, spiMISOWrite
INC R18
CPI	R18, spiRAMStorageMISOLength	//If write pointer reached the end of MISO buffer...
BRLO MISOWriterSkip
	CLR	R18						//...then reset it

MISOWriterSkip:

LDS R16, spiMISORead
CP R16, R18
BRNE noMISOOverflow		//If not equal, then there's no overflow

	//MISO buffer overflow
	ORI flagStorage, (1<<spiMISOBufferOverflow)	//Set MISO Overflow flag

noMISOOverflow:

STS spiMISOWrite, R18

   POPSREG
  POP YH
 POP YL
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
spiDataSequence: .db 0x69, 0x00, 0x69, 0x64, 0x00, 0x64, 0x69, 0x00
//					AE		FF		00		FF		66		FF		00		FF
//0xFF, 0x50, 0x53, 0x08, 0x00, 0x00, 0x00, 0x00
//NOP, select bank1, read register 8, spam zero 4 times to shift data out
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
; 2400 baud -> 0x01A0
; 9600 baud -> 0x0067
; 1Mbaud -> 0x0000
LDI R16, 0x00
UOUT UBRR1H, R16
LDI R16, 0x00
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

//SPI Initialization

;PB3	MISO (SPI Bus Master Input/Slave Output)
;PB2	MOSI (SPI Bus Master Output/Slave Input)
;PB1	SCK (SPI Bus Serial Clock)
;PB0	SS (SPI Slave Select input)

IN R16, DDRB
ORI R16, (1<<DDB0 | 1<<DDB1 | 1<<DDB2)		//Set 1's in PortB Direction Register
OUT DDRB, R16			//..in order to configure /SS (PB0) as output

IN R16, PORTB
ANDI R16, ~(1<<PORTB0)
OUT PORTB, R16

//SPCR - SPI Control Register
//7 – (SPIE): SPI Interrupt Enable
//6 – (SPE): SPI Enable 
//5 – (DORD): Data Order
//4 – (MSTR): Master/Slave Select
//3 – (CPOL): Clock Polarity
//2 – (CPHA): Clock Phase
//1 - (SPR1): SPI Clock Rate Select 1
//0 - (SPR0): SPI Clock Rate Select 0
//Int. enable, SPI enable, MSBit first, Master Mode, ...
//...SCK is low when idle, read on leading SCK edge, speed=clock/8 (see SPI2X)
LDI R16, 0b_1101_0001
UOUT SPCR, R16

;	SPI2X	SPR1	SPR0	SCK Frequency
;	0		0		0		f osc / 4
;	0		0		1		f osc / 16
;	0		1		0		f osc / 64
;	0		1		1		f osc / 128
;	1		0		0		f osc / 2
;	1		0		1		f osc / 8
;	1		1		0		f osc / 32
;	1		1		1		f osc / 64

//SPSR – SPI Status Register
//7 – (SPIF): SPI Interrupt Flag	r/o
//6 – (WCOL): Write COLlision Flag	r/o
//5:1 – (Res): Reserved Bits		r/o
//0 – (SPI2X): Double SPI Speed Bit
LDI R16, 0b_0000_0001
UOUT SPSR, R16

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
RJMP noNewDataUART				//bypass

	ANDI flagStorage, ~(1<<achtung)		//clear ACHTUNG

	MOVW YH:YL, XH:XL				//Load X pointer

	RCALL compareYAndRXStorageStart

	LD R16, -Y						//look into SRAM, last 3 written bytes deep
	CPI R16, control3
	BRNE noNewDataUART		//Branch if not equal to last, 3rd byte of the control sequence

		RCALL compareYAndRXStorageStart

		LD R16, -Y
		CPI R16, control2
		BRNE noNewDataUART				//Branch if last two received bytes not equal to 3rd and 2nd control sequence bytes

			RCALL compareYAndRXStorageStart

			LD R16, -Y
			CPI R16, control1
			BRNE noNewDataUART			//Branch if last 3 received bytes not equal to 3rd, 2nd and 1st control sequence bytes

	ORI flagStorage, (1<<transmit)		//Else set TRANSMIT status if proper control sequence is received

noNewDataUART:

	SBRC flagStorage, spiMISOBufferOverflow
	RJMP uartLoadTXBuffer			

LDS R18, spiMISOWrite
LDS R16, spiMISORead
CP R18, R16
BREQ nothingToSendUART

uartLoadTXBuffer:

SBRC flagStorage, uartTXBufferOverflow
RJMP nothingToSendUART				//If the flag is set then skip, CRITICAL!

LDI YL, low(spiMISO)
LDI YH, high(spiMISO)
LDS R18, spiMISORead
ADD YL, R18
CLR R18
ADC YH, R18

LD R16, Y

ANDI flagStorage, ~(1<<spiMISOBufferOverflow)	//Clear MISO overflow flag

STS subrSource, R16

LDI R16, low(uartTX)
	STS subrStartLabel, R16
LDI R16, high(uartTX)
	STS (subrStartLabel+1), R16
LDI R16, low(uartTXWrite)
	STS subrWriteLabel, R16
LDI R16, high(uartTXWrite)
	STS (subrWriteLabel+1), R16
LDI R16, low(uartTXRead)
	STS subrReadLabel, R16
LDI R16, high(uartTXRead)
	STS (subrReadLabel+1), R16
LDI R16, uartRAMStorageTXLength
	STS subrLength, R16

RCALL storeToRAMBuffer

LDS R16, subrSource
SBRC R16, 5			//no matter which bit will be tested: in case of overflow the byte equals 0xFF
ORI flagStorage, (1<<uartTXBufferOverflow)

UIN R16, UCSR1B
ORI R16, (1<<UDRIE1)	//Permit interrupt
UOUT UCSR1B, R16

LDS R18, spiMISORead
INC R18
CPI	R18, spiRAMStorageMISOLength	//If write pointer reached the end of MISO buffer...
BRLO MISOReaderSkip
	CLR	R18						//...then reset it

MISOReaderSkip:

STS spiMISORead, R18

nothingToSendUART:

//------============--------------===============--------------

SBRS flagStorage, transmit
RJMP nothingToSendSPI				//If the flag isn't set then skip

SBRC flagStorage, spiMOSIBufferOverflow
RJMP nothingToSendSPI				//If the flag is set then skip

	MOV R16, R11
	LDI ZL, Low(spiDataSequence*2)
	LDI ZH, High(spiDataSequence*2)

	ADD ZL, R16
	CLR R16
	ADC ZH, R16

	LPM R16, Z	//Load (from Program Memory) a content of the cell Z points to

	STS subrSource, R16

	LDI R16, low(spiMOSI)
		STS subrStartLabel, R16
	LDI R16, high(spiMOSI)
		STS (subrStartLabel+1), R16
	LDI R16, low(spiMOSIWrite)
		STS subrWriteLabel, R16
	LDI R16, high(spiMOSIWrite)
		STS (subrWriteLabel+1), R16
	LDI R16, low(spiMOSIRead)
		STS subrReadLabel, R16
	LDI R16, high(spiMOSIRead)
		STS (subrReadLabel+1), R16
	LDI R16, spiRAMStorageMOSILength
		STS subrLength, R16

	RCALL storeToRAMBuffer

	LDS R16, subrSource
	SBRC R16, 5			//no matter which bit will be tested: in case of overflow the byte equals 0xFF
	ORI flagStorage, (1<<spiMOSIBufferOverflow)
	
	MOV R16, R11
	INC R16
	LDI ZL, 7 //we have 8 bytes long sequency
	CP ZL, R16
	BRGE spiMOSISkipLabel1
		CLR R16
//		ANDI flagStorage, ~(1<<transmit)

	spiMOSISkipLabel1:
	MOV R11, R16

nothingToSendSPI:

//------============--------------===============--------------
LDS R16, spiMOSIRead
LDS R18, spiMOSIWrite

SBRC flagStorage, spiMOSIBufferOverflow
RJMP unreadDataSPI

CP R16, R18
BRNE unreadDataSPI

	//Buffer is empty
//	ORI flagStorage, (1<<spiAllDataSent) //we just shifted out all pending data
	RJMP exitMOSIRoutineTrue

unreadDataSPI:

LDI YL, low(spiMOSI)
LDI YH, high(spiMOSI)
ADD YL, R16
CLR R18
ADC YH, R18

LD R18, Y
UOUT SPDR, R18

ANDI flagStorage, ~(1<<spiMOSIBufferOverflow)	//Clear MOSI Overflow flag
INC R16
CPI R16, spiRAMStorageMOSILength
BRLO exitMOSIRoutine
CLR R16

exitMOSIRoutine:

STS spiMOSIRead, R16

exitMOSIRoutineTrue:

//------============--------------===============--------------

SBRS flagStorage, timeToRefresh			//7-segment digits output routine
RJMP notATimeToRefresh					//If the flag isn't set then skip

	ANDI flagStorage, ~(1<<timeToRefresh)	//CBR wont work or I am stupid -_- clear the flag

	LDI YL, low(spiMOSI)
	LDI YH, high(spiMOSI)
	MOV R16, R13
	ADD YL, R16
	LDI R16, 0x00
	ADC YH, R16
	LD R16, Y
	MOV YL, R16
	MOV YH, R16
	ANDI YL, 0b_0000_1111		//Mask high and low digits
	ANDI YH, 0b_1111_0000
	LSR YH
	LSR YH
	LSR YH
	LSR YH						//Shift high masked digit right 4 times
	MOV digitToDisp1, YH
	MOV digitToDisp2, YL		//Display both high and low digit separately on the LED

	MOV R16, R13

	MOV YL, R16
	MOV YH, R16
	ANDI YL, 0b_0000_1111		//Mask high and low digits
	ANDI YH, 0b_1111_0000
	LSR YH
	LSR YH
	LSR YH
	LSR YH						//Shift high masked digit right 4 times
	MOV digitToDisp3, YH
	MOV digitToDisp4, YL		//Display both high and low digit separately on the LED

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

	SBRC flagStorage, spiMOSIBufferOverflow
	SET

	SBRC flagStorage, spiMISOBufferOverflow
	SET

	SBRC flagStorage, uartTXBufferOverflow
	SET

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

storeToRAMBuffer:
//Warning: affects YH:YL, ZH:ZL and R16
//Arguments:
;subrStartLabel:	.BYTE 2 (Low byte first!)
;subrWriteLabel:	.BYTE 2 (Low byte first!)
;subrReadLabel:		.BYTE 2 (Low byte first!)
;subrSource:		.BYTE 1
;subrLength:		.BYTE 1
//Returns 0x00 in subrSource if there's no overflow, else returns 0xFF

	CLI

	LDS YL, subrStartLabel
	LDS YH, (subrStartLabel+1)
	LDS ZL, subrWriteLabel
	LDS ZH, (subrWriteLabel+1)
	LD R16, Z

	ADD	YL, R16		//Add write pointer to start address...
	CLR	R16
	ADC	YH, R16		//...with carry

	LDS R16, subrSource

	ST Y, R16		//...and store it in buffer + write pointer

	LDS ZL, subrReadLabel
	LDS ZH, (subrReadLabel+1)
	LD YH, Z

	LDS ZL, subrWriteLabel
	LDS ZH, (subrWriteLabel+1)
	LD YL, Z

	INC YL				//Increment write pointer

	LDS R16, subrLength
	CP YL, R16			//If write pointer reached the end of the buffer...
	BRLO subrSkip1
	CLR	YL			//...then reset it

	subrSkip1:

	CP YL, YH				//Compare the pointers
	BREQ subrOverflow		//If not equal, then there's no overflow

		//TX buffer overflow
		LDI R16, 0x00
		STS subrSource, R16

	subrExit:
	LDS ZL, subrWriteLabel
	LDS ZH, (subrWriteLabel+1)
	ST Z, YL	//Save write pointer

	SEI

RET

	subrOverflow:
	LDI R16, 0xFF
	STS subrSource, R16
	RJMP subrExit

;--------------------------------------------------------------------------------------------

//Decoding the value of R12//
Decode:		//if one of 4 digits is chosen, then select a sign to be displayed

LSL R12				//Logical Shift Left: a number gets multiplied by 2 (e.g. 0011<<1 == 0110, 3*2=6)
LDI ZL, Low(decAddrTable*2)	//Put the low part of the table of addresses' address into Z
LDI ZH, High(decAddrTable*2)	//Same for the high one
//Note that the preprocessing of the assembler interpretes addresses as words (for using in program counter)
//And, in order to appeal to specific bytes (not the whole word), we should multiply an address by 2

CLR R16		//CLeaRing the R16
ADD ZL, R12	//Adding the "offset" to the address of the table of addresses
ADC ZH, R16	//If there was an overflow string upper ^, "C" flag should appear
		//So we can handle this flag by ADding zero with Carry 
//Now Z points to the beginning of the table PLUS number of cells defined by R12
//After all, Z points exactly to desired address in the table

LPM YL, Z+	//Load (from Program Memory) a content of the cell Z points to. And increment Z.
LPM YH, Z	//Next part of final destination address
//LPM command works with bytes, not with words, remember?

MOVW ZH:ZL, YH:YL	//now the desired address goes into Z

IJMP	//go to the address of desired subsequence
//http://easyelectronics.ru/avr-uchebnyj-kurs-vetvleniya.html

RJMP Start	//Go to start of the Main Routine <--- probably, now with index jumping, this string is useless
;--------------------------------------------------------------------------------------------

firstDig:
LDI R16, 0b_0000_0001		//turn on PC0 (1st digit)
OUT PORTC, R16
	BRTS dispE			//if the number is incorrect, display the "E" letter ("Err-")
MOV R12, digitToDisp1	//Just put an appropriate number (that should be lit) in R12
RJMP decode			//go to specific digit displaying

secondDig:
LDI R16, 0b_0000_0010		//turn on PC1 (2nd digit)
OUT PORTC, R16
	BRTS dispR			//if the number is incorrect, display the "r" letter ("Err-")
MOV R12, digitToDisp2	//Just put an appropriate number (that should be lit) in R12
RJMP decode			//go to specific digit displaying

thirdDig:
LDI R16, 0b_0000_0100		//turn on PC2 (3rd digit)
OUT PORTC, R16
	BRTS dispR			//if the number is incorrect, display the "r" letter ("Err-")
MOV R12, digitToDisp3	//Just put an appropriate number (that should be lit) in R12
RJMP decode			//go to specific digit displaying

fourthDig:
LDI R16, 0b_0000_1000		//turn on PC3 (4th digit)
OUT PORTC, R16
	BRTS dispDash			//if the number is incorrect, display dash ("Err-")
MOV R12, digitToDisp4	//Just put an appropriate number (that should be lit) in R12
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
