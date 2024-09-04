            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL05 Assembly with Keil C startup
;R. W. Melton
;November 3, 2020
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s
            OPT  1          ;Turn on listing
;****************************************************************
;EQUates
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x0
MAX_STRING	EQU	 79
QUEUE_SIZE	EQU	 80
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue buffer contents
Q_REC_SZ    EQU   18  ;Queue management record
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'
;DAC0
DAC0_BITS EQU 12
DAC0_STEPS EQU 4096
;Servo
SERVO_POSITIONS EQU 5
PWM_DUTY_5 EQU 2500 ;5% duty cycle
PWM_DUTY_10 EQU 6300 ;10% duty cycle
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
			EXPORT		Init_PIT_IRQ
			EXPORT		Init_UART0_IRQ
			EXPORT		PIT_ISR
			EXPORT		UART0_IRQHandler
			EXPORT		UART0_ISR
			EXPORT		InitQueue
			EXPORT		Enqueue
			EXPORT		Dequeue
			EXPORT		PutNumHex
			EXPORT		PutNumUB
			EXPORT		PutNumU
			EXPORT		DIVU
			EXPORT		PutChar
			EXPORT		GetChar
			EXPORT		GetStringSB
			EXPORT		PutStringSB
			EXPORT		NewLine
			EXPORT 		PWM_duty_table_0 ;include if accessed from C
			EXPORT 		DAC0_table_0 ;make available to C program
;>>>>> begin subroutine code <<<<<
Init_PIT_IRQ	PROC	{R0-R14}
				PUSH	{R0-R7}
				
				;Enable clock for PIT module
				LDR R0,=SIM_SCGC6
				LDR R1,=SIM_SCGC6_PIT_MASK
				LDR R2,[R0,#0]
				ORRS R2,R2,R1
				STR R2,[R0,#0]
				;Disable PIT timer 0
				LDR R0,=PIT_CH0_BASE
				LDR R1,=PIT_TCTRL_TEN_MASK
				LDR R2,[R0,#PIT_TCTRL_OFFSET]
				BICS R2,R2,R1
				STR R2,[R0,#PIT_TCTRL_OFFSET]
				;Set PIT interrupt priority
				LDR R0,=PIT_IPR
				LDR R1,=NVIC_IPR_PIT_MASK
				LDR R2,=NVIC_IPR_PIT_PRI_0
				LDR R3,[R0,#0]
				BICS R3,R3,R1
				ORRS R3,R3,R2
				STR R3,[R0,#0]
				;Clear any pending PIT interrupts
				LDR R0,=NVIC_ICPR
				LDR R1,=NVIC_ICPR_PIT_MASK
				STR R1,[R0,#0]
				;Unmask PIT interrupts
				LDR R0,=NVIC_ISER
				LDR R1,=NVIC_ISER_PIT_MASK
				STR R1,[R0,#0]
				;Enable PIT module
				LDR R0,=PIT_BASE
				LDR R1,=PIT_MCR_EN_FRZ
				STR R1,[R0,#PIT_MCR_OFFSET]
				;Set PIT timer 0 period for 0.01 s
				LDR R0,=PIT_CH0_BASE
				LDR R1,=PIT_LDVAL_10ms
				STR R1,[R0,#PIT_LDVAL_OFFSET]
				;Enable PIT timer 0 interrupt
				LDR R1,=PIT_TCTRL_CH_IE
				STR R1,[R0,#PIT_TCTRL_OFFSET]
				
				POP		{R0-R7}
				BX		LR
				ENDP

PIT_ISR			PROC	{R0-R14}
				PUSH	{R0-R1}
				
				LDR		R0, =RunStopWatch
				LDRB	R1, [R0, #0]
				CMP		R1, #0
				BEQ		NothingAtAll
				LDR 	R0,=Count
				LDR 	R1,[R0,#0]
				ADDS	R1, R1, #1
				STR		R1, [R0, #0]
NothingAtAll
				LDR		R0, =PIT_CH0_BASE
				LDR		R1, =PIT_TFLG_TIF_MASK
				STR		R1, [R0, #PIT_TFLG_OFFSET]

				POP		{R0-R1}
				BX 		LR
				ENDP
				
Init_UART0_IRQ		PROC	{R0-R14}
	
			PUSH	{R0,R1,R2, LR}
			
			LDR		R0, =RxQueue
			LDR		R1, =RxQRecord
			MOVS	R2, #QUEUE_SIZE
			BL		InitQueue					;initialize the queue
			
			
			LDR		R0, =TxQueue
			LDR		R1, =TxQRecord
			MOVS	R2, #QUEUE_SIZE
			BL		InitQueue					;initialize the queue
			
;Select MCGFLLCLK as UART0 clock source
										
			LDR R0,=SIM_SOPT2
			LDR R1,=SIM_SOPT2_UART0SRC_MASK
			LDR R2,[R0,#0]
			BICS R2,R2,R1
			LDR R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
			ORRS R2,R2,R1
			STR R2,[R0,#0]
			
;Set UART0 for external connection

			LDR R0,=SIM_SOPT5
			LDR R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR R2,[R0,#0]
			BICS R2,R2,R1
			STR R2,[R0,#0]
			
;Enable UART0 module clock

			LDR R0,=SIM_SCGC4
			LDR R1,=SIM_SCGC4_UART0_MASK
			LDR R2,[R0,#0]
			ORRS R2,R2,R1
			STR R2,[R0,#0]
;Enable PORT B module clock
			LDR R0,=SIM_SCGC5
			LDR R1,=SIM_SCGC5_PORTB_MASK
			LDR R2,[R0,#0]
			ORRS R2,R2,R1
			STR R2,[R0,#0]
			
;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)

			LDR R0,=PORTB_PCR2
			LDR R1,=PORT_PCR_SET_PTB2_UART0_RX
			STR R1,[R0,#0]
			
; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
			
			LDR R0,=PORTB_PCR1
			LDR R1,=PORT_PCR_SET_PTB1_UART0_TX
			STR R1,[R0,#0]

;Disable UART0 receiver and transmitter

			LDR R0,=UART0_BASE
			MOVS R1,#UART0_C2_T_R
			LDRB R2,[R0,#UART0_C2_OFFSET]
			BICS R2,R2,R1
			STRB R2,[R0,#UART0_C2_OFFSET]

;Initialize NVIC for UART0 interuppts
			
			;Set UART0 IRQ priority
			LDR	R0, =UART0_IPR
			LDR	R1, =NVIC_IPR_UART0_MASK
			LDR	R2, =NVIC_IPR_UART0_PRI_3
			LDR	R3, [R0, #0]
			BICS	R3, R3, R1
			ORRS	R3, R3, R2
			STR	R3, [R0, #0]
			
			;Clear any pending UART0 interrupts
			LDR R0,=NVIC_ICPR
			LDR R1,=NVIC_ICPR_UART0_MASK
			STR R1,[R0,#0]
			
			;Unmask UART0 interrupts
			LDR R0, =NVIC_ISER
			LDR R1,=NVIC_ISER_UART0_MASK
			STR R1,[R0,#0]

;Set UART0 for 9600 baud, 8N1 protocol
			LDR R0, =UART0_BASE
			MOVS R1,#UART0_BDH_9600
			STRB R1,[R0,#UART0_BDH_OFFSET]
			MOVS R1,#UART0_BDL_9600
			STRB R1,[R0,#UART0_BDL_OFFSET]
			MOVS R1,#UART0_C1_8N1
			STRB R1,[R0,#UART0_C1_OFFSET]
			MOVS R1,#UART0_C3_NO_TXINV
			STRB R1,[R0,#UART0_C3_OFFSET]
			MOVS R1,#UART0_C4_NO_MATCH_OSR_16
			STRB R1,[R0,#UART0_C4_OFFSET]
			MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB R1,[R0,#UART0_C5_OFFSET]
			MOVS R1,#UART0_S1_CLEAR_FLAGS
			STRB R1,[R0,#UART0_S1_OFFSET]
			MOVS R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  
			STRB R1,[R0,#UART0_S2_OFFSET] 


;Enable UART0 receiver and transmitter
			LDR R0, =UART0_BASE
			MOVS R1,#UART0_C2_T_RI
			STRB R1,[R0,#UART0_C2_OFFSET]
			
			POP{R0,R1,R2, PC}
			ENDP
				
UART0_IRQHandler
UART0_ISR	PROC	{R0-R14}
;*********************************************************
;
;
;
;*********************************************************
			CPSID	I
			PUSH	{R4-R6, LR}
			
			LDR		R4, =UART0_BASE
			MOVS	R6, #UART0_C2_TIE_MASK
			LDRB	R5, [R4, #UART0_C2_OFFSET]
			ANDS	R5, R5, R6
			BEQ		ItsNotOne
			B		ItsOne
ItsOne
			MOVS	R6, #UART0_S1_TDRE_MASK
			LDRB	R5, [R4, #UART0_S1_OFFSET]
			ANDS	R5, R5, R6
			BEQ		ItsNotOne
			B		ItsOneAgain
ItsOneAgain
			LDR		R1, =TxQRecord
			BL		Dequeue
			BCC		GreatSuccess
			BCS		UnSuccess
GreatSuccess
			STRB	R0,[R4,#UART0_D_OFFSET]
			B		NextThing
UnSuccess	
			MOVS	R5, #UART0_C2_T_RI
			STRB	R5, [R4, #UART0_C2_OFFSET]
			B		NextThing
ItsNotOne
NextThing
			MOVS	R6, #UART0_S1_RDRF_MASK
			LDRB	R5, [R4, #UART0_S1_OFFSET]
			ANDS	R5, R5, R6
			BEQ		ItIsJoever
			B		LastThing
LastThing
			LDRB	R0,[R4,#UART0_D_OFFSET]
			LDR		R1, =RxQRecord
			BL		Enqueue
			B		ItIsJoever
ItIsJoever
			CPSIE	I
			POP		{R4-R6, PC}
			
			ENDP
			
			

InitQueue	PROC	{R0-R14}
;*********************************************************************
;Initialize the queue
;Input parameters:
;	R0: address of queue buffer (unsigned word address)
;	R1: address of queue record structure (unsigned word address)
;	R2: queue capacity in bytes (unsigned byte value)
;Output parameter:
;	R1: queue record structure (via reference by unsigned word address)
;*********************************************************************
			PUSH	{R0, R1, R2, LR}
			
			STR		R0, [R1, #IN_PTR]
			STR		R0, [R1, #OUT_PTR]
			STR		R0, [R1, #BUF_STRT]
			ADDS	R0, R0, R2
			STR		R0, [R1, #BUF_PAST]
			STRB	R2, [R1, #BUF_SIZE]
			MOVS	R0, #0
			STRB	R0, [R1, #NUM_ENQD]
			
			POP		{R0, R1, R2, PC}
			ENDP
				
Dequeue		PROC	{R0-R14}
;*********************************************************************
;Remove and return a number from the queue
;Input parameters:
;	R1: address of queue record structure (unsigned word address)
;Output parameter:
;	R0: character dequeued (unsigned byte ASCII code)
;	C: dequeue operation status: 0 success; 1 failure (PSR bit flag);*********************************************************************
;****************************************************************			
			
			PUSH	{R1, R2, R3, R4, R5, R6, LR}
			
			LDR		R3, [R1, #OUT_PTR]		;get dequeue pointer address
			LDR		R4, [R1, #BUF_PAST]		;get address past buffer
			LDR		R5, [R1, #BUF_STRT]		;get buffer start
			
			LDRB	R2,[R1, #NUM_ENQD]		
			CMP		R2, #0					;check if there are enqueued bytes
			BEQ		SkipD
			
			LDRB	R0, [R3, #0]		;get the dequeued byte
			SUBS	R2, R2, #1				;decrement numEnqueued
			STRB	R2,[R1, #NUM_ENQD]
			
			ADDS	R3, R3, #1				;increment past queue item
			CMP		R3, R4
			BEQ		AdjustD
			B		KeepGoing
AdjustD		
			MOVS	R3,   R5
KeepGoing
			STR		R3, [R1, #OUT_PTR]
			ADDS	R6, R6, #0				; clear C flag
			B		TheEnd
SkipD
			SUBS	R6, R6, #0				;set C flag
			B TheEnd
TheEnd
			POP		{R1, R2, R3, R4, R5, R6, PC}
			BX 		LR
			ENDP
			
Enqueue		PROC	{R0-R14}
;*********************************************************************
;add an item to the queue
;Input parameters:
;	R0: Character to enqueue
;	R1: address of queue record structure (unsigned word address)
;Output parameter:
;	C: enqueue operation status: 0 success; 1 failure (PSR bit flag);*********************************************************************
;****************************************************************			
			
			PUSH	{R1, R2, R3, R4, R5, R6, LR}
			
			LDR		R3, [R1, #IN_PTR]		;get dequeue pointer address
			LDR		R4, [R1, #BUF_PAST]		;get address past buffer
			LDR		R5, [R1, #BUF_STRT]		;get buffer start
			
			LDRB	R6, [R1 ,#BUF_SIZE]		;get the queue size
			LDRB	R2,[R1, #NUM_ENQD]		
			CMP		R2, R6 					;check if there are enqueued bytes
			BEQ		SkipE
			
			STRB	R0, [R3, #0]			;add byte to queue
			ADDS	R2, R2, #1				;increment numEnqueued
			STRB	R2,[R1, #NUM_ENQD]
			
			ADDS	R3, R3, #0x01			;increment past queue item
			CMP		R3, R4
			BEQ		AdjustE
			B		KeepGoingE
AdjustE		
			MOVS	R3, R5
KeepGoingE
			STR		R3, [R1, #IN_PTR]
			ADDS	R6, R6, #0				; clear C flag
			B		TheEnd
SkipE		
			SUBS	R6, R6, #0				;set C flag
			B TheEndE
TheEndE
			POP		{R1, R2, R3, R4, R5, R6, PC}
			BX 		LR
			ENDP
			
PutNumHex	PROC	{R0-R14}
;********************************************************
;
;
;
;Input parameter:
;	R0: number to print in hexadecimal (unsigned word value)
;Output parameter:
;	(none)
;*********************************************************
			PUSH{R0,R1, R2, R3, LR}
			
			MOVS	R1, #28
			MOVS 	R2, #0xF
			MOVS	R4, #8
			MOVS	R5, R0
LoopInnit
			CMP		R4, #0
			BEQ		EndIt
			RORS	R5, R5, R1
			MOVS	R3, R5
			ANDS	R3, R3, R2
			
			CMP		R3, #9
			BHI		ConvertLetters
			B 		NotHigher
ConvertLetters
			ADDS	R3, R3, #55
			MOVS	R0, R3
			BL		PutChar
			SUBS	R4, R4, #1
			B 		LoopInnit
NotHigher
			ADDS	R3, R3, #0x30
			MOVS	R0, R3
			BL		PutChar
			SUBS	R4, R4, #1
			B		LoopInnit
EndIt
			POP{R0,R1, R2, R3, PC}
			
			BX 		LR
			ENDP
				
PutNumUB	PROC	{R0-R14}
;********************************************************
;Prints to the screen the text decimal representation of the 
;unsigned byte value in R0.
;Input parameter:
;	R0: number to print in decimal (unsigned byte value)
;*********************************************************
			PUSH{R0, R1, LR}
			
			MOVS	R1, #0xFF
			ANDS	R0, R0, R1
			BL 		PutNumU
			
			POP{R0, R1, PC}
			
			BX 		LR
			ENDP


PutNumU		PROC	{R0-R14}
;*********************************************************************
;Displays the text decimal representation to the terminal screen of the 
;unsigned word value in R0
;Parameters
;	Input:  R0: Number for output to terminal (unsigned word value)
;	Modify: APSR
;Uses:
;	DIVU, PutChar
;**********************************************************************

			PUSH	{R0-R6, LR}
			
			LDR		R6, =DIVTemp
			MOVS	R5, #0						;initialize remainder counter

DIVLoop
			MOVS	R1, R0
			MOVS	R0, #0x0000000A				;put number to divide in R1 and divisor in R0
			
			BL		DIVU
			STRB	R1,[R6, R5]					;add remainder into address
			CMP		R0,#0
			BEQ		EndDIVLoop
			ADDS 	R5, R5, #1					;increment remainder counter
			B		DIVLoop
EndDIVLoop	

OtherLoop
			CMP		R5, #0
			BEQ		ItsJoever
i			LDRB	R0, [R6, R5]				;load in remainder
			ADDS	R0,R0,#0x30					;add 30 bc thats what u do
			BL		PutChar
			SUBS	R5, R5, #1					;decrement remainder counter
			B		OtherLoop
			
ItsJoever
			LDRB	R0, [R6, R5]				;load in remainder
			ADDS	R0,R0,#0x30					;add 30 bc thats what u do
			BL		PutChar
			POP		{R0-R6, PC}
			
			BX		LR
			ENDP
	
	
DIVU		PROC	{R5-R15}
;*********************************************************************
;Divides the number given by another number givenand returns the  
;quotient and remainder
;Parameters
;	Input:  R0: Divisor
;			R1: Dividend
;	Output: R0: Quotient
;			R1: Remainder
;	Modify: APSR
;**********************************************************************
			
			PUSH{R2,R3,R4, LR}
			
			CMP 	R0, #0 				;check if divisor is 0
			BEQ		DivisorZero
			
			MOVS 	R2, #0				;Quotient
			B		Division
			
Division
			CMP		R1,R0				;chceks if dividend is lower than divisor
			BLO 	Remainder
			ADDS	R2, R2, #1			;adds 1 to quotient
			SUBS	R1, R1, R0			;subs the divisor from the divident
			B		Division
			
			
Remainder
			MOVS	R0, R2				;moves the quotient and remainder into R0 and R1
			
			MRS		R2, APSR
			MOVS	R3, #0x20
			LSLS	R3, R3, #24
			BICS	R2, R2, R3
			MSR		APSR, R2			;clear C
			
			B		FinalDiv
			
DivisorZero	
			
			MRS		R2, APSR
			MOVS	R3, #0x20
			LSLS	R3, R3, #24
			ORRS	R2, R2, R3
			MSR		APSR, R2
			
			B		FinalDiv
			
FinalDiv	

			POP{R2, R3, R4, PC}
			BX 		LR
			ENDP
				
PutChar		PROC	{R0-R14}
;**************************************************************************************
;
;
;
;**************************************************************************************
	
			PUSH	{R1,R2,R3, LR}
			
			LDR		R1, =TxQRecord
			
			
putCharL	;the loop until TDRE == 1
			CPSID I
			BL		Enqueue
			CPSIE I
			BCS	putCharL
			
			;store into wherever idk
			LDR		R1, =UART0_BASE
			MOVS	R2, #UART0_C2_TI_RI
			STRB	R2,[R1,#UART0_C2_OFFSET]
			
			POP		{R1, R2, R3, PC}
			BX		LR
			ENDP

GetChar		PROC	{R1-R14}
;**********************************************************************
;
;
;**********************************************************************
	
			PUSH	{LR}
			
			LDR		R1, =RxQRecord

			
getCharL	;the loop until RDRF == 1
			CPSID	I
			BL		Dequeue
			CPSIE	I
			BCS	getCharL
			

			POP		{PC}
			BX		LR
			ENDP 

GetStringSB	PROC	{R0-R14}
;*********************************************************************
;Reads a string from the terminal keyboard,
;(i.e., characters typed until "Enter" is pressed),
;stores it in memory starting at the address
;where R0 points, and echpes it t the terminal screen
;Parameters
;	Input: R1: Buffer capacity
;		   R0: Pointer to destination string
;	Modify: APSR
;Uses:
;	GetChar, PutChar
;*********************************************************************
	
			PUSH	{LR, R0, R1, R2, R3}
			
			MOVS	R3, #0
			MOVS	R2, R0					;saves the pointer address
			CMP		R1, #0
			BEQ		EndString

TheLoop
			BL		GetChar					;load the character into R0
			
			CMP		R0, #13					;check if character is ENTER
			BEQ		EndString
			
			CMP		R0, #10
			BEQ		EndString
			
			CMP		R0, #0x7F
			BEQ		TheLoop
			
			CMP 	R1, #1
			BEQ		EndString				;check for overrun
			
			STRB 	R0, [R2, R3]			;store the character into the address
			
			BL		PutChar					;call PutChar algorithm
			
			SUBS	R1, R1, #1				;confirm one charaacter was added
			ADDS	R3, R3, #1				;get next character in string
			B		TheLoop

EndString
			MOVS	R0, #NULL
			BL		PutChar
			MOVS	R0, #10
			BL 		PutChar
			MOVS	R0, #13
			BL		PutChar
			
			
			POP		{PC, R0, R1, R2, R3}
			
			BX		LR
			ENDP
			
			
			
PutStringSB	PROC	{R0-R14}
;*********************************************************************
;Displays a null-terminated string to the terminal screen,
;by using PutChar to display characters from the string.
;Parameters
;	Input: R1: Buffer capacity
;		   R0: Pointer to destination string
;	Modify: APSR
;Uses:
;	PutChar
;**********************************************************************

			PUSH	{LR, R0, R1, R2}
			
			MOVS	 R2, R0
			LDRB     R0,[R2,#0]					;get the first chracter from the pointer address
			
Loop        
			CMP      R1,#0						
            BEQ      endPutString				;check for overflow
			
            BL       PutChar
            ADDS     R2,R2,#1
            LDRB     R0,[R2,#0]					;get the next character from pointer addresss
			
			SUBS	R1, R1, #1					;confirm one charaacter was added
            
			CMP		 R0, #NULL
			BEQ		endPutString
			
			B        Loop
			
endPutString
			POP		{PC, R0, R1, R2}
			BX		LR
			ENDP
;--------------------------------------------------------------
NewLine		PROC 	{R0, R14}
			PUSH 	{R0-R7, LR} 
			MOVS 	R0,#CR
			BL		PutChar
			MOVS 	R0,#LF
			BL		PutChar
			POP 	{R0-R7, PC}
			BX		LR
			ENDP
;-------------------------------------------------------------------
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
PWM_duty_table
PWM_duty_table_0 ;include if accessed from C
;Servo positions from 1 (leftmost) to 5 (rightmost)
;Position 1: ?50% range
		DCW (PWM_DUTY_10 - 1)
;Position 2: ?25% of range
		DCW (((3 * (PWM_DUTY_10 - PWM_DUTY_5) / 4) + PWM_DUTY_5) - 1)
;Position 3: 0% of range
		DCW	((((PWM_DUTY_10 - PWM_DUTY_5) / 2) + PWM_DUTY_5) - 1)
;Position 4: ?25% of range
		DCW ((((PWM_DUTY_10 - PWM_DUTY_5) / 4) + PWM_DUTY_5) - 1)
;Position 5: ?50% of range
		DCW	(PWM_DUTY_5 - 1)

DAC0_table_0
DAC0_table
			DCW ((DAC0_STEPS - 1) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 3) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 5) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 7) / (SERVO_POSITIONS * 2))
			DCW (((DAC0_STEPS - 1) * 9) / (SERVO_POSITIONS * 2))

Instructions	DCB		"Press key for stopwatch command (C,D,H,P,T)", NULL
Help			DCB		":    C(lear), D(isplay), H(elp), P(ause), T(ime)", NULL
Gap				DCB		":	  ", NULL
MultiplyPrompt 	DCB      " x 0.01 s\0"
;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
QBuffer		SPACE	Q_BUF_SZ	;Queue contents
			ALIGN
QRecord		SPACE	Q_REC_SZ	;Queue management record
			ALIGN
RxQueue		SPACE	QUEUE_SIZE
			ALIGN
RxQRecord	SPACE	Q_REC_SZ
			ALIGN
TxQueue		SPACE	QUEUE_SIZE
			ALIGN
TxQRecord	SPACE	Q_REC_SZ
			ALIGN
String			SPACE	MAX_STRING
				ALIGN
DIVTemp			SPACE	2
				ALIGN
RunStopWatch	SPACE	1
				ALIGN
Count			SPACE	4
				ALIGN
;>>>>>   end variables here <<<<<
			ALIGN
            END
