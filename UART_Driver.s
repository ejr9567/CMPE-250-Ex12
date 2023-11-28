    TTL	UART Driver
;****************************************************************
;Subroutines implementing common read/write character and string
;operations over the KL05's UART module, implemented using
;interrupt-based operation, as well as an integer division subroutine,
;and a queue implementation using a ring buffer

;Subroutines:
;* Init_UART0_IRQ
;* GetChar
;* PutChar
;* UART0_ISR
;* DIVU
;* PutStringSB
;* PutNumU
;* InitQueue
;* Dequeue
;* Enqueue
;* PutNumHex
;* PutNumUB

;Name:	Eric Reed
;Date:	November 5, 2023
;Class:	CMPE-250
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
; MKL05Z4.s
    GET     MKL05Z4.s
    OPT     1   ;Turn on listing
;****************************************************************
;EQUates
;Characters
BS          EQU  0x08
ESC         EQU  0x1B
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
;Maximum number of characters required to represent unsigned 32-bit value in decimal
MAX_DECIMAL_STR_LEN EQU 10
;Amount to divide by to produce each bit of decimal output
PUTNUMU_DIVISOR EQU 10

; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue buffer contents
Q_REC_SZ    EQU   18  ;Queue management record
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'

; Max number of characters that can be stored in
; UART driver transmit and receive buffers
UART0_BUFFER_SIZE   EQU 80

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
;---------------------------------------------------------------
;Macros
; -----------------------------------------
; MOVC: move specified immediate value into the C flag
; -- INPUT PARAMS
; $Value: immediate value; LSB is placed into the C flag
; $Scratch: register to be modified internally as part of implementation;
;           it is the caller's responsibility to save to and restore from stack if desired
; -- OUTPUT PARAMS
; C flag: $Value[0]
; -----------------------------------------
        MACRO
        MOVC    $Value,$Scratch

        ; DISABLED as optimization:
        ;PUSH    {R0}                 ; save value of scratch register on stack

        MOVS    $Scratch,$Value      ; place given immediate into scratch register
        LSRS    $Scratch,$Scratch,#1 ; shift value out; LSB goes into C flag

        ; also disabled:
        ;POP     {R0}                 ; restore original value of scratch register

        MEND
;****************************************************************
            AREA    UART_Operations_Code,CODE,READONLY

            EXPORT  Init_UART0_IRQ
            EXPORT  GetChar
            EXPORT  PutChar
            EXPORT  UART0_IRQHandler
            EXPORT  UART0_ISR
            EXPORT  DIVU
            EXPORT  PutStringSB
            EXPORT  PutNumU
            EXPORT  InitQueue
            EXPORT  Dequeue
            EXPORT  Enqueue
            EXPORT  PutNumHex
            EXPORT  PutNumUB

;>>>>> begin subroutine code <<<<<
;***************************************************************
;Init_UART0_IRQ: initializes UART0 for interrupt-based serial I/O
;through the KL05Z USB virtual serial port with
;eight data bits, no parity, and one stop bit at 9600 baud.
;---------------------------------------------------------------
;Inputs: (none)
;Outputs: (none)
;***************************************************************
Init_UART0_IRQ PROC {R0-R13}
            PUSH {R0-R3,LR}

        ; Initialize any data structures associated with handling UART0 interrupts
        MOVS    R2,#UART0_BUFFER_SIZE
        LDR     R0,=RxQBuffer   ; Initialize receive queue
        LDR     R1,=RxQRecord
        BL      InitQueue
        LDR     R0,=TxQBuffer   ; Initialize transmit queue
        LDR     R1,=TxQRecord
        BL      InitQueue

;Select MCGFLLCLK as UART0 clock source
            LDR  R0,=SIM_SOPT2
            LDR  R1,=SIM_SOPT2_UART0SRC_MASK
            LDR  R2,[R0,#0]
            BICS R2,R2,R1
            LDR  R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
            ORRS R2,R2,R1
            STR  R2,[R0,#0]
;Set UART0 for external connection
            LDR  R0,=SIM_SOPT5
            LDR  R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR  R2,[R0,#0]
            BICS R2,R2,R1
            STR  R2,[R0,#0]
;Enable UART0 module clock
            LDR  R0,=SIM_SCGC4
            LDR  R1,=SIM_SCGC4_UART0_MASK
            LDR  R2,[R0,#0]
            ORRS R2,R2,R1
            STR  R2,[R0,#0]
;Enable PORT B module clock
            LDR  R0,=SIM_SCGC5
            LDR  R1,=SIM_SCGC5_PORTB_MASK
            LDR  R2,[R0,#0]
            ORRS R2,R2,R1
            STR  R2,[R0,#0]
;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
            LDR  R0,=PORTB_PCR2
            LDR  R1,=PORT_PCR_SET_PTB2_UART0_RX
            STR  R1,[R0,#0]
;Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
            LDR  R0,=PORTB_PCR1
            LDR  R1,=PORT_PCR_SET_PTB1_UART0_TX
            STR  R1,[R0,#0]
;Disable UART0 receiver and transmitter
            LDR  R0,=UART0_BASE
            MOVS R1,#UART0_C2_T_R
            LDRB R2,[R0,#UART0_C2_OFFSET]
            BICS R2,R2,R1
            STRB R2,[R0,#UART0_C2_OFFSET]

;Initialize NVIC for UART0 interrupts
; Set UART0 IRQ priority
            LDR     R0,=UART0_IPR
            LDR     R2,=NVIC_IPR_UART0_PRI_3
            LDR     R3,[R0,#0]
            ORRS    R3,R3,R2
            STR     R3,[R0,#0]
; Clear any pending UART0 interrupts
            LDR     R0,=NVIC_ICPR
            LDR     R1,=NVIC_ICPR_UART0_MASK
            STR     R1,[R0,#0]
; Unmask UART0 interrupt
            LDR     R0,=NVIC_ISER
            LDR     R1,=NVIC_ISER_UART0_MASK
            STR     R1,[R0,#0]

;Set UART2 for 9600 baud, 8N1 protocol
            LDR     R0,=UART0_BASE
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
            MOVS R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB R1,[R0,#UART0_S2_OFFSET]
;Enable UART0 transmitter, receiver, and receive interrupt
            MOVS R1,#UART0_C2_T_RI
            STRB R1,[R0,#UART0_C2_OFFSET]

            POP  {R0-R3,PC}
            ENDP

;***************************************************************
;GetChar: reads a single character from the terminal keyboard into R0
;---------------------------------------------------------------
;Inputs: (none)
;Outputs: R0: character read over UART0
;***************************************************************
GetChar     PROC    {R1-R13}
            PUSH    {R1-R2,LR}

            LDR     R1,=RxQRecord

;Poll queue until a character has been placed in it
;Mask and unmask interrupts when checking to prevent the queue
;from being modified in the middle of the dequeue operation ("critical section")
GetCharPoll CPSID   I   ; Mask interrupts
            BL      Dequeue
            CPSIE   I   ; Unmask interrupts
            BCS     GetCharPoll ; Loop if the dequeue failed (C=1)

            POP     {R1-R2,PC}
            ENDP

;***************************************************************
;PutChar: displays the single character from R0 to the terminal screen
;---------------------------------------------------------------
;Inputs: R0: character to write to UART0
;Outputs: (none)
;***************************************************************
PutChar     PROC {R0-R13}
            PUSH {R0-R2,LR}

            LDR     R1,=TxQRecord

;Poll queue until a character has been placed in it
;Mask and unmask interrupts when checking to prevent the queue
;from being modified in the middle of the enqueue operation ("critical section")
PutCharPoll CPSID   I   ; Mask interrupts
            BL      Enqueue
            CPSIE   I   ; Unmask interrupts
            BCS     PutCharPoll ; Loop if the enqueue failed (C=1)

;Enable transmit interrupt, if it was disabled when the queue was emptied
            MOVS    R1,#UART0_C2_TI_RI  ; Value for control register indicating transmitter + interrupt enabled, and receiver + interrupt enabled
            LDR     R2,=UART0_C2    ; Address of control register
            STRB    R1,[R2,#0]

            POP  {R0-R2,PC}
            ENDP

;***************************************************
;UART0_ISR: interrupt service routine processing for
;transmit/receive UART0 interrupts.
;Uses: Dequeue, Enqueue
;***************************************************
UART0_IRQHandler
UART0_ISR   PROC    {R0-R14}

            CPSID   I   ; avoid nested interrupts by masking them

            PUSH    {LR}

            LDR     R2,=UART0_BASE  ; load base address for UART0 registers
            LDRB    R3,[R2,#UART0_S1_OFFSET]    ; load current value of status reg 1

            ; check if transmit interrupt enabled
            LDRB    R0,[R2,#UART0_C2_OFFSET]    ; check Transmit Interrupt Enable bit of UART0 control reg 2
            LSRS    R1,R0,#(UART0_C2_TIE_SHIFT + 1) ; by shifting it into the C flag
            BCC     UART0_ISR_RxCheck   ; skip if transmit disabled

            ; check if transmit ready
            LSRS    R1,R3,#(UART0_S1_TDRE_SHIFT + 1)    ; check Transmit Data Register Empty bit of UART0 status reg 1 in the same fashion
            BCC     UART0_ISR_RxCheck   ; skip if transmit not ready (we received a receive interrupt)

            ; pop character from transmit queue
            LDR     R1,=TxQRecord
            BL      Dequeue
            BCS     UART0_ISR_TxDequeueFail ; branch if queue is empty

            STRB    R0,[R2,#UART0_D_OFFSET] ; move character into UART transmit register
            B       UART0_ISR_RxCheck

UART0_ISR_TxDequeueFail ; disable transmit interrupt
            MOVS    R0,#UART0_C2_T_RI   ; enable transmitter (but not interrupt), and receiver + interrupt
            STRB    R0,[R2,#UART0_C2_OFFSET]

UART0_ISR_RxCheck
            ; check if receive completed (we received a receive interrupt)
            LSRS    R0,R3,#(UART0_S1_RDRF_SHIFT + 1)    ; check Receive Data Register Full bit of UART0 status reg 1 in the same fashion as in Tx check
            BCC     UART0_ISR_End   ; skip if receive not complete (we did not receive a receive interrupt)

            ; push character into receive queue
            LDRB    R0,[R2,#UART0_D_OFFSET] ; move character out of UART receive register
            LDR     R1,=RxQRecord
            BL      Enqueue
            ; character is gone forever if the queue was full :(


UART0_ISR_End

            CPSIE   I   ; unmask interrupts
            POP     {PC}
            ENDP

; -----------------------------------------
; DIVU: unsigned word division
; -- INPUT PARAMS
; R0: divisor
; R1: dividend
; -- OUTPUT PARAMS
; R0: quotient
; R1: remainder
; C flag: 0 for valid result, 1 for invalid (inputs are unchanged)
; -----------------------------------------

DIVU        PROC    {R3-R14}
            PUSH    {R2,R3}          ; save R2, R3 on stack, used for divisor and C flag mask

            MRS     R2,APSR          ; prepare to modify C flag
            MOVS    R3,#1            ; place C flag mask in R3
            LSLS    R3,#APSR_C_SHIFT

            CMP     R0,#0            ; if (Divisor == 0) {
            BNE     DIVU_Good

            ORRS    R2,R2,R3         ; C = 1;
            B       DIVU_End         ; goto DONE; }

DIVU_Good   BICS    R2,R2,R3         ; C = 0; // divisor > 0
            MOVS    R2,R0            ; clear out R0 for quotient; use R2 for divisor
            MOVS    R0,#0            ; Quotient = 0

DIVU_Loop   CMP     R1,R2            ; while (Dividend >= Divisor) {
            BLO     DIVU_End
            ADDS    R0,R0,#1         ; Quotient = Quotient + 1;
            SUBS    R1,R1,R2         ; Dividend = Dividend - Divisor;

            B       DIVU_Loop

DIVU_End                             ; } DONE: // end loop

            MSR     APSR,R2          ; save new C flag value

                                     ; Remainder = Dividend; // R0 has remainder already

            POP     {R2,R3}          ; restore old value of R2 and R3
            BX      LR               ; return
            ENDP

;-----------------------------------------------------------------------
;GetStringSB: read a line of text over UART0 into a user-supplied buffer
;--> Inputs:
;R0: starting address of destination string
;R1: max length of destination string, incl. null byte
;--> No outputs
;--> Calls: GetChar, PutChar
;-----------------------------------------------------------------------
GetStringSB PROC    {R0-R14}

            PUSH    {R0-R4,LR}

            MOVS    R2,R0    ; save destination address in R2
            MOVS    R3,#0    ; number of characters read so far
            MOVS    R4,#0    ; whether the last character was ESC
            SUBS    R1,R1,#1 ; R1 <- max # of chars (buffer len - 1, for null terminator)

GetStringSBLoop
            BL      GetChar

            ; check for enter key hit
            CMP     R0,#CR
            BEQ     GetStringSBLoopEnd

            ; check if the character was the backspace character BS,
            ; which we want to use to delete the last typed character
            CMP     R0,#BS
            BNE     GetStringSBNoBS

            ; make sure there is a character to delete
            ; (that num of chars read > 0)
            CMP     R3,#0
            ; if there is not, skip processing the BS
            BEQ     GetStringSBLoop
            ; delete last written char from terminal:
            ; move cursor back, send space, move cursor back
            MOVS    R0,#BS
            BL      PutChar
            MOVS    R0,#' '
            BL      PutChar
            MOVS    R0,#BS
            BL      PutChar
            ; 'delete' last char of string by subtracting 1
            ; from its length; the char will be overwritten
            ; by either a new char or the null terminator
            SUBS    R3,R3,#1
            ; stop further processing of this BS

GetStringSBNoBS

            ; if we have read == R1 characters, skip displaying + storing the character
            CMP     R3,R1
            BEQ     GetStringSBLoop

            ; check for ESC character last time and bracket this time,
            ; indicating escape sequence
            CMP     R4,#0
            BEQ     GetStringSBCheckESC ; ESC was not hit last time
            CMP     R0,#'['
            BNE     GetStringSBCheckESC ; ] was not hit this time
            ; read rest of escape sequence 
GetStringSBWhileReadEscSeq
            BL      GetChar
            CMP     R0,#'~' ; check for ~ indicating end of escape sequence
            BNE     GetStringSBWhileReadEscSeq
            ; go back to reading characters normally
            B       GetStringSBLoop

            ; record in R4 whether the ESC key was hit this time
GetStringSBCheckESC
            CMP     R0,#ESC
            BNE     GetStringSBNoESC
            MOVS    R4,#1
            B       GetStringSBLoop ; skip processing of ESC
GetStringSBNoESC
            MOVS    R4,#0

            ; check if the character was a control character
            ; control characters are 0x00-0x1F and 0x7F
            ; if it is, skip further processing of the character
            CMP     R0,#0x7F
            BEQ     GetStringSBLoop
            CMP     R0,#0x1F
            BLS     GetStringSBLoop

            ; display character
            BL      PutChar

            ; store character, increment # of chars
            STRB    R0,[R2,R3]
            ADDS    R3,R3,#1

            ; repeat
            B       GetStringSBLoop

GetStringSBLoopEnd

            ; append null terminator
            MOVS    R4,#NULL
            STRB    R4,[R2,R3]

            ; move to beginning of next line
            MOVS    R0,#CR
            BL      PutChar
            MOVS    R0,#LF
            BL      PutChar

            POP     {R0-R4,PC}
            ENDP

;-----------------------------------------------------------------------
;PutStringSB: write a line of text over UART0 from a user-supplied buffer
;--> Inputs:
;R0: starting address of source string
;R1: max length of source string, incl. null byte
;--> No outputs
;--> Calls: PutChar
;-----------------------------------------------------------------------
PutStringSB PROC    {R0-R14}

            PUSH    {R0-R3,LR}

            MOVS    R2,R0       ; save starting address
            MOVS    R3,#0       ; number of characters written

PutStringSBLoop
            ; load character
            LDRB    R0,[R2,R3]

            ; exit if we found the null terminator
            CMP     R0,#0
            BEQ     PutStringSBLoopEnd

            ; write character
            BL      PutChar

            ; increment number of characters written
            ADDS    R3,R3,#1

            ; only repeat if we have not read all characters in the buffer (missing null terminator)
            CMP     R3,R1
            BNE     PutStringSBLoop

PutStringSBLoopEnd

            POP     {R0-R3,PC}

            ENDP

;-----------------------------------------------------------------------
;PutNumU: write an unsigned word value as a string over UART0
;--> Inputs:
;R0: unsigned word value
;--> No outputs
;--> Calls: DIVU, PutChar
;-----------------------------------------------------------------------
PutNumU     PROC    {R0-R14}

            PUSH    {R0-R3,LR}

            ; get address of first destination character
            ; we leave it at addr of first dest + 1, since the first loop
            ; iteration will subtract 1 from it
            LDR     R2,=PutNumUWorkBuffer + PUTNUMU_WORK_BUFFER_LEN
            ; store the actual addr of last char separately,
            ; it's used to detect the end of the string when writing it to the console
            SUBS    R3,R2,#1

PutNumULoop
            ; decrement destination pointer.
            ; the pointer is initialized past the end of the string,
            ; so that the first loop accesses the first character.
            SUBS    R2,R2,#1

            ; get the next digit
            MOVS    R1,R0               ; previous quotient becomes next dividend
            MOVS    R0,#PUTNUMU_DIVISOR ; dividing by ten
            BL      DIVU

            ; convert remainder from integer to matching character value
            ADDS    R1,R1,#'0'
            ; store in destination string
            STRB    R1,[R2,#0]
 
            ; break when quotient = 0
            CMP     R0,#0
            BNE     PutNumULoop

PutNumUOutputLoop

            ; print character in output
            LDRB    R0,[R2,#0]
            BL      PutChar

            ; increment pointer to character
            ADDS    R2,R2,#1

            ; continue loop if addr of next char is <= addr of last char
            CMP     R2,R3
            BLS     PutNumUOutputLoop

            POP     {R0-R3,PC}

            ENDP

;-----------------------------------------------------------------------
;InitQueue: initialize the given queue data structure as an empty queue
;--> Inputs:
;R0: address of desired buffer to use for queue data storage
;R1: address of queue record structure
;R2: maximum size of queue contents
;--> No outputs
;-----------------------------------------------------------------------
InitQueue   PROC    {R0-R14}

            PUSH    {R0}

            STR     R0,[R1,#IN_PTR]   ; address of next input data starts at beginning of buffer
            STR     R0,[R1,#OUT_PTR]  ; address of next output data also starts at beginning
            STR     R0,[R1,#BUF_STRT] ; buffer start address is constant as address of beginning of buffer
            ADDS    R0,R0,R2          ; calculate buffer 'past' address as buffer start + buffer size
            STR     R0,[R1,#BUF_PAST] ; buffer past address is constant as first address after buffer
            STRB    R2,[R1,#BUF_SIZE] ; buffer size is constant as given num of bytes used to alloc buffer
            MOVS    R0,#0             ; number of elements enqueued starts at 0
            STRB    R0,[R1,#NUM_ENQD]

            POP     {R0}
            BX      LR

            ENDP

;-----------------------------------------------------------------------
;Dequeue: retrieve a single character from the given queue
;--> Inputs:
;R1: address of queue record structure
;--> Outputs:
;R0: character value from queue, if not empty
;C flag: 0 if operation succeeded, 1 if queue was empty
;-----------------------------------------------------------------------
Dequeue     PROC    {R1-R14}

            PUSH    {R2-R3}

            ; make sure we have at least one item to dequeue (i.e. check number enqueued)
            LDRB    R2,[R1,#NUM_ENQD] ; retrieve number of items enqueued
            CMP     R2,#0             ; if (number enqueued != 0)
            BNE     DequeueGood       ; goto DequeueGood;

            ; --- failure case (number enqueued == 0)
            MOVC    #1,R2             ; indicate failure in C flag, using R2 as scratch
            B       DequeueEnd        ; return immediately

DequeueGood ; --- success case (number enqueued != 0)
            SUBS    R2,R2,#1          ; decrement number enqueued
            STRB    R2,[R1,#NUM_ENQD] ; store new number enqueued

            LDR     R2,[R1,#OUT_PTR]  ; load address of next byte to output
            LDRB    R0,[R2,#0]        ; load next byte to output
            ADDS    R2,R2,#1          ; increment address of next byte to output

            ; check if the output pointer has moved past the end of the buffer
            LDR     R3,[R1,#BUF_PAST] ; load first address past the buffer
            CMP     R2,R3             ; if (addr of next byte != first address past buffer)
            BNE     DequeueNoLoop     ; goto DequeueNoLoop;
            LDR     R2,[R1,#BUF_STRT] ; address of next byte to output = first address in buffer

DequeueNoLoop
            STR     R2,[R1,#OUT_PTR]  ; store new address of next byte

            MOVC    #0,R2             ; indicate success in C flag, using R2 as scratch

DequeueEnd  POP     {R2-R3}

            BX      LR

            ENDP

;-----------------------------------------------------------------------
;Enqueue: place a single character into the given queue
;--> Inputs:
;R0: character value to place in queue
;R1: address of queue record structure
;--> Outputs:
;C flag: 0 if operation succeeded, 1 if queue was full
;-----------------------------------------------------------------------
Enqueue     PROC    {R0-R14}

            PUSH    {R2-R3}

            ; make sure we have space to enqueue one item (i.e. check number enqueued)
            LDRB    R2,[R1,#NUM_ENQD] ; retrieve number of items enqueued
            LDRB    R3,[R1,#BUF_SIZE] ; retrieve max number of items enqueued
            CMP     R2,R3             ; if (number enqueued != max number of items)
            BNE     EnqueueGood       ; goto EnqueueGood;

            ; --- failure case (number enqueued == max number of items)
            MOVC    #1,R2             ; indicate failure in C flag, using R2 as scratch
            B       EnqueueEnd        ; return immediately

EnqueueGood ; --- success case (number enqueued != max number of items)
            ADDS    R2,R2,#1          ; increment number enqueued
            STRB    R2,[R1,#NUM_ENQD] ; store new number enqueued

            LDR     R2,[R1,#IN_PTR]   ; load address of next byte to input
            STRB    R0,[R2,#0]        ; input the byte
            ADDS    R2,R2,#1          ; increment address of next byte to input

            ; check if the input pointer has moved past the end of the buffer
            LDR     R3,[R1,#BUF_PAST] ; load first address past the buffer
            CMP     R2,R3             ; if (addr to place next byte != first address past buffer)
            BNE     EnqueueNoLoop     ; goto EnqueueNoLoop;
            LDR     R2,[R1,#BUF_STRT] ; address to place next byte = first address in buffer

EnqueueNoLoop
            STR     R2,[R1,#IN_PTR]   ; store new address to place next byte

            MOVC    #0,R2             ; indicate success in C flag, using R2 as scratch

EnqueueEnd  POP     {R2-R3}

            BX      LR

            ENDP

;-----------------------------------------------------------------------
;PutNumHex: print hex representation of unsigned word value over UART
;--> Inputs:
;R0: unsigned word value to print
;--> No outputs
;-----------------------------------------------------------------------
PutNumHex   PROC    {R0-R14}

            PUSH    {R0-R4,LR}

            MOVS    R1,R0             ; move value into R1 to free up R0 for PutChar usage
            MOVS    R2,#7             ; nibble down counter: 8 nibbles to process (loop breaks when < 0)
            MOVS    R3,#0x0F          ; mask for lowest nibble = 0x0000000F
            MOVS    R4,#28            ; # of bits to rotate right to get next nibble

PutNumHexLoop
            RORS    R1,R1,R4          ; put next nibble into least significant nibble
            MOVS    R0,R1             ; copy into R0 to prepare for AND, which can only be in-place
            ANDS    R0,R0,R3          ; mask off everything that is not least significant nibble

            ; --- generate ASCII code based on nibble value
            CMP     R0,#9             ; if (R0 in 0xA to 0xF)
            BHI     PutNumHexLetter   ; goto PutNumHexLetter;

            ADDS    R0,R0,#'0'        ; calculate '0'-'9' from 0x0-0x9
            B       PutNumHexCalculated

PutNumHexLetter
            ADDS    R0,R0,#('A' - 10) ; calculate 'A'-'F' from 0xA-0xF

PutNumHexCalculated

            BL      PutChar           ; send character over UART

            SUBS    R2,R2,#1          ; decrement down counter
            BPL     PutNumHexLoop     ; loop if there are more nibbles to process

            POP     {R0-R4,PC}

            ENDP

;-----------------------------------------------------------------------
;PutNumUB: print decimal representation of unsigned byte value over UART
;--> Inputs:
;R0: unsigned byte value to print (only least significant byte is printed)
;--> No outputs
;-----------------------------------------------------------------------
PutNumUB    PROC    {R0-R14}

            PUSH    {R0-R1,LR}

            MOVS    R1,#0xFF          ; mask for lowest byte = 0x000000FF
            ANDS    R0,R0,R1          ; mask off everything that is not least significant byte

            BL      PutNumU           ; print byte value as decimal over UART

            POP     {R0-R1,PC}

            ENDP

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Constants
            AREA    UART_Operations_Const,DATA,READONLY

Newline     DCB     CR,LF
NewlinePast

NewlineLen  EQU     (NewlinePast - Newline)

            ALIGN
;****************************************************************
;Variables
            AREA    UART_Operations_Data,DATA,READWRITE
;>>>>> begin variables here <<<<<
PutNumUWorkBuffer
            SPACE   MAX_DECIMAL_STR_LEN
PUTNUMU_WORK_BUFFER_LEN EQU . - PutNumUWorkBuffer

            ALIGN

RxQBuffer   SPACE   UART0_BUFFER_SIZE
TxQBuffer   SPACE   UART0_BUFFER_SIZE
            ALIGN
RxQRecord   SPACE   Q_REC_SZ
            ALIGN
TxQRecord   SPACE   Q_REC_SZ
            ALIGN
;>>>>>   end variables here <<<<<
            END
