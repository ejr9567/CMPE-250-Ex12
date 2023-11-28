            TTL Lab Exercise Twelve Supporting Assembly
;****************************************************************
;Assembly subroutines supporting the C implementation of Ex. 12
; Name:  Eric Reed, Ruhan Syed
; Date:  11/28/23-12/5/23
; Class:  CMPE 250
; Section:  L2, L4
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
;****************************************************************
;MACROs
;****************************************************************
;Program
            AREA    MyCode,CODE,READONLY
;>>>>> begin subroutine code <<<<<
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;>>>>>   end variables here <<<<<
            END
