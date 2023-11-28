/*************************************************************************/
/* Subroutines implementing common read/write character and string       */
/* operations over the KL05's UART module, implemented using             */
/* interrupt-based operation, as well as an integer division subroutine, */
/* and a queue implementation using a ring buffer                        */
/*                                                                       */
/* Subroutines:                                                          */
/* * Init_UART0_IRQ                                                      */
/* * GetChar                                                             */
/* * PutChar                                                             */
/* * UART0_ISR                                                           */
/* * DIVU                                                                */
/* * PutStringSB                                                         */
/* * PutNumU                                                             */
/* * InitQueue                                                           */
/* * Dequeue                                                             */
/* * Enqueue                                                             */
/* * PutNumHex                                                           */
/* * PutNumUB                                                            */
/*                                                                       */
/* Name:	Eric Reed                                                      */
/* Date:	November 5, 2023                                               */
/* Class:	CMPE-250                                                       */
/*************************************************************************/

#ifndef UART_DRIVER_H
#define UART_DRIVER_H (1)

#include "common.h"

// subroutines
char GetChar (void);
void GetStringSB (char String[], int StringBufferCapacity);
void Init_UART0_IRQ (void);
void PutChar (char Character);
void PutNumHex (UInt32);
void PutNumUB (UInt8);
void PutStringSB (char String[], int StringBufferCapacity);

#endif // UART_DRIVER_H
