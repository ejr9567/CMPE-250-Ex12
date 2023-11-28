/*********************************************************************/
/* Lab Exercise Twelve                                               */
/* Speed-matching game using the onboard accelerometer and timer     */
/* Name:  Eric Reed, Ruhan Syed                                      */
/* Date:  11/28/23-12/5/23                                           */
/* Class:  CMPE 250                                                  */
/* Section:  L2, L4                                                  */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            March 30, 2018                                         */
/*********************************************************************/
#include "MKL05Z4.h"
#include "Ex12_c.h"
#include "Ex12_asm.h"
#include "UART_Driver.h"
#include "RNG.h"

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  Init_UART0_IRQ ();
	Init_RNG();
  __asm("CPSIE   I");  /* unmask interrupts */

  for (;;) { /* do forever */
		static char testString[] = "Test\r\n";
		PutStringSB(testString, sizeof testString);
  } /* do forever */
} /* main */
