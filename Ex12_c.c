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
#include "Accel_Driver.h"
#include <math.h>

// static void PrintByte(UInt8 byte) {
//     PutChar('0');
//     PutChar('b');
//     for (int i = 7; i >= 0; i--) {
//         int val = (byte >> i) & 0b1;
//         PutChar(val ? '1' : '0');
//     }
// }

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  Init_UART0_IRQ ();
	Init_RNG();
  Init_Accel();
  __asm("CPSIE   I");  /* unmask interrupts */

  for (;;) { /* do forever */
    float accelX = Accel_Get_Accel_X();
		UInt8* accelXAsBytes = (UInt8*) (float*) &accelX;
		UInt32 accelXBytes = (((UInt32) accelXAsBytes[3]) << 24) | (((UInt32) accelXAsBytes[2]) << 16) | (((UInt32) accelXAsBytes[1]) << 8) | ((UInt32) accelXAsBytes[0]);
    PutNumHex(accelXBytes);
    PutChar('\r');
    PutChar('\n');
	

    // beautiful work of art
    // const int numPlaces = 2;
    // int accelXWholePart = (int) accelX;
    // int accelXDecimalPart = (int) ((accelX - accelXWholePart) * pow(10, numPlaces));

    // PutNumUB(accelXWholePart);
    // PutStringSB(".", 1);
    // PutNumUB(accelXDecimalPart);
    // PutStringSB("\r\n", 2);
  } /* do forever */
} /* main */
