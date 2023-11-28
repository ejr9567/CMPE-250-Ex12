/*********************************************************************/
/* Random number generator implemented using the CM0+ SysTick timer. */
/* Name:  Eric Reed, Ruhan Syed                                      */
/* Date:  11/28/23-12/5/23                                           */
/* Class:  CMPE 250                                                  */
/* Section:  L2, L4                                                  */
/*********************************************************************/

#include "RNG.h"

#include "MKL05Z4.h"


static UInt32 rngLast = 0;

// -------------------------------------------------------
// | Initialize RNG, enabling the SysTick timer          |
// | for polling operation.                              |
// |                                                     |
// | Input: none                                         |
// | Output: none                                        |
// -------------------------------------------------------
void Init_RNG(void) {

	rngLast = 0;

// See ARMv6-M ref, B3.3.3
#define SysTick_CTRL_CPUClock_NoInt_Enable ( \
		SysTick_CTRL_CLKSOURCE_Msk | /* // KL05 ref 3.3.1.3: CLKSOURCE = 1 --> CPU clock */ \
		SysTick_CTRL_ENABLE_Msk \
	)

	SysTick->CTRL = SysTick_CTRL_CPUClock_NoInt_Enable;
}


// -------------------------------------------------------
// | Generate value using RNG, querying the SysTick      |
// | timer's current value.                              |
// |                                                     |
// | -> Input: none                                      |
// | -> Output: pseudorandom (time-variate) 32-bit value |
// -------------------------------------------------------
UInt32 RNG_next(void) {
	// Retrieve current 24-bit time value
	UInt32 time = SysTick->VAL;

	// Calculate a pseudorandom most significant byte
	// by subtracting the new time from the last RNG value
	// and taking the second-least significant byte
	UInt32 nextValMSB = ((rngLast - time) & 0x00FF00) << 16;

	// Calculate a pseudorandom lower three bytes
	// by adding the new time to the last RNG value
	// Includes bonus entropy from potential addition overflow
	UInt32 nextVal = nextValMSB | (rngLast + time);

	// Rotate right by a random number of bits (lowest nibble)
	UInt8 rotateBitCount = nextVal & 0xF;
	nextVal = (nextVal >> rotateBitCount) | (nextVal << (32 - rotateBitCount));

	// Save current val for next cycle
	rngLast = nextVal;

	return nextVal;
}
