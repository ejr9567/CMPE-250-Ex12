/*********************************************************************/
/* Random number generator implemented using the CM0+ SysTick timer. */
/* Name:  Eric Reed, Ruhan Syed                                      */
/* Date:  11/28/23-12/5/23                                           */
/* Class:  CMPE 250                                                  */
/* Section:  L2, L4                                                  */
/*********************************************************************/

#ifndef RNG_H
#define RNG_H (1)

#include "common.h"


// -------------------------------------------------------
// | Initialize RNG, enabling the SysTick timer          |
// | for polling operation.                              |
// |                                                     |
// | Input: none                                         |
// | Output: none                                        |
// -------------------------------------------------------
void Init_RNG(void);


// -------------------------------------------------------
// | Generate value using RNG, querying the SysTick      |
// | timer's current value.                              |
// |                                                     |
// | -> Input: none                                      |
// | -> Output: pseudorandom (time-variate) 32-bit value |
// -------------------------------------------------------
UInt32 RNG_next(void);

#endif
