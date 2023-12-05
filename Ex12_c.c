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
#include "PIT_Driver.h"
#include <math.h>

void PIT_handler(void) {
  Accel_Update_Velocity();
}

void Newline(void) {
    PutChar('\r');
    PutChar('\n');
}

extern UInt8 RxQRecord[18];

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  Init_UART0_IRQ ();
	Init_RNG();
  Init_Accel();
  Init_PIT(PIT_handler);
  __asm("CPSIE   I");  /* unmask interrupts */

  while (1) {
    Accel_Calibrate();

    int RoundNum = 0;
    int Points = 0;
    int LifePoints = 5;

    PutStringSB ("Welcome to Ice Cream Frenzy!", 100);
    Newline();
    PutStringSB ("Press a key to start: ", 100);
    GetChar();
    Newline();

    for (;;) { /* do forever */
      // Generate random number (check rounds to generate random number ie make it harder as time goes on)
      int RNG = (RNG_next() >> 30) + 1; // get value from 1 to 4
      RNG += RoundNum; // make harder as rounds increase
      float targetSpeed = ((float) RNG) / 4; // put into range (0.25, 1.25) m/s, with +0.25 extra m/s target

      // Put Random number to terminal and tell the player they have to move that fast with the car
      PutStringSB("You must move the cart at ", 100);
      PutFloat(targetSpeed);
      PutStringSB(" m/s!\r\n", 100);

      // During the round every 1 second print the current velocity of the car to the terminal

      UInt32 startTimerTick = PIT_Get_Ticks();
      #define ROUND_SECONDS (10)
      UInt32 endTimerTick = startTimerTick + (ROUND_SECONDS * PIT_TICKS_PER_SECOND);
      UInt32 success;
      while (1) {
        float currVel = Accel_Get_Velocity();
        if (currVel >= targetSpeed) {
          success = 1;
          break;
        }

        UInt32 currTimerTick = PIT_Get_Ticks();
        UInt32 secondsRemaining = (endTimerTick + (PIT_TICKS_PER_SECOND - 1) - currTimerTick) / PIT_TICKS_PER_SECOND;
        if (secondsRemaining == 0) {
          success = 0;
          break;
        }
        PutStringSB("Seconds remaining: ", 100);
        PutNumUB(secondsRemaining);
        PutStringSB(", velocity: ", 100);
        PutFloat(currVel);
        // Spacing here counteracts characters left over from
        // possible shorter messages as time runs out:
        PutStringSB(" m/s            ", 100);
        PutChar('\r');
      }
      if (success) {
        Points += RNG;
        PutStringSB("\nGood job! You gained ", 100);
        PutNumU(RNG);
        PutStringSB(" points, for a total of ", 100);
        PutNumU(Points);
        PutStringSB(".\r\n", 100);
      } else {
        LifePoints--;
        PutStringSB("\nYou lost the round!", 100);
        if (LifePoints == 0) {
          PutStringSB(".\r\n", 100);
          break;
        } else {
          PutStringSB(" You have ", 100);
          PutNumU(LifePoints);
          PutStringSB(" life points remaining.\r\n", 100);
        }
      }

      RoundNum++;

      PIT_Wait_Seconds(5);

      Accel_Reset_Velocity();

      // If the timer runs out prompt the player to play again and start from press a key to play 
      // If the player gets it correct prompt them to move on to the next round for 5 rounds total 
    } /* do forever */

    PutStringSB("Game over! Your score was: ", 100);
    PutNumU(Points);
    PutStringSB(" points.\r\nPress any key to play again: ", 100);

		// Empty UART receive queue
    while (RxQRecord[17] /* num enqd */) GetChar();

		// Wait for user to press key
    GetChar();
    PutStringSB("\r\n", 100);
  }
} /* main */
