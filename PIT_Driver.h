#include "common.h"

// For each tick = 10 ms, 1000 ms/second / 10 ms/tick = 100 ticks/second
#define PIT_TICKS_PER_SECOND (100)

typedef void (*PIT_Handler_Type)(void);

void Init_PIT(PIT_Handler_Type handler);
UInt32 PIT_Get_Ticks();
void PIT_Wait_Ticks(UInt32 ticks);
void PIT_Wait_Seconds(UInt32 seconds);
