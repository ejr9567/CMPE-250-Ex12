#include "PIT_Driver.h"
#include "MKL05Z4.h"
#include "common.h"

static PIT_Handler_Type pit_handler;

// Set up PIT for 0.01s interrupts; interrupt handler will call PIT handler function
void Init_PIT(PIT_Handler_Type handler) {
    pit_handler = handler;

    // enable clock for PIT module
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    // disable PIT timer 0
    PIT->CHANNEL[0].TCTRL &= ~PIT_TCTRL_TEN_MASK;

    #define PIT_IPR (NVIC->IP[5])
    #define PIT_PRI_POS (24 - __NVIC_PRIO_BITS)
    #define NVIC_IPR_PIT_MASK (3 << PIT_PRI_POS)
    // set interrupt priority (0)
    PIT_IPR &= NVIC_IPR_PIT_MASK;

    #define PIT_IRQ_MASK (1 << PIT_IRQn)
    #define NVIC_ICPR_PIT_MASK (PIT_IRQ_MASK)
    #define NVIC_ISER_PIT_MASK (PIT_IRQ_MASK)
    // clear pending PIT interrupt
    NVIC->ICPR[0] = NVIC_ICPR_PIT_MASK;
    // unmask PIT interrupt
    NVIC->ISER[0] = NVIC_ISER_PIT_MASK;

    #define PIT_LDVAL_10ms (239861)
    // #define PIT_LDVAL_1ms (23986)
    #define PIT_MCR_EN_FRZ (PIT_MCR_FRZ_MASK)
    #define PIT_TCTRL_CH_IE (PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK)

    PIT->MCR = PIT_MCR_EN_FRZ;
    PIT->CHANNEL[0].LDVAL = PIT_LDVAL_10ms;
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_CH_IE;
}

volatile UInt32 Seconds_Counter = 0;
volatile UInt32 Ticks_Counter = 0; //0.01 s to 1s converter
volatile UInt32 Consistent_Ticks_Counter = 0;
void __attribute__((interrupt("IRQ"))) PIT_IRQHandler(void) {
    __asm(" CPSID   I");
    //Sample velocity
    pit_handler();
    Consistent_Ticks_Counter++;
    //loop below is for every 1 second output
    Ticks_Counter++;
    if (Ticks_Counter == 100) {
        Seconds_Counter++;
        Ticks_Counter = 0;
    }
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;
   __asm(" CPSIE   I");
}

UInt32 PIT_Get_Ticks() {
    return Consistent_Ticks_Counter;
}

void PIT_Wait_Ticks(UInt32 ticks) {
    UInt32 targetCount = Consistent_Ticks_Counter + ticks;
    while (Consistent_Ticks_Counter < targetCount);
}

void PIT_Wait_Seconds(UInt32 seconds) {
    PIT_Wait_Ticks(seconds * PIT_TICKS_PER_SECOND);
}
