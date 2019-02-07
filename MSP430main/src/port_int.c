#include "port_int.h"
#include "msp430f5529.h"

// Port_1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
    #pragma vector = PORT1_VECTOR
    __interrupt void Port_1(void)
#elif defined(__GNUC__)
    void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
    #error Compiler not supported!
#endif
{
//    // Brd_Key_S2: P1.1
//    if(P1IFG & BIT1)
//    {
//        ;
//        
//        P1IFG &= ~BIT1;
//    }
}

// Port_2
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
    #pragma vector = PORT2_VECTOR
    __interrupt void Port_2(void)
#elif defined(__GNUC__)
    void __attribute__ ((interrupt(PORT1_VECTOR))) Port_2 (void)
#else
    #error Compiler not supported!
#endif
{
//    // Brd_Key_S1: P2.1
//    if(P2IFG & BIT1)
//    {
//        ;
//        
//        P2IFG &= ~BIT1;
//    }
}