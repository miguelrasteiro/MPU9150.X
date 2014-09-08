/* 
 * File:   hardware.h
 * Author: MiguelRasteiro
 *
 * Created on 25 de Agosto de 2014, 17:04
 */

#ifndef HARDWARE_H
#define	HARDWARE_H


#define SYS_FREQ (80000000L)                                       /**< CPU clock frequency */
#define FOSC       SYS_FREQ                                        /**< CPU clock frequency */
#define	GetPeripheralClock()	(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()	(SYS_FREQ)                         /**< Instructions frequency */
#define PBCLK       SYS_FREQ/4
#define Fsck        375000
#define BRG         400000      /**< I2C frequency FastMode = 400kHz */
#define UARTBAUDRATE 230400
        // For Timers
#define ONE_SECOND                    (FOSC/2)                  // 1s of PIC32 core timer ticks (== Hz)
#define MS_TO_CORE_TICKS(x)   ((UINT64)(x)*ONE_SECOND/1000)
#define uS_TO_CORE_TICKS(x)   ((UINT64)(x)*ONE_SECOND/1000000)
#define CT_TICKS_SINCE(tick)   (ReadCoreTimer() - (tick))	// number of core timer ticks since "tick"

#define TIMER_1_INT_VECTOR      4                                       /**< Interruption Vector For timer1 */
#define EXTERNAL_2_INT_VECTOR  11                                       /**< Interruption Vector For external interrupt 2*/

#define MPU_I2C I2C1



#endif	/* HARDWARE_H */

