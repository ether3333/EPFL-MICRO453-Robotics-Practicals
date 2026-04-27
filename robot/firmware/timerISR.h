#ifndef __TIMER_ISR_H
#define __TIMER_ISR_H

/**
 * \file   timerISR.h
 * \brief  Timer1 support for defining interrupt-started periodic user routines
 * \author Alessandro Crespi
 * \date   August 2009
 */

/// Maximum number of user-defined functions, keep as low as needed to
/// avoid using unneeded memory.
#define MAX_TIMER1_USER_FUNCTIONS 4

/// Function type for user functions.
typedef void (*TimerFunction)(void);

/** \brief Initializes Timer1 configuration.
 *  \param match_count Timer period in microseconds
 */
void timer1_init(uint32_t match_count);

/// Installs the Timer1 interrupt service routine.
void timer1_init_isr(void);

/** \brief Adds a user function to the table.
  * \param tf Timer function to be called. It should be a simple void
  *   function without any parameters (see ::TimerFunction).
  * \param period Period (in timer interrupts) of the user routine. For
  *   example, if the timer has been initialised with init_timer1(50),
  *   and period is 20, the function will be called every millisecond.
  */
void timer1_add_user_function(TimerFunction tf, uint16_t period);

/** \brief Removes the specified function from the table.
 *  \param tf Timer function to be removed.
 */
void timer1_remove_user_function(TimerFunction tf);

/** \brief Disables the match interrupt for Timer1, thus preventing user
 *    functions to be started.
 *  \return Previous state of the Timer1 Match Control Register, to be used
 *    later with ::restore_timer1_irq().
 */
uint32_t disable_timer1_irq(void);

/** \brief Enables the match interrupt for Timer1.
 *  \return Previous state of the Timer1 Match Control Register, to be used
 *    later with ::restore_timer1_irq().
 */
uint32_t enable_timer1_irq(void);

/** \brief Restores the Timer1 match interrupt state from the corresponding bit.
 *  \param  flag Match control register. Only the ::TMCR_MR0_I bit will be used.
 *  \return Previous state of the Timer1 Match Control Register.
 */
uint32_t restore_timer1_irq(uint32_t flag);

#endif // __TIMER_ISR_H
