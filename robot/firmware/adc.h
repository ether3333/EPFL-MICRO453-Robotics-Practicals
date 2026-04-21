#ifndef __ADC_H
#define __ADC_H

/**
 * \file   adc.h
 * \brief  Routines for accessing the A/D converter
 * \author Alessandro Crespi
 * \date   August 2009
 */

#include <stdint.h>

/// Initialises the A/D converter, sets all AN0-AN3 pins to analog, use AN0.
void adc_init(void);

/** \brief Selects a channel on the ADC.
 *  \param ch Input channel to select, should be between 0 and 3
 */
void adc_set_channel(uint8_t ch);

/** \brief  Starts a A/D conversion on the selected channel of the ADC.
 *  \return 10-bit result of the conversion
 */
uint16_t adc_read(void);

/** This function disables the Timer1 IRQ during the conversion, to ensure
 *  that users functions that would use the ADC could not be executed
 *  and thus perturb the conversion.
 *  \brief  Selects a channel on the ADC and starts a conversion on it.
 *  \param  ch Input channel to select, should be between 0 and 3
 *  \return 10-bit result of the conversion
 */
uint16_t adc_read_ch(uint8_t ch);

#endif // __ADC_H
