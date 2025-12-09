#ifndef ULTRASOUND_H
#define ULTRASOUND_H

/**
 * @file 
 * @code #include <ultrasound.h> @endcode
 *
 * @brief Library for controlling HY-SRF05 / HC-SR04 ultrasonic sensors.
 *
 * The library contains functions for initializing the sensor and measuring 
 * distance.
 *
 * @note Dependencies: gpio.h, avr/io.h, util/delay.h
 * @{
 */


#include <stdint.h>
#include <avr/io.h>

#define US_TRIG_DDR  DDRB
#define US_TRIG_PORT PORTB
#define US_TRIG_PIN  PB0

#define US_ECHO_DDR  DDRD
#define US_ECHO_PINR PIND
#define US_ECHO_PIN  PD7

// --- Function Prototypes ---
/**
 * @brief Initialize the ultrasonic sensor pins.
 * Sets TRIG as output and ECHO as input.
 */
void ultrasound_init(void);

/**
 * @brief Distance measurement.
 * Generates a trigger pulse and measures the response duration.
 * @return Distance in mm. Returns 0 if out of range or timeout.
 */
uint16_t ultrasound_read(void);

#endif // ULTRASOUND_H