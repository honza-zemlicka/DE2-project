#ifndef ULTRASOUND_H
#define ULTRASOUND_H

/** 
 * @brief Ultrasound Library for HY-SRF05
 * Dependencies: gpio.h, avr/io.h
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
 * Formula for calculation: (Time (us) * 10) / 58 = distance (mm)
 ** To travel 1 cm, sound needs 1 / 0.0343 = 29.15 us * 2 = 58.3 us per cm
 * @return Distance in mm. Returns 0 if out of range or timeout.
 */
uint16_t ultrasound_read(void);

#endif // ULTRASOUND_H