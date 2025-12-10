#ifndef ULTRASOUND_H
#define ULTRASOUND_H

/*
 * Ultrasound Library for HY-SRF05
 * Dependencies: gpio.h
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
 * @brief Measure distance and return it in millimeters.
 * @return Distance in mm (uint16_t). Returns 0 if out of range or timeout.
 */
uint16_t ultrasound_read(void);

#endif // ULTRASOUND_H