#ifndef GPIO_H
#define GPIO_H

/* 
 * GPIO library for AVR-GCC.
 * (c) 2019-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel ACVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

/**
 * @file 
 * @defgroup fryza_gpio GPIO Library <gpio.h>
 * @code #include <gpio.h> @endcode
 *
 * @brief GPIO library for AVR-GCC.
 *
 * The library contains functions for controlling AVRs' gpio pin(s).
 *
 * @note Based on AVR Libc Reference Manual.
 * @copyright (c) 2019-2025 Tomas Fryza, MIT license
 * @{
 */

// -- Includes ---------------------------------------------
#include <avr/io.h>


// -- Function prototypes ----------------------------------
/**
 * @brief  Configure one output pin.
 * @param  reg Address of Data Direction Register, such as &DDRB
 * @param  pin Pin designation in the interval 0 to 7
 * @return none
 */
void gpio_mode_output(volatile uint8_t *reg, uint8_t pin);


/**
 * @brief  Configure one input pin and enable pull-up.
 * @param  reg Address of Data Direction Register, such as &DDRB
 * @param  pin Pin designation in the interval 0 to 7
 * @return none
 */
void gpio_mode_input_pullup(volatile uint8_t *reg, uint8_t pin);


/**
 * @brief  Write one pin to low value.
 * @param  reg Address of Port Register, such as &PORTB
 * @param  pin Pin designation in the interval 0 to 7
 * @return none
 */
void gpio_write_low(volatile uint8_t *reg, uint8_t pin);


/**
 * @brief  Write one pin to high value.
 * @param  reg Address of Port Register, such as &PORTB
 * @param  pin Pin designation in the interval 0 to 7
 * @return none
 */
void gpio_write_high(volatile uint8_t *reg, uint8_t pin);


/**
 * @brief  Read a value from input pin.
 * @param  reg Address of Pin Register, such as &PIND
 * @param  pin Pin designation in the interval 0 to 7
 * @return Pin value
 */
uint8_t gpio_read(volatile uint8_t *reg, uint8_t pin);


/**
* @brief    Configure one input pin without pull-up resistor.  
* @param    reg Address of Data Direction Register, such as &DDRB 
* @param    pin Pin designation in the interval 0 to 7
* @return   none
*/
void gpio_mode_input_nopull(volatile uint8_t *reg, uint8_t pin);

/**
* @brief    Toggle one output pin value.
* @param    reg Address of Port Register, such as &PORTB
* @param    pin Pin designation in the interval 0 to 7
* @return   none
*/
void gpio_toggle(volatile uint8_t *reg, uint8_t pin);

/**
* @brief    Initialize ADC.
* @return   none
*/
void adc_init();

/**
* @brief    Read an analog value from a specific channel.
* @param    pin Analog channel designation in the interval 0 to 5
* @return   10-bit digital value
*/
uint16_t analog_read(uint8_t pin);

/**
* @brief    Initialize PWM generator on Timer0 and Timer1.
* @return   none
*/
void pwm_init();

/**
* @brief    Write PWM to specific pin.
* @param    reg Address 
* @param    pin Pin designation in the interval 0 to 7
* @param    val PWM duty cycle (0 to 255)
* @return   none
*/
void pwm_write(volatile uint8_t *reg, uint8_t pin, uint8_t val);


/** @} */

#endif