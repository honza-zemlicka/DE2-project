/* 
 * GPIO library for AVR-GCC.
 * (c) 2019-2024 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and AVR 8-bit Toolchain 3.6.2.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes -------------------------------------------------------
#include <gpio.h>


// -- Function definitions -------------------------------------------
/*
 * Function: gpio_mode_output()
 * Purpose:  Configure one output pin.
 * Input(s): reg - Address of Data Direction Register, such as &DDRB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  none
 */
void gpio_mode_output(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg | (1<<pin);
}


/*
 * Function: gpio_mode_input_pullup()
 * Purpose:  Configure one input pin and enable pull-up.
 * Input(s): reg - Address of Data Direction Register, such as &DDRB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  none
 */
void gpio_mode_input_pullup(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);  // Data Direction Register
    reg++;                    // Change pointer to Data Register
    *reg = *reg | (1<<pin);   // Data Register
}


/*
 * Function: gpio_write_low()
 * Purpose:  Write one pin to low value.
 * Input(s): reg - Address of Port Register, such as &PORTB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  none
 */
void gpio_write_low(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);
}


/*
 * Function: gpio_write_high()
 * Purpose:  Write one pin to high value.
 * Input(s): reg - Address of Port Register, such as &PORTB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  none
 */
void gpio_write_high(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg | (1<<pin);
}


/*
 * Function: gpio_read()
 * Purpose:  Read a value from input pin.
 * Input(s): reg - Address of Pin Register, such as &PINB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  Pin value
 */
uint8_t gpio_read(volatile uint8_t *reg, uint8_t pin)
{
    uint8_t temp;

    temp = *reg & (1<<pin);

    if (temp != 0) {
        return 1;
    }
    else {
        return 0;
    }
}


/*
 * Function: gpio_mode_input_nopull()
 */
void gpio_mode_input_nopull(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);  // Data Direction Register
    reg++;                    // Change pointer to Data Register
    *reg = *reg & ~(1<<pin);  // Data Register
}


/*
 * Function: gpio_toggle()
 */
void gpio_toggle(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg ^ (1<<pin);
}

/*
 * Function: adc_init()
 */
void adc_init()
{
  ADMUX = (1 << REFS0);   // ref, 3V3 / 5V
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // ADC ON, prescaler 128
}

/*
 * Function: gpio_analog_read()
 */
uint16_t analog_read(uint8_t pin)
{
  ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);    // select channel
  ADCSRA |= (1 << ADSC);                    // start conversion
  while (ADCSRA & (1 << ADSC));             // wait until conversion is done

  return ADC;     // return 10-bit value
}
