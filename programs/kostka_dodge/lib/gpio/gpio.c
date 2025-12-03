/* 
 * GPIO library for AVR-GCC.
 * (c) 2019-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel AVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes ---------------------------------------------
#include <gpio.h>


// -- Function definitions ---------------------------------
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
 * Function: gpio_mode_input()
 * Purpose:  Configure one input pin and enable pull-up.
 * Input(s): reg - Address of Data Direction Register, such as &DDRB
 *           pin - Pin designation in the interval 0 to 7
 * Returns:  none
 */
void gpio_mode_input(volatile uint8_t *reg, uint8_t pin)
{
    *reg = *reg & ~(1<<pin);  // Data Direction Register
    reg++;                    // Change pointer to Data Register
    *reg = *reg & ~(1<<pin);   // Data Register
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


/*
 * Function: pwm_init()
 */
void pwm_init()
{
    /*
    // ---TIMER 0---
    TCCR0A |= (1 << WGM01) | (1 << WGM00);     // Fast PWM
    TCCR0B |= (1 << CS01)  | (1 << CS00);      // Prescaler 64 (f ≈ 976 Hz)

    // ---TIMER 1---
    TCCR1A |= (1 << WGM11) | (1 << WGM12)
            | (1 << WGM13) | (1 << WGM10);     // Fast PWM 16-bit
    TCCR1B |= (1 << CS11);                     // Prescaler 8 (f ≈ 976 Hz)
    */

    // ---TIMER 0 (8-bit)---
    TCCR0A |= (1 << WGM01) | (1 << WGM00);     // Fast PWM
    TCCR0B |= (1 << CS01);                     // Prescaler 8 (f ≈ 7,8 kHz)

    // ---TIMER 1 (16-bit)---
    TCCR1A |= (1 << WGM11) | (1 << WGM12)
            | (1 << WGM13) | (1 << WGM10);     // Fast PWM 16-bit
    TCCR1B |= (1 << CS10);                     // Prescaler 1 (f ≈ 7,8 kHz)

  /*
  * f_pwm8 = 16M / (prescaler * 256)
  * prescaler 64 -> f_pwm ≈ 976 Hz
  * prescaler 8 -> f_pwm ≈ 7,8 kHz
  * 
  * * f_pwm8 = 16M / (prescaler * 2048)
  * prescaler 8 -> f_pwm ≈ 976 Hz
  * prescaler 1 -> f_pwm ≈ 7,8 kHz
  */

  // ---ENABLE PWM---
  TCCR0A |= (1 << COM0B1);  // PD6
  TCCR0A &= ~(1 << COM0B0);

  TCCR0A |= (1 << COM0A1);  // PD5
  TCCR0A &= ~(1 << COM0A0);

  TCCR1A |= (1 << COM1A1);  // PB1
  TCCR1A &= ~(1 << COM1A0);

  TCCR1A |= (1 << COM1B1);  // PB2
  TCCR1A &= ~(1 << COM1B0);
}


/*
 * Function: pwm_write()
 */
void pwm_write(volatile uint8_t *reg, uint8_t pin, uint8_t val)   // val must be in range 0 - 255
{
  if(val == 0)    // special case - 0
  {
    gpio_write_low(*reg, pin);
    return;
  }

  if(val == 255)    // special case - 255
  {
    gpio_write_high(*reg, pin);
    return;
  }

  switch(pin)
  {
    // ---TIMER 0---
    case PD5: //AR 5
        OCR0B = val;
        break;

    case PD6: //AR 6
        OCR0A = val;
        break;

    // ----TIMER 1---
    case PB1: // AR 9
        OCR1A = 1023/255*val;       // correction 8-bit to 16-bit
        break;

    case PB2:  // AR 10
        OCR1B = 1023/255*val;       // correction 8-bit to 16-bit
        break;
  }
}