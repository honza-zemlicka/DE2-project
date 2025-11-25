#include <avr/io.h> // pro funkci
#include <stdint.h> // pro funkci
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/delay.h>
#include <gpio.h>
#include "timer.h"

int main(void)
{
  gpio_mode_output(&DDRD, PD5);
  gpio_mode_output(&DDRD, PD6);
  gpio_mode_output(&DDRB, PB1);
  gpio_mode_output(&DDRB, PB2);

/*
  // ---TIMER 0---
  TCCR0A |= (1 << WGM01) | (1 << WGM00);     // Fast PWM
  TCCR0B |= (1 << CS01)  | (1 << CS00);      // Prescaler 64 (f ≈ 976 Hz)

  // ---TIMER 1---
  TCCR1A = (1 << WGM10) | (1 << WGM11);      // Fast PWM
  TCCR1B |= (1 << CS11);                     // Prescaler 8 (f ≈ 976 Hz)

  // ---ENABLE PWM---
  TCCR0A |= (1 << COM0B1);  // PD6
  TCCR0A &= ~(1 << COM0B0);

  TCCR0A |= (1 << COM0A1);  // PD5
  TCCR0A &= ~(1 << COM0A0);

  TCCR1A |= (1 << COM1A1);  // PB1
  TCCR1A &= ~(1 << COM1A0);

  TCCR1A |= (1 << COM1B1);  // PB2
  TCCR1A &= ~(1 << COM1B0);
*/

  pwm_init();

  while(1)
  {
    uint8_t duty_val = 200; // max 255

    /*OCR0B = duty_val;
    OCR0A = duty_val;

    OCR1B = 1023/255*duty_val;
    OCR1A = 1023/255*duty_val;*/

    pwm_write(&PORTD, PD5, duty_val);
    pwm_write(&PORTD, PD6, duty_val);
    pwm_write(&PORTB, PB1, duty_val);
    pwm_write(&PORTB, PB2, duty_val);

  }
}

void pwm_init()
{
  /*
  // ---TIMER 0 (8-bit)---
  TCCR0A |= (1 << WGM01) | (1 << WGM00);     // Fast PWM
  TCCR0B |= (1 << CS01)  | (1 << CS00);      // Prescaler 8 (f ≈ 7,8 kHz)

  // ---TIMER 1 (16-bit)---
  TCCR1A |= (1 << WGM11) | (1 << WGM12)
          | (1 << WGM13) | (1 << WGM10);     // Fast PWM 8-bit
  TCCR1B |= (1 << CS11);                     // Prescaler 8 (f ≈ 3,9 kHz)
  */

  // ---TIMER 0 (8-bit)---
  TCCR0A |= (1 << WGM01) | (1 << WGM00);     // Fast PWM
  TCCR0B |= (1 << CS01);                     // Prescaler 8 (f ≈ 7,8 kHz)

  // ---TIMER 1 (16-bit)---
  TCCR1A |= (1 << WGM11) | (1 << WGM12)
          | (1 << WGM13) | (1 << WGM10);     // Fast PWM 16-bit
  TCCR1B |= (1 << CS10);                     // Prescaler 1 (f ≈ 7,8 kHz)
  



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
        OCR1A = 1023/255*val;   // correction 8-bit to 16-bit
        break;

    case PB2:  // AR 10
        OCR1B = 1023/255*val;   // correction 8-bit to 16-bit
        break;
  }

}

// ---TIMER 2---
    /*case PD3: // AR 3
        TCCR2A |= (1 << COM2B1);
        TCCR2A &= ~(1 << COM2B0);
        OCR2B = val;
        break;

    case PB3:  // AR 11
        TCCR2A |= (1 << COM2A1);
        TCCR2A &= ~(1 << COM2A0);
        OCR2A = val;
        break;*/