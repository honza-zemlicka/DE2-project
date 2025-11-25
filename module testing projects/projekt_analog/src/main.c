#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdlib.h>         // C library. Needed for number conversions
#include <gpio.h>

volatile uint8_t flag_update_uart = 0;
uint16_t analog_value0, analog_value1, analog_value2, analog_value3;

int main(void)
{
    // Initialize USART to asynchronous, 8-N-1, 9600 Bd
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    char uart_msg[16];
    tim1_ovf_1sec();
    tim1_ovf_enable();
    sei();

    adc_init();

    flag_update_uart = 1;

    while (1)
    {
      if(flag_update_uart == 1)
      {
        sprintf(uart_msg, "Analog: %u - %u - %u - %u\n\r", analog_value0, analog_value1, analog_value2, analog_value3);
        uart_puts(uart_msg);
        flag_update_uart = 0;
      }
    }
    return 0;
}

ISR(TIMER1_OVF_vect)
{
  analog_value0 = analog_read(PC0);
  analog_value1 = analog_read(PC1);
  analog_value2 = analog_read(PC2);
  analog_value3 = analog_read(PC3);
  uart_puts("AD prevod dokoncen");
  flag_update_uart = 1;
}

// 10-bit číslo = 1024

/*void adc_init()
{
  ADMUX = (1 << REFS0);   // ref, 3V3 / 5V
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // ADC ON, prescaler 128
}

uint16_t gpio_analog_read(uint8_t pin)
{
  ADMUX = (ADMUX & 0xF0) | (pin & 0x0F);    // select channel
  ADCSRA |= (1 << ADSC);          // start conversion
  while (ADCSRA & (1 << ADSC));   // wait until conversion is done

  return ADC;     // return 10-bit value
}*/
