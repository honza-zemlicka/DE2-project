/*
 * Testing app for ultra sound sensor for robot line follower.
 * Target: AVR ATmega328P
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "uart.h"
#include "ultrasound.h"

/**
 * @brief Helper function to print integer over UART.
 */
static void uart_print_uint16(uint16_t value)
{
    char buf[10];
    itoa(value, buf, 10); // integer to string in base 10
    uart_puts(buf);
}

int main(void)
{
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    sei();

    ultrasound_init();

    while (1)
    {
        // measure distance in mm
        uint16_t distance_mm = ultrasound_read();

        if (distance_mm == 0)
        {
            uart_puts("Error: Out of range\r\n");
        }
        else
        {
            // mm to cm conversion
            uint16_t cm_whole = distance_mm / 10;
            uint16_t cm_dec   = distance_mm % 10;

            uart_puts("Dist: ");
            uart_print_uint16(cm_whole);
            uart_puts(".");
            uart_print_uint16(cm_dec);
            uart_puts(" cm\r\n");
        }

        // wait 100ms before next measurement
        _delay_ms(100);
    }

    return 0;
}