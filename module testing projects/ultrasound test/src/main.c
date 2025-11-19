#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

#include "uart.h"
#include "ultrasonic.h"

static void uart_print_uint16(uint16_t value)
{
    char buf[10];
    itoa(value, buf, 10);
    uart_puts(buf);
}

int main(void)
{
    // UART 9600 baud (16 MHz clock)
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    sei();

    ultrasonic_init();

    while (1)
    {
        float distance_cm_f = ultrasonic_read();

        // If 0.0 => timeout / no echo
        if (distance_cm_f == 0.0f)
        {
            uart_puts("No echo / out of range\r\n");
        }
        else
        {
            // rounding
            uint16_t distance_cm = (uint16_t)(distance_cm_f);

            uart_puts("Distance: ");
            uart_print_uint16(distance_cm);
            uart_puts(" cm\r\n");
        }

        // delay between measurements so echoes don't overlap
        _delay_ms(100);
    }

    return 0;
}
