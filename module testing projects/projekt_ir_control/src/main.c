#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <uart.h>
#include <gpio.h>

#define frame_length 32

volatile uint32_t ir_data = 0;
volatile uint8_t bit_index = 0;
volatile uint32_t frame_received = 0;
volatile uint8_t rx_bytes = 0;

volatile uint8_t last_time = 0;
volatile uint8_t measured = 0;
volatile uint8_t length = 0;
volatile uint8_t reception_complete = 0;

// 32-bit + start bit

void timer2_init()
{
    TCCR2A = 0;                                        // normal mode
    //TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024, 64us
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler 1024, 64us
    TCNT2 = 0;
}

void int0_init()
{
    EICRA = (1 << ISC01) | (1 << ISC00); // INT0 = rising edge
    EIMSK = (1 << INT0);                 // enable INT0
}

ISR(INT0_vect)
{
    uint8_t measured = TCNT2;
    uint8_t length = measured - last_time;

    // Start bit
    if (length > 60 && length < 100) // 5 062 us / 64 = 79 tics
    {
        ir_data = 0;
        bit_index = 0;
        frame_received = 0;
    }

    // High
    if (length > 25 && length < 45) // 2 249 us / 64 = 35 tics
    {
        // ir_data |= (1 << bit_index);
        ir_data <<= 1;
        ir_data |= 1;
        bit_index++;
    }

    // Low
    if (length > 7 && length < 25) // 1 124 us / 64 = 17,5 tics
    {
        // ir_data |= (0 << bit_index);
        ir_data <<= 1;
        bit_index++;
    }

    if (bit_index == frame_length)
    {
        EIMSK &= ~(1 << INT0); // disable INT0
        char uart_msg[32];
        rx_bytes++;
        //sprintf(uart_msg, "Received(%d): %08lX\n\r", rx_bytes, ir_data);
        //uart_puts(uart_msg);
        gpio_toggle(&PORTB, PB5);

        reception_complete = 1;
        TCNT2 = 0;
        return;
    }
    last_time = measured;
}

int main(void)
{
    //uart_init(UART_BAUD_SELECT(9600, F_CPU));
    char uart_msg[8];
    timer2_init();
    int0_init();
    sei();

    //sprintf(uart_msg, "IR remote control\n\r");
    //uart_puts(uart_msg);

    uint8_t delay_ir = 0;

    gpio_mode_output(&DDRB, PB5);

    while (1)
    {
        if (reception_complete == 1)
        {
            // NEC format:
            uint8_t address = (ir_data >> 24) & 0xFF;     // [31..24] Address
            uint8_t address_inv = (ir_data >> 16) & 0xFF; // [23..16] Address_inv
            uint8_t command = (ir_data >> 8) & 0xFF;      // [15..8]  Command
            uint8_t command_inv = (ir_data >> 0) & 0xFF;  // [7..0]   Command_inv

            // Validity check
            if ((command ^ command_inv) == 0xFF)
            {
                //sprintf(uart_msg, "Command: %x\n\r", command);
                //uart_puts(uart_msg);
            }

            reception_complete = 0;
            //TCNT2 = 0;
            delay_ir = 0;
        }

        if (TIFR2 & (1 << TOV2)) // Timer2 overflow occurred
        {
            delay_ir++;
            TIFR2 |= (1 << TOV2); // clear flag by writing 1
        }

        if (delay_ir == 3)
        {
            EIFR |= (1 << INTF0); // erase interrupt flag
            EIMSK = (1 << INT0);  // enable INT0
        }
    }
}

/*

// Validity check
if ((address ^ address_inv) == 0xFF &&
    (command ^ command_inv) == 0xFF)
{
}
*/

/*
maly ovladac, tlacitko G

0000 0000 1111 0111 1010 0000 0101 1111
  0    0    f    7    a    0    5    f

00-f7-a0-5f
*/
