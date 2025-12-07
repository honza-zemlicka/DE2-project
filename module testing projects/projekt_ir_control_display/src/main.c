#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <uart.h>
#include <gpio.h>
#include <oled.h>

#define frame_length 32
// 32-bit + start bit

#define RUN 0x00
#define STOP 0x80
#define CALIB 0x40
//#define SET 0xc0

#define GAIN_PLUS_1 0x10
#define GAIN_PLUS_10 0x50
#define GAIN_MINUS_1 0x90
#define GAIN_MINUS_10 0xd0

#define SPEED_PLUS_1 0x08
#define SPEED_PLUS_10 0x48
#define SPEED_MINUS_1 0x88
#define SPEED_MINUS_10 0xc8

uint8_t motor_speed = 255;
uint8_t gain_kp = 10;
char display_buffer[16];

uint8_t robot_run = 0;

volatile uint32_t ir_data = 0;
volatile uint8_t bit_index = 0;
volatile uint32_t frame_received = 0;
volatile uint8_t rx_bytes = 0;

volatile uint8_t last_time = 0;
volatile uint8_t measured = 0;
volatile uint8_t length = 0;
volatile uint8_t reception_complete = 0;

const char *encode_cmd(uint8_t command);

const char *message;

void update_display()
{
    oled_gotoxy(17, 0);
    sprintf(display_buffer, "%2d", gain_kp);
    oled_puts(display_buffer);

    oled_gotoxy(15, 2);
    sprintf(display_buffer, "%3d", motor_speed);
    oled_puts(display_buffer);

    oled_display();
}

void timer2_init()
{
    TCCR2A = 0; // normal mode
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
        rx_bytes++;
        gpio_toggle(&PORTB, PB5);

        TCNT2 = 0;
        reception_complete = 1;
        return;
    }
    last_time = measured;
}

int main(void)
{
    timer2_init();
    int0_init();
    sei();

    twi_init();
    oled_init(OLED_DISP_ON);
    oled_clrscr();
    oled_charMode(DOUBLESIZE);

    oled_gotoxy(0, 0);
    oled_puts("Kp    =");
    oled_gotoxy(0, 2);
    oled_puts("Speed =");
    update_display();

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
                const char *msg = encode_cmd(command);
                update_display();              
            }

            reception_complete = 0;
            // TCNT2 = 0;
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
maly ovladac, tlacitko G

0000 0000 1111 0111 1010 0000 0101 1111
  0    0    f    7    a    0    5    f

00-f7-a0-5f
*/

const char *encode_cmd(uint8_t command)
{
    switch (command)
    {
    // ROBOT COMMANDS
    
        case RUN:
        return robot_run = 1;

    case STOP:
        return robot_run = 0;

    /*case CALIB:
        return calibration();*/

    // DISPLAY COMMANDS

    case GAIN_PLUS_1:
        return gain_kp+=1;

    case GAIN_MINUS_1:
        return gain_kp-=1;

    case GAIN_PLUS_10:
        return gain_kp+=10;

    case GAIN_MINUS_10:
        return gain_kp-=10;

    case SPEED_PLUS_1:
        return motor_speed+=1;

    case SPEED_MINUS_1:
        return motor_speed-=1;

    case SPEED_PLUS_10:
        return motor_speed+=10;

    case SPEED_MINUS_10:
        return motor_speed-=10;
    }
}
