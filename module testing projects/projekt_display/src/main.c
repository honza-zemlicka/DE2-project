/*
 * The I2C (TWI) bus scanner tests all addresses and detects devices
 * that are connected to the SDA and SCL signals.
 * (c) 2023-2025 Tomas Fryza, MIT license
 *
 * Developed using PlatformIO and Atmel AVR platform.
 * Tested on Arduino Uno board and ATmega328P, 16 MHz.
 */

// -- Includes ---------------------------------------------
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <twi.h>            // I2C/TWI library for AVR-GCC
#include <stdio.h>          // C library. Needed for `sprintf`
#include "timer.h"
#include <oled.h>
#define RTC_ADR 0x68

// -- Global variables -------------------------------------
volatile uint8_t rtc_values[2];

// -- Function definitions ---------------------------------
/*
 * Function: Main function where the program execution begins
 * Purpose:  Call function to test all I2C (TWI) combinations and send
 *           detected devices to UART.
 * Returns:  none
 */


uint8_t motor_speed = 255;
uint8_t gain_kp = 10;
char display_buffer[16];

void update_display() {
    oled_gotoxy(17, 0); 
    sprintf(display_buffer, "%2d", gain_kp);
    oled_puts(display_buffer);

    oled_gotoxy(15, 2);
    sprintf(display_buffer, "%3d", motor_speed);
    oled_puts(display_buffer);
    
    oled_display();
}

int main(void)
{
    twi_init();
    oled_init(OLED_DISP_ON);
    oled_clrscr();
    oled_charMode(DOUBLESIZE);

    oled_gotoxy(0, 0); 
    oled_puts("Kp =");
    oled_gotoxy(0, 2); 
    oled_puts("Speed =");
    update_display();

    // Will never reach this
    return 0;
}