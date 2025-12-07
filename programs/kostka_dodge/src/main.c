// -Eppur si muove-  Obstacle Avoidance v03
// The robot drives straight. If it sees an obstacle, it bypasses it "blindly" using timing.
// Added UART debugging for distance measurement.

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h> // Needed for itoa
#include "timer.h"
#include <gpio.h>
#include <ultrasound.h>
#include <uart.h>

// --- PIN CONFIGURATION ---
#define MOTOR_L PD5   // Left wheel (PWM)
#define MOTOR_R PD6   // Right wheel (PWM)
#define USER_LED PB5  // Signaling LED

// --- SPEED CONFIGURATION (0-255) ---
#define SPEED_FORWARD   70   // Normal forward driving
#define SPEED_TURN_HIGH 100  // Faster wheel during turn
#define SPEED_TURN_LOW  0    // Slow/stopped wheel during turn
#define OBSTACLE_DIST   150  // Reaction distance in mm (20 cm)

volatile uint8_t count = 0;

/**
 * @brief Helper function to print integer over UART.
 */
static void uart_print_uint16(uint16_t value)
{
    char buf[10];
    itoa(value, buf, 10); // integer to string in base 10
    uart_puts(buf);
}

// Function for obstacle avoidance maneuver
void dodge_object(void)
{
    // 1. Signalization - LED ON
    PORTB |= (1 << USER_LED);

    // 2. Evasive maneuver - Turn RIGHT
    // Left wheel drives, right stops -> sharp right turn
    pwm_write(&PORTD, MOTOR_L, SPEED_TURN_HIGH);
    pwm_write(&PORTD, MOTOR_R, SPEED_TURN_LOW);
    _delay_ms(700); // Turn duration (adjust based on how much you want to turn)

    // 3. Bypassing obstacle - Drive STRAIGHT / ARC
    // Both wheels drive, robot passes the obstacle
    pwm_write(&PORTD, MOTOR_L, SPEED_FORWARD);
    pwm_write(&PORTD, MOTOR_R, SPEED_FORWARD);
    _delay_ms(1200); // Duration of driving alongside the obstacle

    // 4. Return to direction - Turn LEFT
    // Left wheel stops, right drives -> sharp left turn back
    pwm_write(&PORTD, MOTOR_L, SPEED_TURN_LOW);
    pwm_write(&PORTD, MOTOR_R, SPEED_TURN_HIGH);
    _delay_ms(700); // Duration of straightening direction

    // 5. End of maneuver - LED OFF
    // No need to stop, main loop continues with straight driving immediately
    PORTB &= ~(1 << USER_LED);
}

int main(void)
{
    // Initialization
    gpio_mode_output(&DDRD, MOTOR_L);
    gpio_mode_output(&DDRD, MOTOR_R);
    gpio_mode_output(&DDRB, USER_LED);

    // UART Init (9600 baud)
    uart_init(UART_BAUD_SELECT(9600, F_CPU));

    ultrasound_init();
    pwm_init();
    
    // Timer setup (kept from original code)
    tim2_ovf_16ms();
    tim2_ovf_enable();
    sei(); 

    // Initial start
    pwm_write(&PORTD, MOTOR_L, 0);
    pwm_write(&PORTD, MOTOR_R, 0);
    _delay_ms(1000); // Short pause after reset

    while (1)
    {
        // 1. Measure distance
        uint16_t distance = ultrasound_read();

        // 2. UART Logging
        if (distance == 0)
        {
            uart_puts("Error: Out of range\r\n");
        }
        else
        {
            // mm to cm conversion for display
            uint16_t cm_whole = distance / 10;
            uint16_t cm_dec   = distance % 10;

            uart_puts("Dist: ");
            uart_print_uint16(cm_whole);
            uart_puts(".");
            uart_print_uint16(cm_dec);
            uart_puts(" cm\r\n");
        }

        // 3. Control logic
        if (distance > 0 && distance < OBSTACLE_DIST)
        {
            // OBSTACLE: Perform maneuver
            uart_puts("Obstacle detected! Avoiding...\r\n");
            dodge_object();
            
            // After function completion, robot returns to "else" branch (drive straight) 
            // in next loop iteration, unless another obstacle is immediately ahead.
        }
        else
        {
            // CLEAR: Drive straight
            pwm_write(&PORTD, MOTOR_L, SPEED_FORWARD);
            pwm_write(&PORTD, MOTOR_R, SPEED_FORWARD);
        }

        // Necessary delay for ultrasound sensor (to let echo fade)
        _delay_ms(60);
    }
    return 0;
}

ISR(TIMER2_OVF_vect)
{
    count++;
}