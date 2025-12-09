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
#include "robot_definitions.h"

#define OBSTACLE_DIST 75 // Reaction distance in mm (20 cm)

volatile uint8_t count = 0;

// Function for obstacle avoidance maneuver
void dodge_object(void)
{
    PORTB |= (1 << USER_LED);

    // 2. Evasive maneuver - Turn RIGHT
    // Left wheel drives, right stops -> sharp right turn
    pwm_write(&PORTD, MOTOR_LF, 0);
    pwm_write(&PORTD, MOTOR_RF, 100);
    _delay_ms(1000); // Turn duration

    // 3. Bypassing obstacle - Drive STRAIGHT / ARC
    // Both wheels drive, robot passes the obstacle
    pwm_write(&PORTD, MOTOR_LF, 100);
    pwm_write(&PORTD, MOTOR_RF, 65);
    _delay_ms(3000); // Duration of driving alongside the obstacle

    // 4. Return to direction - Turn LEFT
    // Left wheel stops, right drives -> sharp left turn back
    pwm_write(&PORTD, MOTOR_LF, 10);
    pwm_write(&PORTD, MOTOR_RF, 100);
    _delay_ms(350); // Duration of straightening direction

    // 5. End of maneuver - LED OFF
    // No need to stop, main loop continues with straight driving immediately
    PORTB &= ~(1 << USER_LED);
}

int main(void)
{
    // Initialization
    gpio_mode_output(&DDRD, MOTOR_LF);
    gpio_mode_output(&DDRD, MOTOR_RF);
    gpio_mode_output(&DDRB, USER_LED);

    ultrasound_init();
    pwm_init();

    // Timer setup (kept from original code)
    tim2_ovf_16ms();
    tim2_ovf_enable();
    sei();

    // Initial start
    pwm_write(&PORTD, MOTOR_LF, 0);
    pwm_write(&PORTD, MOTOR_RF, 0);
    _delay_ms(1000); // Short pause after reset

    while (1)
    {
        uint16_t distance = ultrasound_read();

        // 3. Control logic
        if (distance > 0 && distance < 75)
        {
            // OBSTACLE: Perform maneuver
            // uart_puts("Obstacle detected! Avoiding...\r\n");
            dodge_object();

            // After function completion, robot returns to "else" branch (drive straight)
            // in next loop iteration, unless another obstacle is immediately ahead.
        }
        else
        {
            // CLEAR: Drive straight
            pwm_write(&PORTD, MOTOR_LF, 100);
            pwm_write(&PORTD, MOTOR_RF, 100);
        }

        // Necessary delay for ultrasound sensor (to let echo fade)
        //_delay_ms(60);
    }
    return 0;
}

ISR(TIMER2_OVF_vect)
{
    count++;
}