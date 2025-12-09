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

uint16_t S1, S2, S3, S4;

uint16_t S1K[2] = {800, 100};
uint16_t S2K[2] = {800, 100};
uint16_t S3K[2] = {800, 100};
uint16_t S4K[2] = {800, 100};

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);


volatile uint8_t count = 0;

void dodge_object(void)
{
    // Rozsvítit LED - signalizace manévru
    gpio_write_high(&PORTB, USER_LED);

    // 1. Uhnutí vpravo (90 stupňů)
    // Levé kolo jede, pravé stojí -> ostrá zatáčka
    pwm_write(&PORTD, MOTOR_LF, 10);
    pwm_write(&PORTD, MOTOR_RF, 100);
    _delay_ms(1000); // Čas pro otočení cca 90° (nutno odladit v praxi)

    // 2. Objíždění překážky (oblouk) + čekání na čáru
    // Jede v oblouku tak dlouho, dokud nenajede zpět na čáru
    do
    {
        pwm_write(&PORTD, MOTOR_LF, 100);
        pwm_write(&PORTD, MOTOR_RF, 75);
        
        //S1 = map(analog_read(SENSOR_CR), S1K[1], S1K[0], 0, 100);
        //S2 = map(analog_read(SENSOR_CL), S2K[1], S2K[0], 0, 100);

        S1 = analog_read(SENSOR_CR);
        S2 = analog_read(SENSOR_CL);

    } while ((S2 > 300) || (S1 > 300));

    _delay_ms(200);

    // 3. Srovnání do směru čáry (otočení vlevo)
    // Levé kolo pomalu, pravé rychle -> srovnání
    pwm_write(&PORTD, MOTOR_LF, 10);
    pwm_write(&PORTD, MOTOR_RF, 100);
    _delay_ms(350); // Čas na srovnání (nutno odladit)

    // Zhasnout LED
    gpio_write_low(&PORTB, USER_LED);
}

int main(void)
{
    // Initialization
    gpio_mode_output(&DDRD, MOTOR_LF);
    gpio_mode_output(&DDRD, MOTOR_RF);
    gpio_mode_output(&DDRB, USER_LED);

    gpio_mode_input(&DDRC, SENSOR_CR);
    gpio_mode_input(&DDRC, SENSOR_CL);
    gpio_mode_input(&DDRC, SENSOR_RR);
    gpio_mode_input(&DDRC, SENSOR_LL);

    ultrasound_init();
    pwm_init();
    adc_init();
    pwm_write(&PORTD, MOTOR_LF, 0);
    pwm_write(&PORTD, MOTOR_RF, 0);


    // Timer setup (kept from original code)
    tim2_ovf_16ms();
    tim2_ovf_enable();
    sei();

    _delay_ms(1000); // Short pause after reset

    while (1)
    {
        uint16_t distance = ultrasound_read();

        // 3. Control logic
        if (distance > 0 && distance < 75) 
        {
         dodge_object(); // Zavolá vyhýbací manévr
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

static inline int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (int32_t)(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

ISR(TIMER2_OVF_vect)
{
    count++;
}