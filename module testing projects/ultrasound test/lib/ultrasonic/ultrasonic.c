#include "ultrasonic.h"
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <gpio.h>

#define TRIG_DDR   DDRD
#define TRIG_PORT  PORTD
#define TRIG_PIN   PD3

#define ECHO_DDR   DDRD
#define ECHO_PINR  PIND
#define ECHO_PIN   PD4

static inline void delay_us(uint16_t us)
{
    while (us--) {
        _delay_us(1);
    }
}

static uint32_t pulseIn(uint32_t timeout_us)
{
    uint32_t width = 0;

    // čekání na náběžnou hranu (ECHO LOW -> HIGH)
    while (gpio_read(&ECHO_PINR, ECHO_PIN) == 0) {
        if (timeout_us-- == 0) {
            return 0;
        }
        _delay_us(1);
    }

    // measure H pulse
    while (gpio_read(&ECHO_PINR, ECHO_PIN) == 1) {
        width++;
        _delay_us(1);
        if (width >= timeout_us) {
            break;     // timeout
        }
    }

    return width;   // pulse duration in us
}


// pin init
void ultrasonic_init(void)
{
    // TRIG jako výstup
    gpio_mode_output(&TRIG_DDR, TRIG_PIN);

    // ECHO jako vstup bez pull-up (modul má svůj pull-down/pull-up)
    gpio_mode_input_nopull(&ECHO_DDR, ECHO_PIN);

    // safety TRIG to 0
    gpio_write_low(&TRIG_PORT, TRIG_PIN);
}


// measurement func
float ultrasonic_read(void)
{
    uint32_t time_us;
    float distance_cm;

    // trigger pulse: LOW 2 µs, HIGH 10 µs, pak zase LOW
    gpio_write_low(&TRIG_PORT, TRIG_PIN);
    _delay_us(2);
    gpio_write_high(&TRIG_PORT, TRIG_PIN);
    _delay_us(10);
    gpio_write_low(&TRIG_PORT, TRIG_PIN);

    // ECHO pulse duration (timeout např. 30 ms = 30000 µs)
    time_us = pulseIn(30000);

    if (time_us == 0) {
        return 0.0f;
    }

    // Přepočet na vzdálenost v cm:
    // rychlost zvuku ~343 m/s => cca 29.1 µs na 1 cm tam+zpět
    distance_cm = (float)time_us / 29.1;

    return distance_cm;
}
