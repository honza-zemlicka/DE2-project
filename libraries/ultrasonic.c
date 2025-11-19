#include <avr/io.h>
#include <util/delay.h>
//#include "ultrasonic.h"

#define TRIG_DDR   DDRD
#define TRIG_PORT  PORTD
#define TRIG_PIN   PD2

#define ECHO_DDR   DDRD
#define ECHO_PINR  PIND
#define ECHO_PIN   PD3

static inline void delay_us(uint16_t us)
{
    while(us--) _delay_us(1);
}


// measure H pulse
static uint32_t pulseIn(uint8_t mask, uint32_t timeout_us)
{
    uint32_t distance = 0;

    // wait for pulse start
    while ((ECHO_PINR & mask) == 0) {
        if (!timeout_us--) return 0;
        _delay_us(1);
    }

    // measure H pulse duration
    while (ECHO_PINR & mask) {
        distance++;
        _delay_us(1);
        if (distance >= timeout_us) break;
    }

    return distance; // 
}


// pin init
void ultrasonic_init(void)
{
    TRIG_DDR  |=  (1 << TRIG_PIN);

    ECHO_DDR  &= ~(1 << ECHO_PIN);
}


// main measuring func
float ultrasonic_read(void)
{
    uint32_t time_us;
    float distance_cm;

    // trigger pulse
    TRIG_PORT &= ~(1 << TRIG_PIN);
    delay_us(2);
    TRIG_PORT |=  (1 << TRIG_PIN);
    delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);

    // measure pulse duration
    time_us = pulseIn(1 << ECHO_PIN, 30000);

    // calculate distance in cm
    distance_cm = time_us / 29.1f / 2.0f;

    return distance_cm;
}


/*void ultrasonic()
{
  digitalWrite(trig,0);
  delayMicroseconds(2);
  digitalWrite(trig,1);
  delayMicroseconds(10);
  digitalWrite(trig,0);
  cas = pulseIn(echo,1);

  vzdalenost = cas/29.1/2;
}*/