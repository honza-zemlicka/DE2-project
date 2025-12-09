#include "robot_definitions.h"
#include <stdio.h>
#include <avr/io.h>

/*
 * Internal helper function to measure the duration of the Echo pulse.
 * Returns the pulse width in approximate microseconds.
 */
static uint32_t pulse_in(uint32_t timeout_limit)
{
    uint32_t width = 0;
    uint32_t loop_count = 0;

    // wait for pulse to start (L -> H)
    // timeout loop to prevent the robot from freezing if the sensor is disconnected.
    while (gpio_read(&US_ECHO_PINR, US_ECHO_PIN) == 0) {
        loop_count++;
        _delay_us(1);
        if (loop_count > timeout_limit) {
            return 0; // sensor did not respond
        }
    }

    // measure H pulse duration
    while (gpio_read(&US_ECHO_PINR, US_ECHO_PIN) == 1) {
        width++;
        _delay_us(1); // delay for stability
        
        if (width > timeout_limit) {
            return 0; //out of range
        }
    }

    return width;
}

void ultrasound_init(void)
{
    // trigger pin as output
    gpio_mode_output(&US_TRIG_DDR, US_TRIG_PIN);

    // echo pin as input
    gpio_mode_input_nopull(&US_ECHO_DDR, US_ECHO_PIN);

    // ensure trigger is L on startup to prevent false trigger
    gpio_write_low(&US_TRIG_PORT, US_TRIG_PIN);
}

uint16_t ultrasound_read(void)
{
    // generate a 10us trigger Pulse
    gpio_write_low(&US_TRIG_PORT, US_TRIG_PIN);
    _delay_us(2);
    gpio_write_high(&US_TRIG_PORT, US_TRIG_PIN);
    _delay_us(10);
    gpio_write_low(&US_TRIG_PORT, US_TRIG_PIN);

    // measure the duration of echo signal
    // timeout set to 30ms
    uint32_t time_us = pulse_in(30000);

    if (time_us == 0) {
        return 0;
    }

    // calculate distance in mm
    // formula derived from speed of sound: (Time_us * 10) / 58
    return (uint16_t)((time_us * 10) / 58);
}