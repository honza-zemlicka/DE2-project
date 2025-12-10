#ifndef ROBOT_H
#define ROBOT_H

#define MOTOR_LF PD5 // Arduino D5 (PWM)
#define MOTOR_RF PD6 // Arduino D6 (PWM)
#define MOTOR_LB PB1 // Arduino D9 (PWM)
#define MOTOR_RB PB2 // Arduino D10 (PWM)

#define SENSOR_RR PC0 // Arduino A0
#define SENSOR_CR PC1 // Arduino A1
#define SENSOR_CL PC2 // Arduino A2
#define SENSOR_LL PC3 // Arduino A3

#define USER_LED PB5 // Arduino D13
#define BUTTON PD3   // Arduino D3, pullup

#define US_TRIG_DDR  DDRB
#define US_TRIG_PORT PORTB
#define US_TRIG_PIN  PB0

#define US_ECHO_DDR  DDRD
#define US_ECHO_PINR PIND
#define US_ECHO_PIN  PD7

#define frame_length 32
// 32-bit + start bit

#define RUN 0x00
#define STOP 0x80
//#define CALIB 0x40
//#define SET 0xc0

#define GAIN_PLUS_1 0x10
#define GAIN_PLUS_10 0x50
#define GAIN_MINUS_1 0x90
#define GAIN_MINUS_10 0xd0

#define SPEED_PLUS_1 0x08
#define SPEED_PLUS_10 0x48
#define SPEED_MINUS_1 0x88
#define SPEED_MINUS_10 0xc8

#endif