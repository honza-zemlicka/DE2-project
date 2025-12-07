#ifndef ROBOT_H
#define ROBOT_H

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

#endif