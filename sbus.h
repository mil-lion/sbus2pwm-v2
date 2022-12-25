/*
 * File:    sbus.h
 * Project: sbus2pwm
 * Author:  Igor Morenko <imorenko@yandex.ru>
 * Date:    18.01.2019
 */

#ifndef _SBUS_H
#define _SBUS_H

// ********** SBUS ************
#define port Serial1

#define SBUS_BAUDRATE         100000     // 98000
#define SBUS_PORT_OPTIONS     SERIAL_8E2 // (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)

#define ALL_CHANNELS
//#define DIGITAL_CHANNELS

#define SBUS_MAX_CHANNELS     18
#define SBUS_FRAME_SIZE       25

//#define SBUS_FRAME_BEGIN_BYTE 0x0F
#define SBUS_START_BYTE       0x0F
#define SBUS_END_BYTE         0x00

#define SBUS_DIGITAL_CHANNEL_MIN   200  //MIN_PULSE_WIDTH
#define SBUS_DIGITAL_CHANNEL_MAX   1800 //MAX_PULSE_WIDTH

#define SBUS_SIGNAL_OK             0x00
#define SBUS_SIGNAL_LOST           0x01
#define SBUS_SIGNAL_FAILSAFE       0x03

#define SBUS_STATE_FAILSAFE        (1 << 0)
#define SBUS_STATE_SIGNALLOSS      (1 << 1)

#define SBUS_FLAG_CHANNEL_17       (1 << 0)
#define SBUS_FLAG_CHANNEL_18       (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS      (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE  (1 << 3)

struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int ch0 : 11;
    unsigned int ch1 : 11;
    unsigned int ch2 : 11;
    unsigned int ch3 : 11;
    unsigned int ch4 : 11;
    unsigned int ch5 : 11;
    unsigned int ch6 : 11;
    unsigned int ch7 : 11;
    unsigned int ch8 : 11;
    unsigned int ch9 : 11;
    unsigned int ch10 : 11;
    unsigned int ch11 : 11;
    unsigned int ch12 : 11;
    unsigned int ch13 : 11;
    unsigned int ch14 : 11;
    unsigned int ch15 : 11;
    uint8_t flags;
    /**
     * The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;

#endif // _SBUS_H

