/*
 * File:    config.h
 * Project: sbus2pwm
 * Author:  Igor Morenko <imorenko@yandex.ru>
 * Date:    18.01.2019
 */

#ifndef _CONFIG_H
#define _CONFIG_H

#define CHANNELS_COUNT 16

//#define SET_TIME(x) (channels[x]+2000)
//#define SET_TIME(x) (map(channels[x],0,2047,-150,2400)+2000)
#define SET_TIME(x) (map(channels[x],0,2047,1850,4400))

#define REFRESH_INTERVAL_TICKETS 40000U // minumim time to refresh servos in tickets (20ms)

//////////// Servo Ports ////////////

// 0 on: 1      off: 0xFE
// 1 on: (1<<1) off: 0xFD
// 2 on: (1<<2) off: 0xFB
// 3 on: (1<<3) off: 0xF7
// 4 on: (1<<4) off: 0xEF
// 5 on: (1<<5) off: 0xDF
// 6 on: (1<<6) off: 0xBF
// 7 on: (1<<7) off: 0x7F

#if defined(__AVR_ATmega32U4__)
// ATmega 32u4 (Arduino Leonardo)

// CH13 PF7(ADC7)  = A0   = A0
// CH14 PF6(ADC7)  = A1   = A1
// CH15 PF5(ADC7)  = A2   = A2
// CH16 PF4(ADC7)  = A3   = A3
//      PF1(ADC7)  = A4   = A4       Not Used
//      PF0(ADC7)  = A5   = A5       Not Used
//      PD2(RX)    = RXI  = D0
//      PD3(TX)    = TXO  = D1
// CH1  PD1(SDA)   = D2   = D2
// CH2  PD0(SCL)   = D3   = D3#
// CH3  PD4(ADC8)  = D4   = D4/A6
// CH4  PC6        = D5   = D5#
// CH5  PD7(ADC10) = D6   = D6#/A7
// CH6  PE6        = D7   = D7
// CH7  PB4(ADC11) = D8   = D8
// CH8  PB5(ADC12) = D9   = D9#/A8
// CH9  PB6(ADC13) = D10  = D10#
//      PB7        = D11  = D11#     Not Used
//      PD6(ADC9)  = D12  = D12/A10  Not Used
//      PC7        = D13  = D13#     Not Used
//      PB0(SS)    = D14  = D14/RX LED
// CH12 PB1(SCK)   = SCK  = D15
// CH10  PB2(MOSI) = MOSI = D16
// CH11 PB3(MISO)  = MISO = D17
//      PD5        = ---- = ---/TX LED

#define  CH0_on  PORTD |= (1<<1)  //PD1
#define  CH0_off PORTD &= 0xFD    //PD1

#define  CH1_on  PORTD |= 1       //PD0
#define  CH1_off PORTD &= 0xFE    //PD0

#define  CH2_on  PORTD |= (1<<4)  //PD4
#define  CH2_off PORTD &= 0xEF    //PD4

#define  CH3_on  PORTC |= (1<<6)  //PC6
#define  CH3_off PORTC &= 0xBF    //PC6

#define  CH4_on  PORTD |= (1<<7)  //PD7
#define  CH4_off PORTD &= 0x7F    //PD7

#define  CH5_on  PORTE |= (1<<6)  //PE6
#define  CH5_off PORTE &= 0xBF    //PE6

#define  CH6_on  PORTB |= (1<<4)  //PB4
#define  CH6_off PORTB &= 0xEF    //PB4

#define  CH7_on  PORTB |= (1<<5)  //PB5
#define  CH7_off PORTB &= 0xDF    //PB5

#define  CH8_on  PORTB |= (1<<6)  //PB6
#define  CH8_off PORTB &= 0xBF    //PB6

#define  CH9_on  PORTB |= (1<<2)  //PB2
#define  CH9_off PORTB &= 0xFB    //PB2

#define  CH10_on  PORTB |= (1<<3) //PB3
#define  CH10_off PORTB &= 0xF7   //PB3

#define  CH11_on  PORTB |= (1<<1) //PB1
#define  CH11_off PORTB &= 0xFD   //PB1

#define  CH12_on  PORTF |= (1<<7) //PF7
#define  CH12_off PORTF &= 0x7F   //PF7

#define  CH13_on  PORTF |= (1<<6) //PF6
#define  CH13_off PORTF &= 0xBF   //PF6

#define  CH14_on  PORTF |= (1<<5) //PF5
#define  CH14_off PORTF &= 0xDF   //PF5

#define  CH15_on  PORTF |= (1<<4) //PF4
#define  CH15_off PORTF &= 0xEF   //PF4

// For Test
// RXLED
#define  RXLED_off PORTB |= 1      //PB0
#define  RXLED_on  PORTB &= 0xFE   //PB0
// TXLED
#define  TXLED_off PORTD |= (1<<5) //PD5
#define  TXLED_on  PORTD &= 0xDF   //PD5

#else

// Atmega 168/328 (Arduino Uno, Nano)

// CH1  PD2 = D2
// CH2  PD3 = D3
// CH3  PD4 = D4
// CH4  PD5 = D5
// CH5  PD6 = D6
// CH6  PD7 = D7
// CH7  PB0 = D8
// CH8  PB1 = D9
// CH9  PB2 = D10
// CH10 PB3 = D11
// CH11 PB4 = D12
// CH12 PB5 = D13
// CH13 PC0 = A0
// CH14 PC1 = A1
// CH15 PC2 = A2
// CH16 PC3 = A3

#define  CH0_on  PORTD |= (1<<2)  //D2
#define  CH0_off PORTD &= 0xFB    //D2

#define  CH1_on  PORTD |= (1<<3)  //D3
#define  CH1_off PORTD &= 0xF7    //D3

#define  CH2_on  PORTD |= (1<<4)  //D4
#define  CH2_off PORTD &= 0xEF    //D4

#define  CH3_on  PORTD |= (1<<5)  //D5
#define  CH3_off PORTD &= 0xDF    //D5

#define  CH4_on  PORTD |= (1<<6)  //D6
#define  CH4_off PORTD &= 0xBF    //D6

#define  CH5_on  PORTD |= (1<<7)  //D7
#define  CH5_off PORTD &= 0x7F    //D7

#define  CH6_on  PORTB |= 1       //D8
#define  CH6_off PORTB &= 0xFE    //D8

#define  CH7_on  PORTB |= (1<<1)  //D9
#define  CH7_off PORTB &= 0xFD    //D9

#define  CH8_on  PORTB |= (1<<2)  //D10
#define  CH8_off PORTB &= 0xFB    //D10

#define  CH9_on  PORTB |= (1<<3)  //D11
#define  CH9_off PORTB &= 0xF7    //D11

#define  CH10_on  PORTB |= (1<<4) //D12
#define  CH10_off PORTB &= 0xED   //D12

#define  CH11_on  PORTB |= (1<<5) //D13
#define  CH11_off PORTB &= 0xDF   //D13

#define  CH12_on  PORTC |= 1      //A0
#define  CH12_off PORTC &= 0xFE   //A0

#define  CH13_on  PORTC |= (1<<1) //A1
#define  CH13_off PORTC &= 0xFD   //A1

#define  CH14_on  PORTC |= (1<<2) //A2
#define  CH14_off PORTC &= 0xFB   //A2

#define  CH15_on  PORTC |= (1<<3) //A3
#define  CH15_off PORTC &= 0xF7   //A3

#endif
//////////////////////////////////////

#endif // _CONFIG_H

